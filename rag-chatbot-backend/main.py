import os
import requests
from fastapi import FastAPI, HTTPException, Depends, Form
from pydantic import BaseModel
from qdrant_client import QdrantClient
from fastembed import TextEmbedding
from dotenv import load_dotenv
import psycopg
from fastapi.middleware.cors import CORSMiddleware
from passlib.context import CryptContext
from datetime import datetime, timedelta
from typing import Optional
import uuid
from jose import JWTError, jwt
from fastapi.security import OAuth2PasswordBearer

load_dotenv()

# JWT Configuration
SECRET_KEY = os.getenv("SECRET_KEY", "axo7GSAqFOOJ7kOOYU63a0FQtHyMNTc-Q_QqGEdPlWQHcvFdPvd5pTvucpY5j6m69t1LAMxsDDWCunQVCjYbMg")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# Password hashing
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# OAuth2 scheme
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Qdrant ---
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)
embedding_model = TextEmbedding()
COLLECTION_NAME = "textbook_collection"

# --- Gemini API ---
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
GEMINI_MODEL = "gemini-2.0-flash"  # or gemini-2.5-flash

# --- Neon Postgres ---
NEON_DB_URL = os.getenv("NEON_DATABASE_URL")

# --- Authentication Models ---
class UserCreate(BaseModel):
    email: str
    password: str
    full_name: str

class UserLogin(BaseModel):
    email: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: Optional[str] = None

# --- Authentication Functions ---
def verify_password(plain_password, hashed_password):
    # Bcrypt has a 72 character limit, so we truncate if necessary
    if len(plain_password) > 72:
        plain_password = plain_password[:72]
    try:
        return pwd_context.verify(plain_password, hashed_password)
    except ValueError as e:
        if "password cannot be longer than 72 bytes" in str(e):
            # This should not happen due to our truncation, but just in case
            return pwd_context.verify(plain_password[:72], hashed_password)
        else:
            raise e

def get_password_hash(password):
    # Bcrypt has a 72 character limit, so we truncate if necessary
    if len(password) > 72:
        password = password[:72]
    try:
        return pwd_context.hash(password)
    except ValueError as e:
        if "password cannot be longer than 72 bytes" in str(e):
            # This should not happen due to our truncation, but just in case
            return pwd_context.hash(password[:72])
        else:
            raise e

async def get_user_by_email(email: str):
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute(
                "SELECT id, email, full_name, hashed_password FROM users WHERE email = %s",
                (email,)
            )
            result = await cur.fetchone()
            if result:
                return {
                    "id": result[0],
                    "email": result[1],
                    "full_name": result[2],
                    "hashed_password": result[3]
                }
    return None

async def create_user(email: str, full_name: str, password: str):
    hashed_password = get_password_hash(password)
    user_id = str(uuid.uuid4())
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute(
                "INSERT INTO users (id, email, full_name, hashed_password) VALUES (%s, %s, %s, %s)",
                (user_id, email, full_name, hashed_password)
            )
            await conn.commit()
    return {"id": user_id, "email": email, "full_name": full_name}

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

async def authenticate_user(email: str, password: str):
    user = await get_user_by_email(email)
    if not user or not verify_password(password, user["hashed_password"]):
        return False
    return user

async def get_current_user(token: str = Depends(oauth2_scheme)):
    credentials_exception = HTTPException(
        status_code=401,
        detail="Could not validate credentials",
        headers={"WWW-Authenticate": "Bearer"},
    )
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email: str = payload.get("sub")
        if email is None:
            raise credentials_exception
        token_data = TokenData(email=email)
    except JWTError:
        raise credentials_exception
    user = await get_user_by_email(email=token_data.email)
    if user is None:
        raise credentials_exception
    return user

import hashlib

async def get_db_connection():
    return await psycopg.AsyncConnection.connect(NEON_DB_URL)


def get_content_hash(content: str) -> str:
    """Generate a hash for the content to use as a cache key"""
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


async def get_cached_translation(content: str, target_language: str = "ur"):
    """Get cached translation from database if it exists"""
    content_hash = get_content_hash(content)
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute("""
                SELECT translated_content
                FROM translation_cache
                WHERE original_content_hash = %s AND target_language = %s
            """, (content_hash, target_language))
            result = await cur.fetchone()
            if result:
                return result[0]  # Return the cached translation
    return None


async def cache_translation(original_content: str, translated_content: str, target_language: str = "ur"):
    """Cache the translation in the database"""
    content_hash = get_content_hash(original_content)
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute("""
                INSERT INTO translation_cache (original_content_hash, original_content, translated_content, target_language)
                VALUES (%s, %s, %s, %s)
                ON CONFLICT (original_content_hash)
                DO UPDATE SET
                    translated_content = EXCLUDED.translated_content,
                    updated_at = CURRENT_TIMESTAMP
            """, (content_hash, original_content, translated_content, target_language))
            await conn.commit()

async def create_auth_tables():
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            # Create users table
            await cur.execute("""
                CREATE TABLE IF NOT EXISTS users (
                    id VARCHAR(36) PRIMARY KEY,
                    email VARCHAR(255) UNIQUE NOT NULL,
                    full_name VARCHAR(255) NOT NULL,
                    hashed_password VARCHAR(255) NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            # Update chat_history table to include user_id
            try:
                # Add user_id column if it doesn't exist
                await cur.execute("""
                    ALTER TABLE chat_history ADD COLUMN user_id VARCHAR(36) REFERENCES users(id);
                """)
            except psycopg.Error:
                # Column might already exist, continue
                pass
            await conn.commit()

async def create_translation_cache_table():
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute("""
                CREATE TABLE IF NOT EXISTS translation_cache (
                    id SERIAL PRIMARY KEY,
                    original_content_hash VARCHAR(64) UNIQUE NOT NULL,
                    original_content TEXT NOT NULL,
                    translated_content TEXT NOT NULL,
                    target_language VARCHAR(10) DEFAULT 'ur',
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            # Create index for faster lookups by hash
            await cur.execute("""
                CREATE INDEX IF NOT EXISTS idx_translation_cache_hash
                ON translation_cache(original_content_hash);
            """)
            await conn.commit()


async def create_chat_history_table():
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute("""
                CREATE TABLE IF NOT EXISTS chat_history (
                    id SERIAL PRIMARY KEY,
                    query TEXT NOT NULL,
                    response TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                    user_id VARCHAR(36) REFERENCES users(id)
                );
            """)
            await conn.commit()

@app.on_event("startup")
async def startup_event():
    await create_chat_history_table()
    await create_auth_tables()
    await create_translation_cache_table()

# --- Chat Request Model ---
class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

# --- Authentication Endpoints ---
@app.post("/register", response_model=Token)
async def register(user: UserCreate):
    # Check if user already exists
    existing_user = await get_user_by_email(user.email)
    if existing_user:
        raise HTTPException(
            status_code=400,
            detail="Email already registered"
        )

    # Create new user
    db_user = await create_user(user.email, user.full_name, user.password)

    # Create access token
    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": db_user["email"]}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

@app.post("/token", response_model=Token)
async def login_for_access_token(username: str = Form(...), password: str = Form(...)):
    # The frontend sends 'username' but it's actually the email
    user_data = await authenticate_user(username, password)
    if not user_data:
        raise HTTPException(
            status_code=401,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user_data["email"]}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}

@app.get("/users/me")
async def read_users_me(current_user: dict = Depends(get_current_user)):
    return current_user

# --- Translation Request Models ---
class TranslateTextRequest(BaseModel):
    text: str
    target_language: str = "ur"

class TranslateMarkdownRequest(BaseModel):
    markdown_content: str
    target_language: str = "ur"

# --- Translation Endpoints ---
@app.post("/translate")
async def translate_text(request: TranslateTextRequest, current_user: dict = Depends(get_current_user)):
    """Translate text to the specified language (default: Urdu)"""
    try:
        # Check which translation service to use based on environment variables
        if os.getenv("QWEN_API_KEY"):
            from qwen_translation_service import qwen_translation_service
            translation_service = qwen_translation_service
        else:
            from translation_service import translation_service

        translated_text = await translation_service.translate_text(request.text, request.target_language)
        return {"original_text": request.text, "translated_text": translated_text, "target_language": request.target_language}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")


@app.post("/translate-markdown")
async def translate_markdown(request: TranslateMarkdownRequest, current_user: dict = Depends(get_current_user)):
    """Translate markdown content while preserving structure"""
    try:
        # Check which translation service to use based on environment variables
        if os.getenv("QWEN_API_KEY"):
            from qwen_translation_service import qwen_translation_service
            translation_service = qwen_translation_service
        else:
            from translation_service import translation_service

        translated_content = await translation_service.translate_markdown_content(request.markdown_content)
        return {"original_content": request.markdown_content, "translated_content": translated_content, "target_language": request.target_language}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Markdown translation failed: {str(e)}")


# --- Chat Endpoint ---
@app.post("/chat")
async def chat(chat_request: ChatRequest, current_user: dict = Depends(get_current_user)):
    # Import the new gemini agents module only when needed to avoid startup issues
    from gemini_agents import process_user_query

    # Get context - either from selected text or let the agent handle RAG
    context = chat_request.selected_text  # This will be None if no selected text

    # Get the user ID from email
    user = await get_user_by_email(current_user["email"])
    if not user:
        raise HTTPException(status_code=401, detail="User not found")
    user_id = user["id"]

    # Process the query using the new Gemini agent integration
    try:
        result = await process_user_query(user_id, chat_request.query, context)
        return result
    except Exception as e:
        return {"error": str(e)}
