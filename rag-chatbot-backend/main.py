import os
import requests
import uuid
import hashlib
from datetime import datetime, timedelta
from typing import Optional

from fastapi import FastAPI, HTTPException, Depends, Form
from fastapi.middleware.cors import CORSMiddleware
from fastapi.security import OAuth2PasswordBearer
from pydantic import BaseModel, Field
from jose import JWTError, jwt
from passlib.context import CryptContext
from dotenv import load_dotenv
import psycopg

from qdrant_client import QdrantClient
from fastembed import TextEmbedding

load_dotenv()

# --------------------------------------------------
# JWT CONFIG
# --------------------------------------------------

SECRET_KEY = os.getenv(
    "SECRET_KEY",
    "axo7GSAqFOOJ7kOOYU63a0FQtHyMNTc-Q_QqGEdPlWQHcvFdPvd5pTvucpY5j6m69t1LAMxsDDWCunQVCjYbMg",
)
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# --------------------------------------------------
# PASSWORD HASHING (SAFE)
# --------------------------------------------------

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

def _prehash_password(password: str) -> str:
    return hashlib.sha256(password.encode("utf-8")).hexdigest()

def get_password_hash(password: str) -> str:
    return pwd_context.hash(_prehash_password(password))

def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(
        _prehash_password(plain_password),
        hashed_password
    )

# --------------------------------------------------
# FASTAPI APP
# --------------------------------------------------

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")
app = FastAPI()

# --------------------------------------------------
# CORS
# --------------------------------------------------

frontend_url = os.getenv("FRONTEND_URL", "http://localhost:3000")
render_url = os.getenv("RENDER_EXTERNAL_URL", "https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com")
vercel_url = os.getenv("VERCEL_URL", "https://physical-ai-humanoid-robotics-textb-fawn.vercel.app")  # Default Vercel URL

# Build allowed origins list
allowed_origins = [
    frontend_url,
    render_url,
    vercel_url,  # Vercel frontend
    "http://localhost:3000",  # Local development
    "http://127.0.0.1:3000",  # Alternative local development
    "https://physical-ai-humanoid-robotics-textbook-fcve.onrender.com",  # Production Render URL
    "https://physical-ai-humanoid-robotics-textb-fawn.vercel.app",  # Production Vercel URL
    "http://localhost:3001",  # Additional local development port
    "http://127.0.0.1:3001",  # Alternative local development port
]

# Add any additional origins from environment variable (comma-separated)
additional_origins = os.getenv("ADDITIONAL_CORS_ORIGINS", "")
if additional_origins:
    for origin in additional_origins.split(","):
        origin = origin.strip()
        if origin:
            allowed_origins.append(origin)

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --------------------------------------------------
# QDRANT
# --------------------------------------------------

qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)
embedding_model = TextEmbedding()
COLLECTION_NAME = "textbook_collection"

# --------------------------------------------------
# DATABASE
# --------------------------------------------------

NEON_DB_URL = os.getenv("NEON_DATABASE_URL")

async def get_db_connection():
    if not NEON_DB_URL:
        raise RuntimeError("Database URL not configured")
    return await psycopg.AsyncConnection.connect(NEON_DB_URL)

# --------------------------------------------------
# MODELS
# --------------------------------------------------

class UserCreate(BaseModel):
    email: str
    password: str = Field(min_length=8, max_length=128)
    full_name: str

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    email: Optional[str] = None

class ChatRequest(BaseModel):
    query: str
    selected_text: Optional[str] = None

class TranslateTextRequest(BaseModel):
    text: str
    target_language: str = "ur"

class TranslateMarkdownRequest(BaseModel):
    markdown_content: str
    target_language: str = "ur"

# --------------------------------------------------
# USER DB OPS
# --------------------------------------------------

async def get_user_by_email(email: str):
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute(
                "SELECT id, email, full_name, hashed_password FROM users WHERE email = %s",
                (email,),
            )
            row = await cur.fetchone()
            if row:
                return {
                    "id": row[0],
                    "email": row[1],
                    "full_name": row[2],
                    "hashed_password": row[3],
                }
    return None

async def create_user(email: str, full_name: str, password: str):
    user_id = str(uuid.uuid4())
    hashed_password = get_password_hash(password)

    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute(
                """
                INSERT INTO users (id, email, full_name, hashed_password)
                VALUES (%s, %s, %s, %s)
                """,
                (user_id, email, full_name, hashed_password),
            )
            await conn.commit()

    return {"id": user_id, "email": email, "full_name": full_name}

async def authenticate_user(email: str, password: str):
    user = await get_user_by_email(email)
    if not user:
        return False
    if not verify_password(password, user["hashed_password"]):
        return False
    return user

# --------------------------------------------------
# JWT
# --------------------------------------------------

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    expire = datetime.utcnow() + (expires_delta or timedelta(minutes=15))
    to_encode.update({"exp": expire})
    return jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)

async def get_current_user(token: str = Depends(oauth2_scheme)):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        email = payload.get("sub")
        if not email:
            raise HTTPException(status_code=401)
    except JWTError:
        raise HTTPException(status_code=401)

    user = await get_user_by_email(email)
    if not user:
        raise HTTPException(status_code=401)
    return user

# --------------------------------------------------
# STARTUP
# --------------------------------------------------

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
                    ALTER TABLE chat_history ADD COLUMN user_id VARCHAR(36);
                """)
            except Exception:
                # Column might already exist, continue
                pass
            await conn.commit()

        # Add foreign key constraint separately to avoid circular dependency
        try:
            async with conn.cursor() as cur:
                await cur.execute("""
                    ALTER TABLE chat_history
                    ADD CONSTRAINT fk_chat_history_user_id
                    FOREIGN KEY (user_id) REFERENCES users(id);
                """)
        except Exception as e:
            # Foreign key might already exist or users table not ready, continue
            print(f"Could not create foreign key constraint: {e}")
            pass

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
                    user_id VARCHAR(36)
                );
            """)
            await conn.commit()

@app.on_event("startup")
async def startup_event():
    try:
        # Create users table first (no dependencies)
        await create_auth_tables()
        # Then create other tables that might reference users
        await create_chat_history_table()
        await create_translation_cache_table()
        print("Database tables created successfully")
    except Exception as e:
        print(f"Error during startup: {e}")
        # Don't raise the exception to avoid crashing the server, but log it

# --------------------------------------------------
# ROUTES
# --------------------------------------------------

@app.get("/")
async def root():
    return {
        "message": "Welcome to the Physical AI Humanoid Robotics Textbook RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": {
            "GET /health": "Health check with database connectivity",
            "POST /register": "User registration",
            "POST /token": "User login and token generation",
            "GET /users/me": "Get current user info (requires authentication)",
            "POST /chat": "Chat with the RAG system (requires authentication)",
            "POST /translate": "Translate text (requires authentication)",
            "POST /translate-markdown": "Translate markdown content (requires authentication)",
        },
        "description": "This API provides access to a RAG chatbot for the Physical AI Humanoid Robotics Textbook, with authentication, translation capabilities, and chat functionality."
    }

@app.get("/health")
async def health_check():
    """Health check endpoint that also tests database connectivity"""
    try:
        # Test database connection
        async with await get_db_connection() as conn:
            async with conn.cursor() as cur:
                await cur.execute("SELECT 1")
                result = await cur.fetchone()

        return {
            "status": "healthy",
            "database": "connected",
            "message": "API is running and database is accessible"
        }
    except Exception as e:
        print(f"Health check failed: {e}")
        return {
            "status": "unhealthy",
            "database": "disconnected",
            "error": str(e),
            "message": "API is running but database connection failed"
        }

@app.post("/register", response_model=Token)
async def register(user: UserCreate):
    if await get_user_by_email(user.email):
        raise HTTPException(status_code=400, detail="Email already registered")

    db_user = await create_user(user.email, user.full_name, user.password)
    token = create_access_token(
        {"sub": db_user["email"]},
        timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES),
    )
    return {"access_token": token, "token_type": "bearer"}

@app.post("/token", response_model=Token)
async def login(username: str = Form(...), password: str = Form(...)):
    user = await authenticate_user(username, password)
    if not user:
        raise HTTPException(status_code=401, detail="Invalid credentials")

    token = create_access_token(
        {"sub": user["email"]},
        timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES),
    )
    return {"access_token": token, "token_type": "bearer"}

@app.get("/users/me")
async def users_me(current_user: dict = Depends(get_current_user)):
    return current_user

# --- Chat Endpoint ---
@app.post("/chat")
async def chat(chat_request: ChatRequest, current_user: dict = Depends(get_current_user)):
    try:
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
        result = await process_user_query(user_id, chat_request.query, context)
        return result
    except ImportError as e:
        return {"error": f"Chat functionality not available: {str(e)}"}
    except Exception as e:
        return {"error": f"Chat processing failed: {str(e)}"}

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
