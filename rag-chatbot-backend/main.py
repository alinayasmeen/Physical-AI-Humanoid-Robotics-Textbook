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
render_url = os.getenv("RENDER_EXTERNAL_URL")
vercel_url = os.getenv("VERCEL_URL")

allowed_origins = [
    frontend_url,
    render_url,
    vercel_url,
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=[o for o in allowed_origins if o],
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

@app.on_event("startup")
async def startup_event():
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute("""
                CREATE TABLE IF NOT EXISTS users (
                    id VARCHAR(36) PRIMARY KEY,
                    email VARCHAR(255) UNIQUE NOT NULL,
                    full_name VARCHAR(255) NOT NULL,
                    hashed_password VARCHAR(255) NOT NULL,
                    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            await conn.commit()

# --------------------------------------------------
# ROUTES
# --------------------------------------------------

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
