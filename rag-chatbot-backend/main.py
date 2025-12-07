import os
import requests
from fastapi import FastAPI
from pydantic import BaseModel
from qdrant_client import QdrantClient
from fastembed import TextEmbedding
from dotenv import load_dotenv
import psycopg
from fastapi.middleware.cors import CORSMiddleware

load_dotenv()

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
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

async def get_db_connection():
    return await psycopg.AsyncConnection.connect(NEON_DB_URL)

async def create_chat_history_table():
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute("""
                CREATE TABLE IF NOT EXISTS chat_history (
                    id SERIAL PRIMARY KEY,
                    query TEXT NOT NULL,
                    response TEXT NOT NULL,
                    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                );
            """)
            await conn.commit()

@app.on_event("startup")
async def startup_event():
    await create_chat_history_table()

# --- Chat Request Model ---
class ChatRequest(BaseModel):
    query: str
    selected_text: str = None

# --- Chat Endpoint ---
@app.post("/chat")
async def chat(chat_request: ChatRequest):
    # Step 1: Get context
    if chat_request.selected_text:
        context = chat_request.selected_text
    else:
        query_embedding = embedding_model.embed([chat_request.query])[0]
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding.tolist(),
            limit=5,
        )
        context = " ".join([hit.payload["text"] for hit in search_result])

    # Step 2: Call Gemini API inside try/except
    try:
        url = f"https://generativelanguage.googleapis.com/v1beta2/models/{GEMINI_MODEL}:generateMessage?key={GEMINI_API_KEY}"
        headers = {"Content-Type": "application/json"}
        data = {
           "input": {
        "messages": [
            {"author": "system", "content": {"text": "You are a helpful assistant for a textbook."}},
            {"author": "user", "content": {"text": f"Context:\n{context}\n\nQuery:\n{chat_request.query}"}}
            ]
        }
        }

        response = requests.post(url, headers=headers, json=data)
        response.raise_for_status()
        gemini_response = response.json()
        response_text = gemini_response["candidates"][0]["content"][0]["text"]

        # Step 3: Save chat history in Neon DB
        async with await get_db_connection() as conn:
            async with conn.cursor() as cur:
                await cur.execute(
                    "INSERT INTO chat_history (query, response, timestamp) VALUES (%s, %s, NOW())",
                    (chat_request.query, response_text)
                )
                await conn.commit()

        return {"response": response_text}

    except requests.HTTPError as http_err:
        return {"error": f"HTTP error occurred: {http_err}"}
    except Exception as e:
        return {"error": str(e)}
