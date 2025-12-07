import os
from fastapi import FastAPI
from pydantic import BaseModel
from qdrant_client import QdrantClient
from dotenv import load_dotenv
from fastembed import TextEmbedding
import openai # pyright: ignore[reportMissingImports]
import psycopg # Add this import for Neon Postgres

load_dotenv()

# Initialize FastAPI app
app = FastAPI()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Initialize the embedding model
embedding_model = TextEmbedding()

# Initialize OpenAI client for response generation
openai.api_key = os.getenv("GEMINI_API_KEY")
OPENAI_CHAT_MODEL = "gemini-2.5-flash" # Or "gpt-4" or similar

COLLECTION_NAME = "textbook_collection"

# --- Neon Postgres Integration Start ---
# Initialize Neon Postgres connection
NEON_DB_URL = os.getenv("NEON_DATABASE_URL")
if not NEON_DB_URL:
    raise ValueError("NEON_DATABASE_URL environment variable not set.")

# Function to get an asynchronous database connection
async def get_db_connection():
    """
    Establishes and returns an asynchronous connection to the Neon Postgres database.
    """
    try:
        conn = await psycopg.AsyncConnection.connect(NEON_DB_URL)
        return conn
    except Exception as e:
        print(f"Failed to connect to Neon DB: {e}")
        raise # Re-raise the exception to indicate connection failure

@app.get("/test-neon-db")
async def test_neon_db():
    """
    Tests the connection to the Neon Postgres database by fetching its version.
    """
    try:
        async with await get_db_connection() as conn:
            async with conn.cursor() as cur:
                await cur.execute("SELECT version();")
                version = await cur.fetchone()
                return {"message": f"Successfully connected to Neon DB. PostgreSQL version: {version[0]}"}
    except Exception as e:
        return {"message": f"Failed to connect to Neon DB: {e}"}
# --- Neon Postgres Integration End ---

class ChatRequest(BaseModel):
    query: str
    selected_text: str = None

@app.post("/chat")
async def chat(chat_request: ChatRequest):
    """
    Handles a chat request, retrieving context from Qdrant or selected text,
    and generating a response using OpenAI.
    """
    if chat_request.selected_text:
        # If text is selected, use it as context
        context = chat_request.selected_text
    else:
        # Otherwise, retrieve context from Qdrant
        query_embedding = embedding_model.embed([chat_request.query])[0]

        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding.tolist(),
            limit=5,
        )
        context = " ".join([hit.payload["text"] for hit in search_result])

    # Generate a response using OpenAI
    try:
        response = openai.chat.completions.create(
            model=OPENAI_CHAT_MODEL,
            messages=[
                {"role": "system", "content": "You are a helpful assistant for a textbook. Answer questions based on the provided context."},
                {"role": "user", "content": f"Context:\n{context}\n\nQuery:\n{chat_request.query}"}
            ]
        )
        response_text = response.choices[0].message.content

        async with await get_db_connection() as conn:
            async with conn.cursor() as cur:
                await cur.execute(
                    """
                    INSERT INTO chat_history (query, response, timestamp)
                    VALUES (%s, %s, NOW())
                    """,
                    (chat_request.query, response_text)
                )
                await conn.commit()

        return {"response": response_text}
    except Exception as e:
        # Handle cases where content is blocked or other API errors
        return {"response": f"Sorry, I couldn't generate a response. Error: {e}"}


@app.post("/ingest")
async def ingest_data():
    """
    Triggers the ingestion process.
    """
    import subprocess
    import sys
    try:
        subprocess.Popen([sys.executable, "ingest.py"])
        return {"message": "Ingestion process started in the background."}
    except Exception as e:
        return {"message": f"Failed to start ingestion process: {e}"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)