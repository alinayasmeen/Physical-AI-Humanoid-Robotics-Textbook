import os
import google.generativeai as genai
from fastapi import FastAPI
from pydantic import BaseModel
from qdrant_client import QdrantClient
from dotenv import load_dotenv
from fastembed import TextEmbedding

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

# Initialize Gemini client for response generation
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))
model = genai.GenerativeModel('gemini-1.5-flash')

COLLECTION_NAME = "textbook_collection"

class ChatRequest(BaseModel):
    query: str
    selected_text: str = None

@app.post("/chat")
async def chat(chat_request: ChatRequest):
    """
    Handles a chat request.
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

    # Generate a response using Gemini
    try:
        response = model.generate_content(
            f"Based on the following context, answer the user's query.\n\nContext:\n{context}\n\nQuery:\n{chat_request.query}"
        )
        return {"response": response.text}
    except Exception as e:
        # Handle cases where content is blocked or other API errors
        return {"response": f"Sorry, I couldn't generate a response. Error: {e}"}


@app.post("/ingest")
async def ingest_data():
    """
    Triggers the ingestion process.
    """
    # This endpoint can be used to manually trigger the ingestion script.
    # In a real-world scenario, this might be a protected endpoint.
    import subprocess
    import sys
    try:
        # It's better to run this as a background task in a real app
        subprocess.Popen([sys.executable, "ingest.py"])
        return {"message": "Ingestion process started in the background."}
    except Exception as e:
        return {"message": f"Failed to start ingestion process: {e}"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
