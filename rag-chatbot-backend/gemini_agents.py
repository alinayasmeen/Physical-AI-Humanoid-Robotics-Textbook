from dotenv import load_dotenv
import os
from openai import OpenAI
from typing import Optional
import uuid
from datetime import datetime
import psycopg
from fastapi import HTTPException
from qdrant_client import QdrantClient
from fastembed import TextEmbedding

load_dotenv()

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
NEON_DB_URL = os.getenv("NEON_DATABASE_URL")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "textbook_collection"

# Check if the API key is set; if not, raise an error
if not GEMINI_API_KEY:
    raise ValueError("GEMINI_API_KEY environment variable is not set. Please set it before running the script.")

# Initialize clients
def get_gemini_client():
    return OpenAI(
        api_key=GEMINI_API_KEY,
        base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    )

def get_qdrant_client():
    return QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

embedding_model = TextEmbedding()

async def get_db_connection():
    return await psycopg.AsyncConnection.connect(NEON_DB_URL)

async def save_chat_history(user_id: str, query: str, response: str):
    """Save chat history to the database with user association"""
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute(
                "INSERT INTO chat_history (query, response, timestamp, user_id) VALUES (%s, %s, NOW(), %s)",
                (query, response, user_id)
            )
            await conn.commit()

async def get_user_by_id(user_id: str):
    """Get user information by ID"""
    async with await get_db_connection() as conn:
        async with conn.cursor() as cur:
            await cur.execute(
                "SELECT id, email, full_name FROM users WHERE id = %s",
                (user_id,)
            )
            result = await cur.fetchone()
            if result:
                return {
                    "id": result[0],
                    "email": result[1],
                    "full_name": result[2]
                }
    return None

async def retrieve_context(query: str):
    """Retrieve context from Qdrant database using embeddings"""
    try:
        query_embedding = list(embedding_model.embed([query]))[0]  # Convert generator to list and get first element
        qdrant_client = get_qdrant_client()
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding.tolist(),
            limit=5,
        )
        # Handle potential different response formats
        if hasattr(search_result, '__iter__'):
            context = " ".join([hit.payload.get("text", "") if hasattr(hit, 'payload') and hit.payload else "" for hit in search_result if hasattr(hit, 'payload')])
        else:
            context = ""
        return context
    except Exception as e:
        print(f"Error retrieving context: {str(e)}")
        return ""

async def create_gemini_chat_completion(user_id: str, query: str, context: Optional[str] = None):
    """
    Create a chat completion using Gemini API through OpenAI SDK
    """
    try:
        # If no context is provided, retrieve it from the RAG system
        if not context:
            context = await retrieve_context(query)

        # Prepare the message content
        if context:
            system_message = f"You are a helpful assistant for a textbook. Here is the relevant context: {context}"
        else:
            system_message = "You are a helpful assistant for a textbook."

        messages = [
            {"role": "system", "content": system_message},
            {"role": "user", "content": query}
        ]

        # Call the Gemini API using OpenAI SDK
        client = get_gemini_client()
        response = client.chat.completions.create(
            model="gemini-2.0-flash",
            messages=messages,
            temperature=0.7,
            max_tokens=1000
        )

        # Extract the response text
        response_text = response.choices[0].message.content

        # Save to chat history
        await save_chat_history(user_id, query, response_text)

        return {"response": response_text}

    except Exception as e:
        error_msg = str(e)
        print(f"Error in Gemini chat completion: {error_msg}")

        # Check if it's a quota error and provide a more user-friendly message
        if "quota" in error_msg.lower() or "429" in error_msg or "RESOURCE_EXHAUSTED" in error_msg:
            fallback_response = "I'm currently experiencing high demand and have reached my usage limits. Please try again later or check with the administrator about API quotas."
            # Still save to chat history even with fallback response
            await save_chat_history(user_id, query, fallback_response)
            return {"response": fallback_response}
        else:
            raise HTTPException(status_code=500, detail=f"Error calling Gemini API: {error_msg}")

# Example usage function
async def process_user_query(user_id: str, query: str, context: Optional[str] = None):
    """
    Process a user query using Gemini API and save to chat history
    """
    return await create_gemini_chat_completion(user_id, query, context)