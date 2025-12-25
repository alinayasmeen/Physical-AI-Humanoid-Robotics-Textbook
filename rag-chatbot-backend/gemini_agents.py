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

        # Use the correct QdrantClient search method based on version
        # In newer versions, search is the correct method
        # In older versions, it might be search_points or a different API
        if hasattr(qdrant_client, 'search'):
            search_result = qdrant_client.search(
                collection_name=COLLECTION_NAME,
                query_vector=query_embedding.tolist(),
                limit=5,
            )
        elif hasattr(qdrant_client, 'search_points'):
            # Fallback for older versions
            search_result = qdrant_client.search_points(
                collection_name=COLLECTION_NAME,
                query=query_embedding.tolist(),
                limit=5,
            )
        else:
            print("QdrantClient does not have search or search_points method")
            return ""

        # Extract text from the search results
        context = ""
        if search_result:
            # Handle different response formats
            if hasattr(search_result, '__iter__') and not isinstance(search_result, str):
                for hit in search_result:
                    if hasattr(hit, 'payload') and hit.payload and 'text' in hit.payload:
                        context += hit.payload['text'] + " "
            else:
                # Handle other response formats if needed
                pass

        return context.strip()
    except AttributeError as e:
        # Handle potential attribute error if search method doesn't exist
        print(f"QdrantClient search method error: {str(e)}")
        return ""
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
            model="models/gemini-2.5-flash",
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
        if "quota" in error_msg.lower() or "429" in error_msg or "RESOURCE_EXHAUSTED" in error_msg or "generate_content_free_tier" in error_msg:
            # Provide a more informative fallback response
            fallback_response = (
                "I'm currently experiencing high demand and have reached my usage limits for the free tier. "
                "Please try again later, consider upgrading the API plan, or contact the administrator. "
                "As an alternative, you can try reducing the frequency of requests or check if the information you need is available in the textbook content directly."
            )
            # Still save to chat history even with fallback response
            await save_chat_history(user_id, query, fallback_response)
            return {"response": fallback_response}
        else:
            # For other types of errors, provide a generic error message
            generic_error_response = (
                "I'm currently experiencing technical difficulties. "
                "Please try again later or contact the administrator for assistance."
            )
            # Still save to chat history even with error response
            await save_chat_history(user_id, query, generic_error_response)
            return {"response": generic_error_response}

# Example usage function
async def process_user_query(user_id: str, query: str, context: Optional[str] = None):
    """
    Process a user query using Gemini API and save to chat history
    """
    return await create_gemini_chat_completion(user_id, query, context)