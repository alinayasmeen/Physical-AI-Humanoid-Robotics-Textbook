import os
import uuid
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
from fastembed import TextEmbedding

load_dotenv()

# Initialize Qdrant client
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Initialize the embedding model
embedding_model = TextEmbedding()
VECTOR_SIZE = 384  # This is the size for the default "BAAI/bge-small-en" model

COLLECTION_NAME = "textbook_collection"
TEXTBOOK_PATH = "../my-textbook-site/course"

def get_markdown_files(path):
    """
    Recursively gets all markdown files from a directory.
    """
    markdown_files = []
    for root, _, files in os.walk(path):
        for file in files:
            if file.endswith(".md"):
                markdown_files.append(os.path.join(root, file))
    return markdown_files

def chunk_text(text, chunk_size=1000, overlap=200):
    """
    Splits text into chunks of a specific size with overlap.
    """
    chunks = []
    for i in range(0, len(text), chunk_size - overlap):
        chunks.append(text[i:i + chunk_size])
    return chunks

def main():
    """
    Ingests the textbook data into Qdrant.
    """
    # Recreate the collection with the correct vector size for the fastembed model
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
    )

    markdown_files = get_markdown_files(TEXTBOOK_PATH)
    documents = []
    for file_path in markdown_files:
        with open(file_path, "r", encoding="utf-8") as f:
            documents.append(f.read())

    # Chunk the documents
    chunks = list(set(chunk for doc in documents for chunk in chunk_text(doc) if chunk.strip()))

    # Generate embeddings in batches
    batch_size = 32
    for i in range(0, len(chunks), batch_size):
        batch_chunks = chunks[i:i + batch_size]
        
        # Generate embeddings for the batch
        embeddings = embedding_model.embed(batch_chunks)
        
        # Create points for upserting
        points = [
            models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding.tolist(),
                payload={"text": chunk},
            )
            for chunk, embedding in zip(batch_chunks, embeddings)
        ]
        
        # Upsert points to Qdrant
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points,
            wait=True,
        )
        print(f"Successfully ingested batch {i // batch_size + 1}/{(len(chunks) + batch_size - 1) // batch_size}")

    print(f"Successfully ingested {len(chunks)} points into Qdrant.")

if __name__ == "__main__":
    main()
