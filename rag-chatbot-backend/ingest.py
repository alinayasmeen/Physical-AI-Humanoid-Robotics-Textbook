import os
import uuid
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv
from fastembed import TextEmbedding

load_dotenv()

qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

embedding_model = TextEmbedding()
VECTOR_SIZE = 384
COLLECTION_NAME = "textbook_collection"
TEXTBOOK_PATH = "../my-textbook-site/course"

def get_markdown_files(path):
    files = []
    for root, _, filenames in os.walk(path):
        for f in filenames:
            if f.endswith(".md"):
                files.append(os.path.join(root, f))
    return files

def chunk_text(text, chunk_size=1000, overlap=200):
    chunks = []
    for i in range(0, len(text), chunk_size - overlap):
        chunks.append(text[i:i + chunk_size])
    return chunks

def main():
    qdrant_client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
    )

    files = get_markdown_files(TEXTBOOK_PATH)
    documents = []
    for f in files:
        with open(f, "r", encoding="utf-8", errors="ignore") as file:
            documents.append(file.read())

    chunks = list(set(chunk for doc in documents for chunk in chunk_text(doc) if chunk.strip()))
    batch_size = 32

    for i in range(0, len(chunks), batch_size):
        batch_chunks = chunks[i:i+batch_size]
        embeddings = embedding_model.embed(batch_chunks)

        points = [
            models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding.tolist(),
                payload={"text": chunk}
            )
            for chunk, embedding in zip(batch_chunks, embeddings)
        ]
        qdrant_client.upsert(collection_name=COLLECTION_NAME, points=points, wait=True)
        print(f"Ingested batch {i // batch_size + 1}/{(len(chunks)+batch_size-1)//batch_size}")

    print(f"Successfully ingested {len(chunks)} points.")

if __name__ == "__main__":
    main()
