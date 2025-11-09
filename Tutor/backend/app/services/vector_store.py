"""
Vector Store Service - ChromaDB integration

This service manages the ChromaDB vector database for storing
and searching book content embeddings.
"""

import chromadb
from chromadb.config import Settings
from typing import List, Dict, Any, Optional
from pathlib import Path


class VectorStore:
    """
    Vector store service using ChromaDB for RAG.

    Features:
    - Persistent storage of embeddings
    - Metadata filtering (chapter, lesson, difficulty)
    - Cosine similarity search
    - Collection management
    """

    def __init__(self, persist_directory: str = "./data/embeddings"):
        """
        Initialize ChromaDB vector store.

        Args:
            persist_directory: Directory to store the database
        """
        # Ensure directory exists
        Path(persist_directory).mkdir(parents=True, exist_ok=True)

        # Initialize ChromaDB client with persistence
        self.client = chromadb.PersistentClient(
            path=persist_directory,
            settings=Settings(
                anonymized_telemetry=False,
                allow_reset=True
            )
        )

        # Collection will be created/loaded on demand
        self.collection_name = "book_content"
        self.collection = None

    def get_or_create_collection(self, embedding_dimension: int = 768):
        """
        Get or create the book content collection.

        Args:
            embedding_dimension: Dimension of embeddings (768 or 3072)

        Returns:
            ChromaDB collection
        """
        if self.collection is None:
            self.collection = self.client.get_or_create_collection(
                name=self.collection_name,
                metadata={
                    "hnsw:space": "cosine",  # Use cosine similarity
                    "description": "AI-Native Software Development book content",
                    "embedding_dimension": embedding_dimension
                }
            )

        return self.collection

    def add_chunks(
        self,
        ids: List[str],
        embeddings: List[List[float]],
        documents: List[str],
        metadatas: List[Dict[str, Any]]
    ):
        """
        Add chunks to the vector store.

        Args:
            ids: List of chunk IDs
            embeddings: List of embedding vectors
            documents: List of text contents
            metadatas: List of metadata dictionaries
        """
        collection = self.get_or_create_collection()

        # Add to collection in batches if large
        batch_size = 100
        for i in range(0, len(ids), batch_size):
            batch_ids = ids[i:i + batch_size]
            batch_embeddings = embeddings[i:i + batch_size]
            batch_documents = documents[i:i + batch_size]
            batch_metadatas = metadatas[i:i + batch_size]

            collection.add(
                ids=batch_ids,
                embeddings=batch_embeddings,
                documents=batch_documents,
                metadatas=batch_metadatas
            )

        print(f"Added {len(ids)} chunks to vector store")

    def search(
        self,
        query_embedding: List[float],
        n_results: int = 5,
        where: Optional[Dict[str, Any]] = None
    ) -> Dict[str, Any]:
        """
        Search the vector store using query embedding.

        Args:
            query_embedding: Query embedding vector
            n_results: Number of results to return
            where: Metadata filter (e.g., {"chapter": "04-python"})

        Returns:
            Dictionary with ids, documents, metadatas, distances
        """
        collection = self.get_or_create_collection()

        results = collection.query(
            query_embeddings=[query_embedding],
            n_results=n_results,
            where=where
        )

        return results

    def get_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the vector store.

        Returns:
            Dictionary with collection stats
        """
        collection = self.get_or_create_collection()

        return {
            "name": self.collection_name,
            "count": collection.count(),
            "metadata": collection.metadata
        }

    def delete_all(self):
        """Delete all documents from the collection."""
        try:
            self.client.delete_collection(name=self.collection_name)
            self.collection = None
            print(f"Deleted collection: {self.collection_name}")
        except Exception as e:
            print(f"Error deleting collection: {e}")

    def reset(self):
        """Reset the vector store (delete and recreate)."""
        self.delete_all()
        self.get_or_create_collection()

    def upsert_chunks(
        self,
        ids: List[str],
        embeddings: List[List[float]],
        documents: List[str],
        metadatas: List[Dict[str, Any]]
    ):
        """
        Update or insert chunks (upsert operation).

        Args:
            ids: List of chunk IDs
            embeddings: List of embedding vectors
            documents: List of text contents
            metadatas: List of metadata dictionaries
        """
        collection = self.get_or_create_collection()

        # Upsert to collection in batches
        batch_size = 100
        for i in range(0, len(ids), batch_size):
            batch_ids = ids[i:i + batch_size]
            batch_embeddings = embeddings[i:i + batch_size]
            batch_documents = documents[i:i + batch_size]
            batch_metadatas = metadatas[i:i + batch_size]

            collection.upsert(
                ids=batch_ids,
                embeddings=batch_embeddings,
                documents=batch_documents,
                metadatas=batch_metadatas
            )

        print(f"Upserted {len(ids)} chunks to vector store")


# Test function
if __name__ == "__main__":
    import numpy as np

    # Initialize vector store
    store = VectorStore(persist_directory="./test_embeddings")

    # Create test data
    test_ids = ["chunk_01", "chunk_02", "chunk_03"]
    test_embeddings = [np.random.rand(768).tolist() for _ in range(3)]
    test_documents = [
        "Python is a high-level programming language.",
        "JavaScript is used for web development.",
        "Machine learning uses neural networks."
    ]
    test_metadatas = [
        {"chapter": "04-python", "lesson": "01-intro", "difficulty": "beginner"},
        {"chapter": "02-javascript", "lesson": "01-intro", "difficulty": "beginner"},
        {"chapter": "05-ml", "lesson": "01-intro", "difficulty": "advanced"}
    ]

    # Add chunks
    store.add_chunks(test_ids, test_embeddings, test_documents, test_metadatas)

    # Get stats
    stats = store.get_stats()
    print(f"\nVector store stats: {stats}")

    # Search
    query_embedding = np.random.rand(768).tolist()
    results = store.search(query_embedding, n_results=2)
    print(f"\nSearch results: {len(results['ids'][0])} chunks found")

    # Search with filter
    results_filtered = store.search(
        query_embedding,
        n_results=2,
        where={"chapter": "04-python"}
    )
    print(f"Filtered search results: {len(results_filtered['ids'][0])} chunks found")

    # Clean up
    store.delete_all()
    print("\nTest completed and cleaned up")
