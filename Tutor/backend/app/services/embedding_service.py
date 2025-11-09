"""
Embedding Service - Generate embeddings using Gemini

This service uses Google's Gemini embedding model to generate
vector embeddings for text content.
"""

import os
from typing import List, Dict
from google import genai
from dotenv import load_dotenv

# Load environment variables
load_dotenv()


class EmbeddingService:
    """
    Service for generating embeddings using Gemini.

    Uses gemini-embedding-001 model with:
    - 768-dimensional embeddings (default)
    - RETRIEVAL_DOCUMENT task type for document embedding
    - RETRIEVAL_QUERY task type for query embedding
    """

    def __init__(self, api_key: str | None = None):
        """
        Initialize the embedding service.

        Args:
            api_key: Google API key. If None, reads from GOOGLE_API_KEY env var
        """
        self.api_key = api_key or os.getenv("GOOGLE_API_KEY")
        if not self.api_key:
            raise ValueError(
                "Google API key is required. Set GOOGLE_API_KEY environment variable "
                "or pass api_key to constructor."
            )

        # Initialize Gemini client
        self.client = genai.Client(api_key=self.api_key)
        self.model = "gemini-embedding-001"

    def embed_document(self, text: str) -> List[float]:
        """
        Generate embedding for a document.

        Args:
            text: Document text to embed

        Returns:
            Embedding vector (list of floats)
        """
        return self._generate_embedding(text, task_type="RETRIEVAL_DOCUMENT")

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple documents.

        Args:
            texts: List of document texts to embed

        Returns:
            List of embedding vectors
        """
        return [self.embed_document(text) for text in texts]

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a search query.

        Args:
            query: Search query text

        Returns:
            Embedding vector (list of floats)
        """
        return self._generate_embedding(query, task_type="RETRIEVAL_QUERY")

    def _generate_embedding(self, text: str, task_type: str) -> List[float]:
        """
        Internal method to generate embedding with specified task type.

        Args:
            text: Text to embed
            task_type: "RETRIEVAL_DOCUMENT" or "RETRIEVAL_QUERY"

        Returns:
            Embedding vector
        """
        try:
            result = self.client.models.embed_content(
                model=self.model,
                contents=text,
                config={
                    "task_type": task_type,
                    "output_dimensionality": 768  # Use 768 dimensions for faster search
                }
            )

            # Extract embedding from result
            # The result structure is: result['embedding']
            embedding = result.embedding

            return embedding

        except Exception as e:
            print(f"Error generating embedding: {e}")
            raise

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of embeddings produced by this service.

        Returns:
            Embedding dimension (768 or 3072)
        """
        return 768


# Test function
if __name__ == "__main__":
    # Test embedding service
    service = EmbeddingService()

    # Test document embedding
    test_doc = "Python is a high-level programming language known for its readability."
    doc_embedding = service.embed_document(test_doc)
    print(f"Document embedding dimension: {len(doc_embedding)}")
    print(f"First 5 values: {doc_embedding[:5]}")

    # Test query embedding
    test_query = "What is Python?"
    query_embedding = service.embed_query(test_query)
    print(f"\nQuery embedding dimension: {len(query_embedding)}")
    print(f"First 5 values: {query_embedding[:5]}")

    # Test batch embedding
    test_docs = [
        "Python is a programming language.",
        "JavaScript is used for web development.",
        "Machine learning uses neural networks."
    ]
    batch_embeddings = service.embed_documents(test_docs)
    print(f"\nBatch embeddings count: {len(batch_embeddings)}")
    print(f"Each embedding dimension: {len(batch_embeddings[0])}")
