"""
Embedding Service - Generate embeddings using Sentence Transformers (FREE)

This service uses the FREE Sentence Transformers library to generate
vector embeddings locally - NO API KEY REQUIRED!
"""

import os
from typing import List, Dict
from sentence_transformers import SentenceTransformer


class EmbeddingService:
    """
    Service for generating embeddings using Sentence Transformers (FREE).

    Uses 'all-mpnet-base-v2' model with:
    - 768-dimensional embeddings
    - Runs locally (no API calls)
    - FREE - no API key required
    - High quality semantic search
    """

    def __init__(self, model_name: str = "all-mpnet-base-v2"):
        """
        Initialize the embedding service with a local model.

        Args:
            model_name: Sentence Transformer model to use
                       Default: 'all-mpnet-base-v2' (768 dims, high quality)
                       Alternative: 'all-MiniLM-L6-v2' (384 dims, faster)
        """
        print(f"Loading embedding model: {model_name}...")
        print("This is a FREE local model - no API key needed! 🎉")

        # Load the model (will download on first use, then cached)
        self.model = SentenceTransformer(model_name)
        self.model_name = model_name

        print(f"✅ Model loaded successfully!")
        print(f"   Embedding dimension: {self.model.get_sentence_embedding_dimension()}")

    def embed_document(self, text: str) -> List[float]:
        """
        Generate embedding for a document.

        Args:
            text: Document text to embed

        Returns:
            Embedding vector (list of floats)
        """
        # Generate embedding
        embedding = self.model.encode(text, convert_to_numpy=True)
        return embedding.tolist()

    def embed_documents(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple documents (batched for efficiency).

        Args:
            texts: List of document texts to embed

        Returns:
            List of embedding vectors
        """
        # Batch encoding for efficiency
        embeddings = self.model.encode(texts, convert_to_numpy=True, show_progress_bar=False)
        return embeddings.tolist()

    def embed_query(self, query: str) -> List[float]:
        """
        Generate embedding for a search query.

        Args:
            query: Search query text

        Returns:
            Embedding vector (list of floats)
        """
        # Same as embed_document for this model
        embedding = self.model.encode(query, convert_to_numpy=True)
        return embedding.tolist()

    def get_embedding_dimension(self) -> int:
        """
        Get the dimension of embeddings produced by this service.

        Returns:
            Embedding dimension (768 for all-mpnet-base-v2)
        """
        return self.model.get_sentence_embedding_dimension()


# Test function
if __name__ == "__main__":
    # Test embedding service
    print("Testing FREE Sentence Transformers Embedding Service\n")

    service = EmbeddingService()

    # Test document embedding
    print("\n[1] Testing document embedding...")
    test_doc = "Python is a high-level programming language known for its readability."
    doc_embedding = service.embed_document(test_doc)
    print(f"✅ Document embedding dimension: {len(doc_embedding)}")
    print(f"   First 5 values: {doc_embedding[:5]}")

    # Test query embedding
    print("\n[2] Testing query embedding...")
    test_query = "What is Python?"
    query_embedding = service.embed_query(test_query)
    print(f"✅ Query embedding dimension: {len(query_embedding)}")
    print(f"   First 5 values: {query_embedding[:5]}")

    # Test batch embedding
    print("\n[3] Testing batch embedding...")
    test_docs = [
        "Python is a programming language.",
        "JavaScript is used for web development.",
        "Machine learning uses neural networks."
    ]
    batch_embeddings = service.embed_documents(test_docs)
    print(f"✅ Batch embeddings count: {len(batch_embeddings)}")
    print(f"   Each embedding dimension: {len(batch_embeddings[0])}")

    print("\n🎉 All tests passed! FREE embeddings working perfectly!")
