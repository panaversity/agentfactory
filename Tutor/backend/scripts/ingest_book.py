#!/usr/bin/env python3
"""
Book Content Ingestion Script

This script:
1. Parses all markdown lessons in book-source
2. Chunks the content into semantic pieces
3. Generates embeddings using Gemini
4. Stores in ChromaDB for RAG search

Usage:
    python scripts/ingest_book.py
    python scripts/ingest_book.py --reset  # Reset database first
"""

import sys
import argparse
from pathlib import Path
from tqdm import tqdm

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.book_parser import BookContentParser
from app.services.embedding_service import EmbeddingService
from app.services.vector_store import VectorStore


def ingest_book_content(
    book_source_path: str,
    persist_directory: str = "./data/embeddings",
    reset: bool = False,
    batch_size: int = 50
):
    """
    Ingest all book content into the vector store.

    Args:
        book_source_path: Path to book-source directory
        persist_directory: Path to store vector database
        reset: If True, reset the database before ingesting
        batch_size: Number of chunks to process in each batch
    """
    print("=" * 80)
    print("TutorGPT Book Content Ingestion")
    print("=" * 80)

    # Initialize services
    print("\n[1/5] Initializing services...")
    parser = BookContentParser(book_source_path)
    embedding_service = EmbeddingService()
    vector_store = VectorStore(persist_directory=persist_directory)

    # Reset if requested
    if reset:
        print("\n[2/5] Resetting vector store...")
        vector_store.reset()
    else:
        print("\n[2/5] Using existing vector store (use --reset to clear)")

    # Parse all lessons
    print("\n[3/5] Parsing all lessons...")
    chunks = parser.parse_all_lessons()
    print(f"✓ Parsed {len(chunks)} chunks from book content")

    if len(chunks) == 0:
        print("\n❌ No chunks found! Check book_source_path:")
        print(f"   {book_source_path}")
        return

    # Generate embeddings in batches
    print(f"\n[4/5] Generating embeddings (batch size: {batch_size})...")
    all_ids = []
    all_embeddings = []
    all_documents = []
    all_metadatas = []

    # Process in batches for memory efficiency
    for i in tqdm(range(0, len(chunks), batch_size), desc="Embedding batches"):
        batch_chunks = chunks[i:i + batch_size]

        # Extract data for batch
        batch_ids = [chunk.chunk_id for chunk in batch_chunks]
        batch_documents = [chunk.content for chunk in batch_chunks]

        # Generate embeddings
        batch_embeddings = embedding_service.embed_documents(batch_documents)

        # Collect metadatas
        batch_metadatas = [chunk.metadata for chunk in batch_chunks]

        # Accumulate
        all_ids.extend(batch_ids)
        all_embeddings.extend(batch_embeddings)
        all_documents.extend(batch_documents)
        all_metadatas.extend(batch_metadatas)

    print(f"✓ Generated {len(all_embeddings)} embeddings")

    # Store in vector database
    print("\n[5/5] Storing in vector database...")
    vector_store.upsert_chunks(
        ids=all_ids,
        embeddings=all_embeddings,
        documents=all_documents,
        metadatas=all_metadatas
    )

    # Get stats
    stats = vector_store.get_stats()
    print(f"\n✓ Vector store stats:")
    print(f"  - Collection: {stats['name']}")
    print(f"  - Total chunks: {stats['count']}")
    print(f"  - Embedding dimension: {stats['metadata'].get('embedding_dimension', 768)}")

    print("\n" + "=" * 80)
    print("✅ Ingestion complete!")
    print("=" * 80)


def test_search(vector_store: VectorStore, embedding_service: EmbeddingService):
    """
    Test the RAG search with sample queries.

    Args:
        vector_store: Vector store instance
        embedding_service: Embedding service instance
    """
    print("\n" + "=" * 80)
    print("Testing RAG Search")
    print("=" * 80)

    test_queries = [
        "What is Python?",
        "How do I use async/await?",
        "What is AI-driven development?",
        "Explain variables in Python"
    ]

    for query in test_queries:
        print(f"\nQuery: '{query}'")

        # Generate query embedding
        query_embedding = embedding_service.embed_query(query)

        # Search
        results = vector_store.search(
            query_embedding=query_embedding,
            n_results=3
        )

        # Display results
        if results['ids'] and results['ids'][0]:
            for i in range(len(results['ids'][0])):
                distance = results['distances'][0][i]
                score = 1.0 - distance
                content = results['documents'][0][i]
                metadata = results['metadatas'][0][i]

                print(f"\n  [{i+1}] Score: {score:.3f}")
                print(f"      Chapter: {metadata.get('chapter', 'N/A')}")
                print(f"      Lesson: {metadata.get('lesson', 'N/A')}")
                print(f"      Content: {content[:150]}...")
        else:
            print("  No results found")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Ingest book content into vector database for RAG"
    )
    parser.add_argument(
        "--book-source",
        type=str,
        default="../book-source",
        help="Path to book-source directory"
    )
    parser.add_argument(
        "--persist-dir",
        type=str,
        default="./data/embeddings",
        help="Path to store vector database"
    )
    parser.add_argument(
        "--reset",
        action="store_true",
        help="Reset the database before ingesting"
    )
    parser.add_argument(
        "--batch-size",
        type=int,
        default=50,
        help="Batch size for embedding generation"
    )
    parser.add_argument(
        "--test",
        action="store_true",
        help="Run test queries after ingestion"
    )

    args = parser.parse_args()

    # Resolve book source path
    book_source_path = Path(__file__).parent.parent / args.book_source
    book_source_path = book_source_path.resolve()

    # Check if book source exists
    if not book_source_path.exists():
        print(f"❌ Book source not found: {book_source_path}")
        print("\nTried:")
        print(f"  {book_source_path}")
        print("\nPlease check the --book-source argument")
        sys.exit(1)

    # Run ingestion
    try:
        ingest_book_content(
            book_source_path=str(book_source_path),
            persist_directory=args.persist_dir,
            reset=args.reset,
            batch_size=args.batch_size
        )

        # Run tests if requested
        if args.test:
            embedding_service = EmbeddingService()
            vector_store = VectorStore(persist_directory=args.persist_dir)
            test_search(vector_store, embedding_service)

    except KeyboardInterrupt:
        print("\n\n❌ Interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ Error during ingestion: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
