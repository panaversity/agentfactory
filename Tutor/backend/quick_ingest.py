#!/usr/bin/env python3
"""
Quick Book Ingestion Script

This will populate your local ChromaDB with all book content.
Run this on your Windows machine to enable RAG search!
"""

import sys
from pathlib import Path

print("\n" + "=" * 80)
print("📚 Quick Book Ingestion - Populate ChromaDB")
print("=" * 80)
print()

# Check if book-source exists
book_source = Path("../book-source")
if not book_source.exists():
    print("❌ Error: book-source not found!")
    print(f"   Looking for: {book_source.absolute()}")
    print()
    print("Please run this from: Tutor/backend/")
    sys.exit(1)

print(f"✓ Found book-source: {book_source.absolute()}")
print()

# Import and run ingestion
try:
    from app.services.book_parser import BookContentParser
    from app.services.embedding_service import EmbeddingService
    from app.services.vector_store import VectorStore
    from tqdm import tqdm

    print("=" * 80)
    print("STEP 1: Parsing Book Content")
    print("=" * 80)
    print()

    parser = BookContentParser(str(book_source.absolute()))
    chunks = parser.parse_all_lessons()

    print(f"✓ Parsed {len(chunks)} chunks from {len(set(c.metadata['chapter'] for c in chunks))} chapters")
    print()

    print("=" * 80)
    print("STEP 2: Loading FREE Embedding Model")
    print("=" * 80)
    print()

    embedding_service = EmbeddingService()
    print()

    print("=" * 80)
    print("STEP 3: Initializing ChromaDB")
    print("=" * 80)
    print()

    vector_store = VectorStore(persist_directory="./data/embeddings")

    # Reset collection
    print("Resetting 'book_content' collection...")
    try:
        vector_store.delete_collection()
        print("✓ Old collection deleted")
    except:
        print("✓ No existing collection to delete")

    vector_store.get_or_create_collection()
    print("✓ Fresh collection created")
    print()

    print("=" * 80)
    print("STEP 4: Generating Embeddings & Storing")
    print("=" * 80)
    print()
    print("This will take ~5-10 minutes...")
    print()

    # Process in batches
    batch_size = 50
    all_ids = []
    all_embeddings = []
    all_documents = []
    all_metadatas = []

    for i in tqdm(range(0, len(chunks), batch_size), desc="Embedding batches"):
        batch = chunks[i:i + batch_size]

        # Extract data
        batch_docs = [chunk.content for chunk in batch]
        batch_metadatas = [chunk.metadata for chunk in batch]
        batch_ids = [chunk.id for chunk in batch]

        # Generate embeddings
        batch_embeddings = embedding_service.embed_documents(batch_docs)

        # Accumulate
        all_ids.extend(batch_ids)
        all_embeddings.extend(batch_embeddings)
        all_documents.extend(batch_docs)
        all_metadatas.extend(batch_metadatas)

    print()
    print("Storing in ChromaDB...")
    vector_store.upsert_chunks(
        ids=all_ids,
        embeddings=all_embeddings,
        documents=all_documents,
        metadatas=all_metadatas
    )

    print()
    print("=" * 80)
    print("✅ INGESTION COMPLETE!")
    print("=" * 80)
    print()

    # Verify
    stats = vector_store.get_stats()
    print("Vector Store Stats:")
    print(f"  - Collection: {stats['name']}")
    print(f"  - Total chunks: {stats['count']}")
    print()

    # Test search
    print("=" * 80)
    print("Testing Search...")
    print("=" * 80)
    print()

    test_query = "What is Python?"
    print(f"Query: '{test_query}'")

    query_embedding = embedding_service.embed_query(test_query)
    results = vector_store.search(query_embedding, n_results=3)

    if results and results['ids'] and len(results['ids'][0]) > 0:
        print(f"✓ Found {len(results['ids'][0])} results!")
        print()
        for i, (doc_id, distance) in enumerate(zip(results['ids'][0], results['distances'][0])):
            metadata = results['metadatas'][0][i]
            print(f"[{i+1}] Score: {1 - distance:.2f}")
            print(f"    Chapter: {metadata.get('chapter_title', 'N/A')}")
            print(f"    Lesson: {metadata.get('lesson_title', 'N/A')}")
            print()

        print("=" * 80)
        print("🎉 SUCCESS! Your RAG system is ready to use!")
        print("=" * 80)
        print()
        print("Next steps:")
        print("  1. Run: python simple_test.py")
        print("  2. Run: python test_agent_live.py (if you have valid API key)")
        print()
    else:
        print("❌ Search test failed - 0 results returned")
        print("Please check the logs above for errors")

except Exception as e:
    print(f"\n❌ Error during ingestion: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
