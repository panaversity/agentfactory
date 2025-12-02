"""
Qdrant-native state tracking for RoboLearn ingestion.

Modern approach: Query Qdrant payloads directly instead of maintaining
a separate state database. Qdrant 1.14+ supports incremental HNSW indexing
for efficient upserts.

Benefits:
- Single source of truth (Qdrant)
- No external DB dependency
- Atomic with vector operations
- Survives container restarts automatically

References:
- https://qdrant.tech/documentation/concepts/payload/
- https://qdrant.tech/blog/qdrant-1.14.x/ (incremental HNSW)
"""

from typing import Optional
from dataclasses import dataclass

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Filter,
    FieldCondition,
    MatchValue,
    ScrollRequest,
)

from core.config import get_settings


@dataclass
class IndexedFileInfo:
    """Information about an indexed file from Qdrant."""
    source_file: str
    file_hash: str
    chunk_count: int
    chunk_ids: list[str]


class QdrantStateTracker:
    """
    Query Qdrant payloads to track indexed files.

    Replaces file-based state with direct Qdrant queries.
    Uses source_file_hash stored in each chunk's payload.
    """

    def __init__(
        self,
        url: str = None,
        api_key: str = None,
        collection_name: str = None,
    ):
        settings = get_settings()
        self.client = QdrantClient(
            url=url or settings.qdrant_url,
            api_key=api_key or settings.qdrant_api_key,
        )
        self.collection_name = collection_name or settings.collection_name
        self.book_id = settings.book_id

    def get_indexed_files(self) -> dict[str, IndexedFileInfo]:
        """
        Get all indexed files and their hashes from Qdrant.

        Scrolls through collection and groups by source_file.

        Returns:
            Dict of {source_file: IndexedFileInfo}
        """
        indexed = {}

        # Scroll through all points for this book
        offset = None
        while True:
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=Filter(
                    must=[
                        FieldCondition(
                            key="book_id",
                            match=MatchValue(value=self.book_id),
                        )
                    ]
                ),
                limit=100,
                offset=offset,
                with_payload=["source_file", "source_file_hash"],
                with_vectors=False,
            )

            points, next_offset = results

            for point in points:
                source_file = point.payload.get("source_file")
                file_hash = point.payload.get("source_file_hash")

                if source_file not in indexed:
                    indexed[source_file] = IndexedFileInfo(
                        source_file=source_file,
                        file_hash=file_hash,
                        chunk_count=0,
                        chunk_ids=[],
                    )

                indexed[source_file].chunk_count += 1
                indexed[source_file].chunk_ids.append(str(point.id))

            if next_offset is None:
                break
            offset = next_offset

        return indexed

    def get_file_hash(self, source_file: str) -> Optional[str]:
        """
        Get the hash of a specific indexed file.

        Returns:
            File hash if indexed, None otherwise
        """
        results = self.client.scroll(
            collection_name=self.collection_name,
            scroll_filter=Filter(
                must=[
                    FieldCondition(
                        key="book_id",
                        match=MatchValue(value=self.book_id),
                    ),
                    FieldCondition(
                        key="source_file",
                        match=MatchValue(value=source_file),
                    ),
                ]
            ),
            limit=1,
            with_payload=["source_file_hash"],
            with_vectors=False,
        )

        points, _ = results
        if points:
            return points[0].payload.get("source_file_hash")
        return None

    def get_chunks_for_file(self, source_file: str) -> list[str]:
        """
        Get all chunk IDs for a specific file.

        Returns:
            List of chunk UUIDs
        """
        chunk_ids = []
        offset = None

        while True:
            results = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=Filter(
                    must=[
                        FieldCondition(
                            key="book_id",
                            match=MatchValue(value=self.book_id),
                        ),
                        FieldCondition(
                            key="source_file",
                            match=MatchValue(value=source_file),
                        ),
                    ]
                ),
                limit=100,
                offset=offset,
                with_payload=False,
                with_vectors=False,
            )

            points, next_offset = results
            chunk_ids.extend(str(p.id) for p in points)

            if next_offset is None:
                break
            offset = next_offset

        return chunk_ids

    def detect_changes(
        self,
        current_files: dict[str, str],
    ) -> tuple[list[str], list[str], list[str]]:
        """
        Compare current files against Qdrant index.

        Args:
            current_files: Dict of {relative_path: file_hash}

        Returns:
            Tuple of (new_files, modified_files, deleted_files)
        """
        indexed = self.get_indexed_files()
        indexed_paths = set(indexed.keys())
        current_paths = set(current_files.keys())

        # New files: in current but not indexed
        new_files = list(current_paths - indexed_paths)

        # Deleted files: indexed but not in current
        deleted_files = list(indexed_paths - current_paths)

        # Modified files: in both but hash differs
        modified_files = []
        for path in current_paths & indexed_paths:
            if current_files[path] != indexed[path].file_hash:
                modified_files.append(path)

        return new_files, modified_files, deleted_files

    def get_collection_stats(self) -> dict:
        """
        Get statistics about indexed content for this book.

        Returns:
            Dict with file count, chunk count, etc.
        """
        indexed = self.get_indexed_files()

        total_chunks = sum(f.chunk_count for f in indexed.values())

        # Group by module
        modules = {}
        for info in indexed.values():
            # Extract module from path like "module-1-ros2/..."
            parts = info.source_file.split("/")
            if parts:
                module = parts[0]
                modules[module] = modules.get(module, 0) + info.chunk_count

        return {
            "book_id": self.book_id,
            "files_indexed": len(indexed),
            "total_chunks": total_chunks,
            "chunks_by_module": modules,
        }
