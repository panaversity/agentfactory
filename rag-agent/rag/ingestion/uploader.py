"""
Production-grade Qdrant uploader for RoboLearn docs.

Features:
- Incremental updates (only re-embed changed content)
- State persistence for change detection
- Batch operations for efficiency
- Multitenancy with book_id as tenant field
- Full payload indexing for filtered retrieval

References:
- https://particula.tech/blog/update-rag-knowledge-without-rebuilding
"""

import json
from pathlib import Path
from typing import Optional
from datetime import datetime

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance,
    VectorParams,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    Range,
    PayloadSchemaType,
    PointIdsList,
)
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TaskProgressColumn

from models.document import (
    EmbeddedChunk,
    QdrantPayload,
    IngestionState,
    IngestionRecord,
)
from core.config import get_settings

console = Console()


class QdrantUploader:
    """
    Production-grade Qdrant uploader with incremental update support.

    Key features:
    1. State persistence - tracks what's indexed
    2. Incremental updates - only re-embed changed files
    3. Atomic operations - delete old then insert new
    4. Full payload indexing - optimized filtered retrieval
    """

    STATE_FILE = ".ingestion_state.json"

    def __init__(
        self,
        url: str = None,
        api_key: str = None,
        collection_name: str = None,
        state_dir: str = None,
    ):
        """
        Initialize Qdrant client with state management.

        Args:
            url: Qdrant Cloud URL
            api_key: Qdrant API key
            collection_name: Collection name
            state_dir: Directory to store ingestion state
        """
        settings = get_settings()
        self.client = QdrantClient(
            url=url or settings.qdrant_url,
            api_key=api_key or settings.qdrant_api_key,
        )
        self.collection_name = collection_name or settings.collection_name
        self.dimensions = settings.embedding_dimensions
        self.batch_size = settings.batch_size
        self.book_id = settings.book_id

        # State file location
        self.state_dir = Path(state_dir) if state_dir else Path(".")
        self.state_file = self.state_dir / self.STATE_FILE

    def load_state(self) -> IngestionState:
        """Load ingestion state from file."""
        if self.state_file.exists():
            try:
                with open(self.state_file, "r") as f:
                    data = json.load(f)
                return IngestionState(**data)
            except Exception as e:
                console.print(f"[yellow]Warning: Could not load state: {e}[/yellow]")

        return IngestionState(
            book_id=self.book_id,
            collection_name=self.collection_name,
        )

    def save_state(self, state: IngestionState):
        """Save ingestion state to file."""
        with open(self.state_file, "w") as f:
            json.dump(state.model_dump(mode="json"), f, indent=2, default=str)

    def ensure_collection(self, recreate: bool = False) -> bool:
        """
        Ensure collection exists with correct configuration.

        Args:
            recreate: If True, delete and recreate collection

        Returns:
            True if collection was created, False if already existed
        """
        collections = self.client.get_collections().collections
        exists = any(c.name == self.collection_name for c in collections)

        if exists and recreate:
            console.print(f"[yellow]Deleting existing collection: {self.collection_name}[/yellow]")
            self.client.delete_collection(self.collection_name)
            exists = False

            # Also clear state
            if self.state_file.exists():
                self.state_file.unlink()

        if not exists:
            console.print(f"[green]Creating collection: {self.collection_name}[/green]")

            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.dimensions,
                    distance=Distance.COSINE,
                ),
            )

            # Create payload indexes for efficient filtering
            self._create_indexes()

            return True
        else:
            console.print(f"[blue]Collection already exists: {self.collection_name}[/blue]")
            return False

    def _create_indexes(self):
        """Create payload indexes for efficient filtering."""
        console.print("[cyan]Creating payload indexes...[/cyan]")

        indexes = [
            ("book_id", PayloadSchemaType.KEYWORD),
            ("module", PayloadSchemaType.KEYWORD),
            ("hardware_tier", PayloadSchemaType.INTEGER),
            ("chapter", PayloadSchemaType.INTEGER),
            ("lesson", PayloadSchemaType.INTEGER),
            ("proficiency_level", PayloadSchemaType.KEYWORD),
            ("layer", PayloadSchemaType.KEYWORD),
            ("parent_doc_id", PayloadSchemaType.KEYWORD),
            ("content_hash", PayloadSchemaType.KEYWORD),
            ("source_file_hash", PayloadSchemaType.KEYWORD),
        ]

        for field_name, field_type in indexes:
            try:
                self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name=field_name,
                    field_schema=field_type,
                )
            except Exception as e:
                console.print(f"[yellow]Index {field_name} may already exist: {e}[/yellow]")

        console.print("[green]Payload indexes created[/green]")

    def upload(
        self,
        chunks: list[EmbeddedChunk],
        source_files: dict[str, str] = None,
        show_progress: bool = True,
    ) -> int:
        """
        Upload embedded chunks to Qdrant.

        Args:
            chunks: List of EmbeddedChunks to upload
            source_files: Dict of {relative_path: file_hash} for state tracking
            show_progress: Whether to show progress bar

        Returns:
            Number of points uploaded
        """
        if not chunks:
            return 0

        total = len(chunks)
        uploaded = 0

        if show_progress:
            with Progress(
                SpinnerColumn(),
                TextColumn("[progress.description]{task.description}"),
                BarColumn(),
                TaskProgressColumn(),
                console=console,
            ) as progress:
                task = progress.add_task(f"[cyan]Uploading {total} chunks...", total=total)

                for batch_start in range(0, total, self.batch_size):
                    batch_end = min(batch_start + self.batch_size, total)
                    batch = chunks[batch_start:batch_end]

                    self._upload_batch(batch)
                    uploaded += len(batch)

                    progress.update(task, advance=len(batch))
        else:
            for batch_start in range(0, total, self.batch_size):
                batch_end = min(batch_start + self.batch_size, total)
                batch = chunks[batch_start:batch_end]

                self._upload_batch(batch)
                uploaded += len(batch)

        # Update state if source_files provided
        if source_files:
            self._update_state(chunks, source_files)

        return uploaded

    def _upload_batch(self, chunks: list[EmbeddedChunk]):
        """Upload a batch of chunks."""
        points = []

        for chunk in chunks:
            payload = QdrantPayload.from_chunk(chunk)

            point = PointStruct(
                id=chunk.id,
                vector=chunk.embedding,
                payload=payload.model_dump(),
            )
            points.append(point)

        self.client.upsert(
            collection_name=self.collection_name,
            wait=True,
            points=points,
        )

    def _update_state(self, chunks: list[EmbeddedChunk], source_files: dict[str, str]):
        """Update ingestion state after upload."""
        state = self.load_state()

        # Group chunks by source file
        chunks_by_file: dict[str, list[EmbeddedChunk]] = {}
        for chunk in chunks:
            source = chunk.metadata.source_file
            if source not in chunks_by_file:
                chunks_by_file[source] = []
            chunks_by_file[source].append(chunk)

        # Update records
        for source_file, file_chunks in chunks_by_file.items():
            if source_file in source_files:
                state.records[source_file] = IngestionRecord(
                    source_file=source_file,
                    file_hash=source_files[source_file],
                    chunk_ids=[c.id for c in file_chunks],
                    chunk_count=len(file_chunks),
                    ingested_at=datetime.utcnow(),
                    book_id=self.book_id,
                    module=file_chunks[0].metadata.module,
                    chapter=file_chunks[0].metadata.chapter,
                    lesson=file_chunks[0].metadata.lesson,
                )

        state.total_chunks = sum(r.chunk_count for r in state.records.values())
        state.last_incremental_update = datetime.utcnow()

        self.save_state(state)

    def delete_chunks(self, chunk_ids: list[str]) -> int:
        """
        Delete specific chunks by ID.

        Args:
            chunk_ids: List of chunk UUIDs to delete

        Returns:
            Number of chunks deleted
        """
        if not chunk_ids:
            return 0

        console.print(f"[yellow]Deleting {len(chunk_ids)} chunks...[/yellow]")

        # Delete in batches
        for batch_start in range(0, len(chunk_ids), self.batch_size):
            batch_end = min(batch_start + self.batch_size, len(chunk_ids))
            batch = chunk_ids[batch_start:batch_end]

            self.client.delete(
                collection_name=self.collection_name,
                points_selector=PointIdsList(points=batch),
            )

        return len(chunk_ids)

    def delete_by_source_file(self, source_file: str) -> int:
        """
        Delete all chunks from a specific source file.

        Args:
            source_file: Relative path to source file

        Returns:
            Number of chunks deleted (approximate)
        """
        # Get chunk IDs from state
        state = self.load_state()
        if source_file in state.records:
            chunk_ids = state.records[source_file].chunk_ids
            deleted = self.delete_chunks(chunk_ids)

            # Update state
            del state.records[source_file]
            state.total_chunks = sum(r.chunk_count for r in state.records.values())
            self.save_state(state)

            return deleted
        return 0

    def get_collection_info(self) -> dict:
        """Get collection statistics."""
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "vectors_count": getattr(info, 'vectors_count', None) or info.points_count,
                "points_count": info.points_count,
                "status": info.status.value if hasattr(info.status, 'value') else str(info.status),
            }
        except Exception as e:
            return {"error": str(e)}

    def delete_book(self, book_id: str) -> int:
        """
        Delete all points for a specific book.

        Args:
            book_id: Book identifier to delete

        Returns:
            Number of points deleted (approximate)
        """
        info_before = self.get_collection_info()
        count_before = info_before.get("points_count", 0)

        self.client.delete(
            collection_name=self.collection_name,
            points_selector=Filter(
                must=[
                    FieldCondition(
                        key="book_id",
                        match=MatchValue(value=book_id),
                    )
                ]
            ),
        )

        # Clear state for this book
        state = self.load_state()
        if state.book_id == book_id:
            state.records = {}
            state.total_chunks = 0
            self.save_state(state)

        info_after = self.get_collection_info()
        count_after = info_after.get("points_count", 0)

        return count_before - count_after
