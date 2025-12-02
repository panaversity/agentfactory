"""
Embedding generator for RoboLearn docs.

Uses OpenAI's text-embedding-3-small model.
Implements exponential backoff for rate limit handling.
"""

from typing import Iterator
from datetime import datetime
import time

from openai import OpenAI, RateLimitError
from rich.console import Console
from rich.progress import Progress, SpinnerColumn, TextColumn, BarColumn, TaskProgressColumn

from models.document import DocumentChunk, EmbeddedChunk
from core.config import get_settings

console = Console()


class OpenAIEmbedder:
    """
    Generates embeddings using OpenAI's embedding API.

    Uses text-embedding-3-small (1536 dimensions) by default.
    Supports batching for efficiency.
    """

    def __init__(
        self,
        api_key: str = None,
        model: str = None,
        batch_size: int = 20,  # OpenAI recommends batches of ~20 for embeddings
        max_retries: int = 5,
        initial_retry_delay: float = 1.0,
    ):
        """
        Initialize embedder.

        Args:
            api_key: OpenAI API key (defaults to env var)
            model: Embedding model name
            batch_size: Number of texts to embed per API call
            max_retries: Maximum number of retry attempts for rate limits
            initial_retry_delay: Initial delay in seconds for exponential backoff
        """
        settings = get_settings()
        self.client = OpenAI(api_key=api_key or settings.openai_api_key)
        self.model = model or settings.embedding_model
        self.batch_size = batch_size
        self.dimensions = settings.embedding_dimensions
        self.max_retries = max_retries
        self.initial_retry_delay = initial_retry_delay

    def embed_chunks(
        self,
        chunks: list[DocumentChunk],
        show_progress: bool = True,
    ) -> list[EmbeddedChunk]:
        """
        Generate embeddings for a list of chunks.

        Args:
            chunks: List of DocumentChunks to embed
            show_progress: Whether to show progress bar

        Returns:
            List of EmbeddedChunks with embeddings
        """
        if not chunks:
            return []

        embedded = []
        total = len(chunks)

        if show_progress:
            with Progress(
                SpinnerColumn(),
                TextColumn("[progress.description]{task.description}"),
                BarColumn(),
                TaskProgressColumn(),
                console=console,
            ) as progress:
                task = progress.add_task(f"[cyan]Embedding {total} chunks...", total=total)

                for batch_start in range(0, total, self.batch_size):
                    batch_end = min(batch_start + self.batch_size, total)
                    batch = chunks[batch_start:batch_end]

                    batch_embedded = self._embed_batch(batch)
                    embedded.extend(batch_embedded)

                    progress.update(task, advance=len(batch))
        else:
            for batch_start in range(0, total, self.batch_size):
                batch_end = min(batch_start + self.batch_size, total)
                batch = chunks[batch_start:batch_end]

                batch_embedded = self._embed_batch(batch)
                embedded.extend(batch_embedded)

        return embedded

    def _embed_batch(self, chunks: list[DocumentChunk]) -> list[EmbeddedChunk]:
        """
        Embed a batch of chunks in a single API call with exponential backoff.

        Args:
            chunks: Batch of chunks to embed

        Returns:
            List of EmbeddedChunks

        Raises:
            RateLimitError: If max retries exceeded
        """
        texts = [chunk.text for chunk in chunks]

        # Exponential backoff retry logic
        for attempt in range(self.max_retries):
            try:
                response = self.client.embeddings.create(
                    input=texts,
                    model=self.model,
                )
                break  # Success, exit retry loop

            except RateLimitError as e:
                if attempt == self.max_retries - 1:
                    # Max retries exceeded
                    console.print(f"[red]Rate limit exceeded after {self.max_retries} attempts[/red]")
                    raise

                # Calculate exponential backoff delay
                delay = self.initial_retry_delay * (2 ** attempt)
                console.print(
                    f"[yellow]Rate limit hit (attempt {attempt + 1}/{self.max_retries}). "
                    f"Retrying in {delay:.1f}s...[/yellow]"
                )
                time.sleep(delay)

        embedded = []
        now = datetime.utcnow()

        for chunk, embedding_data in zip(chunks, response.data):
            embedded_chunk = EmbeddedChunk(
                id=chunk.id,
                text=chunk.text,
                metadata=chunk.metadata,
                word_count=chunk.word_count,
                token_count=chunk.token_count,
                char_count=chunk.char_count,
                embedding=embedding_data.embedding,
                embedded_at=now,
            )
            embedded.append(embedded_chunk)

        return embedded

    def embed_single(self, text: str) -> list[float]:
        """
        Generate embedding for a single text with exponential backoff.
        Useful for query embedding.

        Args:
            text: Text to embed

        Returns:
            Embedding vector (list of floats)

        Raises:
            RateLimitError: If max retries exceeded
        """
        # Exponential backoff retry logic
        for attempt in range(self.max_retries):
            try:
                response = self.client.embeddings.create(
                    input=[text],
                    model=self.model,
                )
                return response.data[0].embedding

            except RateLimitError as e:
                if attempt == self.max_retries - 1:
                    console.print(f"[red]Rate limit exceeded after {self.max_retries} attempts[/red]")
                    raise

                delay = self.initial_retry_delay * (2 ** attempt)
                console.print(
                    f"[yellow]Rate limit hit (attempt {attempt + 1}/{self.max_retries}). "
                    f"Retrying in {delay:.1f}s...[/yellow]"
                )
                time.sleep(delay)
