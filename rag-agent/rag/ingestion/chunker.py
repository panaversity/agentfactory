"""
Production-grade content chunker for RoboLearn docs.

Features:
- Semantic chunking by ## section headers
- Fixed token-based sizing (not word-based)
- 15% chunk overlap for context continuity
- Content hashing for change detection
- Chunk relationship tracking (prev/next)
- Parent document references

References:
- NVIDIA benchmark: 400-512 tokens optimal for factoid queries
- Weaviate best practices: 10-20% overlap recommended
- Unstructured: Semantic boundaries over fixed character counts
"""

import re
from typing import Iterator
from datetime import datetime

from rich.console import Console

from models.document import (
    DocumentChunk,
    DocumentMetadata,
    LessonFile,
    compute_content_hash,
)
from core.config import get_settings

console = Console()


class SectionChunker:
    """
    Production-grade chunker with overlap and relationship tracking.

    Strategy:
    1. Split content on ## headers (semantic boundaries)
    2. Target ~400 tokens per chunk (optimal for retrieval)
    3. Apply 15% overlap between consecutive chunks
    4. Track prev/next relationships for context expansion
    5. Hash content for change detection
    """

    # Pattern to split on ## headers (preserving header in chunk)
    SECTION_PATTERN = re.compile(r"(?=^## )", re.MULTILINE)

    # Token estimation: ~1.3 tokens per word for English text
    TOKENS_PER_WORD = 1.3

    def __init__(
        self,
        book_id: str = None,
        target_tokens: int = 400,
        max_tokens: int = 512,
        min_tokens: int = 100,
        overlap_percent: float = 0.15,
    ):
        """
        Initialize production chunker.

        Args:
            book_id: Book identifier for chunk IDs
            target_tokens: Target tokens per chunk (default 400)
            max_tokens: Maximum tokens before splitting (default 512)
            min_tokens: Minimum tokens to keep chunk (default 100)
            overlap_percent: Overlap between chunks (default 15%)
        """
        settings = get_settings()
        self.book_id = book_id or settings.book_id
        self.target_tokens = target_tokens
        self.max_tokens = max_tokens
        self.min_tokens = min_tokens
        self.overlap_percent = overlap_percent

        # Convert to words for easier processing
        self.target_words = int(target_tokens / self.TOKENS_PER_WORD)
        self.max_words = int(max_tokens / self.TOKENS_PER_WORD)
        self.min_words = int(min_tokens / self.TOKENS_PER_WORD)
        self.overlap_words = int(self.target_words * overlap_percent)

    def chunk(self, lesson: LessonFile) -> Iterator[DocumentChunk]:
        """
        Split lesson into production-grade chunks with overlap.

        Args:
            lesson: Parsed lesson file with file_hash

        Yields:
            DocumentChunk with full metadata including relationships
        """
        content = lesson.raw_content.strip()

        if not content:
            console.print(f"[yellow]Empty content in {lesson.relative_path}[/yellow]")
            return

        # Split by ## headers first (semantic boundaries)
        sections = self._split_by_sections(content)

        if not sections:
            # Treat entire content as one section
            title = self._extract_h1_title(content)
            sections = [(title, content)]

        # Process sections into fixed-size chunks with overlap
        raw_chunks = self._create_overlapping_chunks(sections)

        # Generate parent document ID
        parent_doc_id = DocumentChunk.generate_parent_doc_id(
            book_id=self.book_id,
            module=lesson.module,
            chapter=lesson.chapter,
            lesson=lesson.lesson,
        )

        # First pass: create chunks with content hashes to generate IDs
        chunk_data = []
        for idx, (section_title, text) in enumerate(raw_chunks):
            content_hash = compute_content_hash(text)
            chunk_id = DocumentChunk.generate_id(
                book_id=self.book_id,
                module=lesson.module,
                chapter=lesson.chapter,
                lesson=lesson.lesson,
                content_hash=content_hash,
            )
            chunk_data.append({
                "id": chunk_id,
                "text": text,
                "section_title": section_title,
                "content_hash": content_hash,
                "index": idx,
            })

        total_chunks = len(chunk_data)

        # Second pass: create DocumentChunks with prev/next relationships
        for idx, data in enumerate(chunk_data):
            prev_id = chunk_data[idx - 1]["id"] if idx > 0 else None
            next_id = chunk_data[idx + 1]["id"] if idx < total_chunks - 1 else None

            word_count = len(data["text"].split())
            token_count = int(word_count * self.TOKENS_PER_WORD)
            char_count = len(data["text"])

            metadata = DocumentMetadata(
                book_id=self.book_id,
                module=lesson.module,
                chapter=lesson.chapter,
                lesson=lesson.lesson,
                section_title=data["section_title"],
                hardware_tier=lesson.hardware_tier,
                proficiency_level=lesson.proficiency_level,
                layer=lesson.layer,
                source_file=lesson.relative_path,
                lesson_title=lesson.title,  # Store lesson title from frontmatter
                lesson_description=lesson.description,  # Store lesson description from frontmatter
                doc_id=lesson.doc_id,  # Store Docusaurus doc ID for URL generation
                parent_doc_id=parent_doc_id,
                chunk_index=idx,
                total_chunks=total_chunks,
                prev_chunk_id=prev_id,
                next_chunk_id=next_id,
                content_hash=data["content_hash"],
                source_file_hash=lesson.file_hash,
                version=1,
                created_at=datetime.utcnow(),
                updated_at=datetime.utcnow(),
            )

            yield DocumentChunk(
                id=data["id"],
                text=data["text"],
                metadata=metadata,
                word_count=word_count,
                token_count=token_count,
                char_count=char_count,
            )

    def _split_by_sections(self, content: str) -> list[tuple[str | None, str]]:
        """Split content by ## headers, keeping sections together."""
        sections = self.SECTION_PATTERN.split(content)
        result = []

        for section in sections:
            section = section.strip()
            if not section:
                continue

            # Extract section title
            title = None
            if section.startswith("## "):
                lines = section.split("\n", 1)
                title = lines[0].replace("## ", "").strip()

            word_count = len(section.split())
            if word_count >= self.min_words:
                result.append((title, section))

        return result

    def _create_overlapping_chunks(
        self,
        sections: list[tuple[str | None, str]]
    ) -> list[tuple[str | None, str]]:
        """
        Create fixed-size chunks with overlap from sections.

        If a section is larger than max_tokens, split it.
        Apply overlap between consecutive chunks for context.
        """
        chunks = []
        overlap_text = ""  # Text to prepend to next chunk

        for section_title, section_text in sections:
            # Prepend overlap from previous chunk
            if overlap_text:
                section_text = overlap_text + "\n\n" + section_text
                overlap_text = ""

            words = section_text.split()
            word_count = len(words)

            if word_count <= self.max_words:
                # Section fits in one chunk
                chunks.append((section_title, section_text))
                # Save overlap for next chunk
                if word_count > self.overlap_words:
                    overlap_text = " ".join(words[-self.overlap_words:])
            else:
                # Section too large, split into multiple chunks
                sub_chunks = self._split_large_section(section_title, section_text)
                chunks.extend(sub_chunks)
                # Save overlap from last sub-chunk
                last_chunk_words = sub_chunks[-1][1].split()
                if len(last_chunk_words) > self.overlap_words:
                    overlap_text = " ".join(last_chunk_words[-self.overlap_words:])

        return chunks

    def _split_large_section(
        self,
        title: str | None,
        text: str,
    ) -> list[tuple[str | None, str]]:
        """
        Split a large section at paragraph boundaries with overlap.
        """
        paragraphs = text.split("\n\n")
        chunks = []
        current_chunk = []
        current_words = 0

        for para in paragraphs:
            para = para.strip()
            if not para:
                continue

            para_words = len(para.split())

            # If adding this paragraph exceeds target, yield current chunk
            if current_words + para_words > self.target_words and current_chunk:
                chunk_text = "\n\n".join(current_chunk)
                chunk_title = title if not chunks else None  # Only first chunk gets title
                chunks.append((chunk_title, chunk_text))

                # Keep overlap from end of current chunk
                overlap_paras = []
                overlap_word_count = 0
                for p in reversed(current_chunk):
                    p_words = len(p.split())
                    if overlap_word_count + p_words <= self.overlap_words:
                        overlap_paras.insert(0, p)
                        overlap_word_count += p_words
                    else:
                        break

                current_chunk = overlap_paras
                current_words = overlap_word_count

            current_chunk.append(para)
            current_words += para_words

        # Yield remaining content
        if current_chunk:
            chunk_text = "\n\n".join(current_chunk)
            chunk_title = title if not chunks else None
            chunks.append((chunk_title, chunk_text))

        return chunks

    def _extract_h1_title(self, content: str) -> str | None:
        """Extract title from # header."""
        if content.startswith("# "):
            lines = content.split("\n", 1)
            return lines[0].replace("# ", "").strip()
        return None
