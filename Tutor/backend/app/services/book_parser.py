"""
Book Content Parser - Extract and chunk markdown lessons

This service parses the book-source markdown files and chunks them
for embedding and RAG search.
"""

import re
from pathlib import Path
from typing import List, Dict, Any
from dataclasses import dataclass
from datetime import datetime
import frontmatter


@dataclass
class BookChunk:
    """Represents a single chunk of book content."""
    chunk_id: str
    content: str
    metadata: Dict[str, Any]


class BookContentParser:
    """
    Parses Docusaurus markdown files and chunks them for RAG.

    Strategy:
    - Parse frontmatter for metadata (title, position)
    - Split by headers (##, ###) to preserve semantic structure
    - Chunk large sections into ~512 token pieces with 50 token overlap
    - Extract code blocks separately with metadata
    - Preserve topic keywords and heading context
    """

    def __init__(self, book_source_path: str):
        """Initialize the parser with book-source path."""
        self.book_source_path = Path(book_source_path)
        self.docs_path = self.book_source_path / "docs"

    def parse_all_lessons(self) -> List[BookChunk]:
        """Parse all markdown lessons in the book-source."""
        chunks = []

        # Find all markdown files
        md_files = list(self.docs_path.rglob("*.md"))
        print(f"Found {len(md_files)} markdown files")

        for md_file in md_files:
            try:
                lesson_chunks = self.parse_lesson(md_file)
                chunks.extend(lesson_chunks)
            except Exception as e:
                print(f"Error parsing {md_file}: {e}")
                continue

        return chunks

    def parse_lesson(self, file_path: Path) -> List[BookChunk]:
        """
        Parse a single lesson markdown file into chunks.

        Args:
            file_path: Path to the markdown file

        Returns:
            List of BookChunk objects
        """
        # Read file with frontmatter
        with open(file_path, 'r', encoding='utf-8') as f:
            post = frontmatter.load(f)

        # Extract metadata from path and frontmatter
        metadata = self._extract_metadata(file_path, post)

        # Parse content into sections
        sections = self._split_by_headers(post.content)

        # Chunk sections
        chunks = []
        for section_idx, section in enumerate(sections):
            section_chunks = self._chunk_section(
                section,
                metadata,
                section_idx
            )
            chunks.extend(section_chunks)

        return chunks

    def _extract_metadata(self, file_path: Path, post: frontmatter.Post) -> Dict[str, Any]:
        """Extract metadata from file path and frontmatter."""
        # Parse path: docs/01-Chapter/02-lesson/file.md
        parts = file_path.relative_to(self.docs_path).parts

        # Extract chapter info
        chapter_dir = parts[0] if len(parts) > 0 else "unknown"
        chapter_match = re.match(r'(\d+)[-_](.+)', chapter_dir)

        if chapter_match:
            chapter_number = int(chapter_match.group(1))
            chapter_name = chapter_match.group(2).replace('-', ' ').replace('_', ' ')
            chapter_id = f"{chapter_number:02d}-{chapter_match.group(2)}"
        else:
            chapter_number = 0
            chapter_name = chapter_dir
            chapter_id = chapter_dir

        # Extract lesson info
        lesson_number = 0
        lesson_name = "readme"
        lesson_id = "readme"

        if len(parts) > 1 and parts[1] != file_path.name:
            lesson_dir = parts[1]
            lesson_match = re.match(r'(\d+)[-_](.+)', lesson_dir)
            if lesson_match:
                lesson_number = int(lesson_match.group(1))
                lesson_name = lesson_match.group(2).replace('-', ' ').replace('_', ' ')
                lesson_id = f"{lesson_number:02d}-{lesson_match.group(2)}"
            else:
                lesson_name = lesson_dir
                lesson_id = lesson_dir

        # Get title from frontmatter or derive from path
        title = post.get('title', chapter_name)

        # Determine difficulty based on chapter number
        difficulty = "beginner"
        if chapter_number >= 4:
            difficulty = "intermediate"
        if chapter_number >= 5:
            difficulty = "advanced"

        return {
            "chapter": chapter_id.lower(),
            "chapter_number": chapter_number,
            "chapter_title": title if 'Chapter' in title else chapter_name.title(),
            "lesson": lesson_id.lower(),
            "lesson_number": lesson_number,
            "lesson_title": lesson_name.title(),
            "file_path": str(file_path.relative_to(self.book_source_path.parent)),
            "file_name": file_path.name,
            "difficulty": difficulty,
            "indexed_at": datetime.utcnow().isoformat() + "Z",
            "updated_at": datetime.utcnow().isoformat() + "Z"
        }

    def _split_by_headers(self, content: str) -> List[Dict[str, str]]:
        """
        Split content by markdown headers while preserving structure.

        Returns:
            List of sections with heading and content
        """
        sections = []
        current_heading = None
        current_content = []

        lines = content.split('\n')

        for line in lines:
            # Check if line is a header (##, ###, ####)
            header_match = re.match(r'^(#{2,4})\s+(.+)$', line)

            if header_match:
                # Save previous section
                if current_content:
                    sections.append({
                        'heading': current_heading,
                        'content': '\n'.join(current_content).strip()
                    })

                # Start new section
                current_heading = header_match.group(2).strip()
                current_content = []
            else:
                current_content.append(line)

        # Save last section
        if current_content:
            sections.append({
                'heading': current_heading,
                'content': '\n'.join(current_content).strip()
            })

        return [s for s in sections if s['content']]  # Filter empty sections

    def _chunk_section(
        self,
        section: Dict[str, str],
        metadata: Dict[str, Any],
        section_idx: int
    ) -> List[BookChunk]:
        """
        Chunk a section into ~512 token pieces with overlap.

        For simplicity, we use ~300 words as proxy for 512 tokens (1 token ≈ 0.6 words).
        """
        content = section['content']
        heading = section['heading']

        # Simple chunking: split by paragraphs and group
        paragraphs = [p.strip() for p in content.split('\n\n') if p.strip()]

        chunks = []
        current_chunk = []
        current_words = 0
        chunk_idx = 0

        target_words = 300  # ~512 tokens
        overlap_words = 30  # ~50 tokens

        for para in paragraphs:
            para_words = len(para.split())

            # If paragraph alone exceeds target, split it
            if para_words > target_words:
                # Save current chunk if exists
                if current_chunk:
                    chunks.append(self._create_chunk(
                        '\n\n'.join(current_chunk),
                        metadata,
                        heading,
                        section_idx,
                        chunk_idx
                    ))
                    chunk_idx += 1
                    current_chunk = []
                    current_words = 0

                # Split large paragraph
                sentences = para.split('. ')
                temp_chunk = []
                temp_words = 0

                for sentence in sentences:
                    sent_words = len(sentence.split())
                    if temp_words + sent_words > target_words and temp_chunk:
                        chunks.append(self._create_chunk(
                            '. '.join(temp_chunk),
                            metadata,
                            heading,
                            section_idx,
                            chunk_idx
                        ))
                        chunk_idx += 1
                        # Overlap: keep last sentence
                        temp_chunk = [temp_chunk[-1]] if temp_chunk else []
                        temp_words = len(temp_chunk[-1].split()) if temp_chunk else 0

                    temp_chunk.append(sentence)
                    temp_words += sent_words

                if temp_chunk:
                    current_chunk = ['. '.join(temp_chunk)]
                    current_words = temp_words

            # Add paragraph to current chunk
            elif current_words + para_words > target_words:
                # Save current chunk
                chunks.append(self._create_chunk(
                    '\n\n'.join(current_chunk),
                    metadata,
                    heading,
                    section_idx,
                    chunk_idx
                ))
                chunk_idx += 1

                # Start new chunk with overlap
                overlap_paras = current_chunk[-1:] if current_chunk else []
                current_chunk = overlap_paras + [para]
                current_words = sum(len(p.split()) for p in current_chunk)
            else:
                current_chunk.append(para)
                current_words += para_words

        # Save last chunk
        if current_chunk:
            chunks.append(self._create_chunk(
                '\n\n'.join(current_chunk),
                metadata,
                heading,
                section_idx,
                chunk_idx
            ))

        return chunks

    def _create_chunk(
        self,
        content: str,
        metadata: Dict[str, Any],
        heading: str | None,
        section_idx: int,
        chunk_idx: int
    ) -> BookChunk:
        """Create a BookChunk with proper ID and metadata."""
        # Generate chunk ID with file hash to ensure uniqueness across multiple files in same lesson
        # Use hash of file path to create unique identifier
        import hashlib
        file_hash = hashlib.md5(metadata['file_path'].encode()).hexdigest()[:6]
        chunk_id = f"chunk_ch{metadata['chapter_number']:02d}_l{metadata['lesson_number']:02d}_{file_hash}_s{section_idx:02d}_c{chunk_idx:02d}"

        # Detect content type
        content_type = "text"
        if "```" in content:
            content_type = "code"
        elif heading and len(content.split()) < 50:
            content_type = "heading"

        # Extract topics from heading and content
        topics = self._extract_topics(heading, content)

        # Build chunk metadata
        # Convert topics list to comma-separated string (ChromaDB doesn't support list metadata)
        chunk_metadata = {
            **metadata,
            "chunk_index": section_idx * 100 + chunk_idx,
            "chunk_size": len(content.split()),
            "content_type": content_type,
            "heading": heading if heading else "",
            "topics": ", ".join(topics) if topics else ""
        }

        return BookChunk(
            chunk_id=chunk_id,
            content=content,
            metadata=chunk_metadata
        )

    def _extract_topics(self, heading: str | None, content: str) -> List[str]:
        """Extract topic keywords from heading and content."""
        topics = []

        # Add heading words as topics
        if heading:
            # Remove common words
            stop_words = {'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'is', 'are', 'was', 'were', 'you', 'your', 'this', 'that'}
            heading_words = [w.lower().strip('?!.,;:') for w in heading.split()]
            topics.extend([w for w in heading_words if w not in stop_words and len(w) > 2])

        # Extract code-related keywords
        code_keywords = ['python', 'async', 'await', 'def', 'class', 'import', 'function', 'variable', 'list', 'dict', 'loop', 'if', 'else', 'try', 'except']
        for keyword in code_keywords:
            if keyword in content.lower():
                topics.append(keyword)

        # Deduplicate and limit
        return list(set(topics))[:10]


# Test function
if __name__ == "__main__":
    import sys

    # Test parsing
    book_source_path = "/home/user/ai-native-software-development/Tutor/book-source"
    parser = BookContentParser(book_source_path)

    # Parse first file
    test_file = Path("/home/user/ai-native-software-development/Tutor/book-source/docs/01-Introducing-AI-Driven-Development/03-billion-dollar-ai/README.md")
    chunks = parser.parse_lesson(test_file)

    print(f"\nParsed {len(chunks)} chunks from test file")
    print("\nFirst chunk:")
    print(f"ID: {chunks[0].chunk_id}")
    print(f"Metadata: {chunks[0].metadata}")
    print(f"Content preview: {chunks[0].content[:200]}...")
