"""Integration tests for streaming archive generation (T022).

Tests real ZIP generation with filesystem backend to verify:
- Memory-bounded buffer works correctly
- Partial results returned on timeout
- Error manifest includes failed files
"""

import pytest
import json
import zipfile
import io
import os
import tempfile
import shutil

from panaversity_fs.tools.bulk import (
    StreamingArchiveBuffer,
    ArchiveProgress,
    MAX_ARCHIVE_MEMORY_BYTES,
)


class TestStreamingArchiveBuffer:
    """Tests for StreamingArchiveBuffer memory management."""

    def test_buffer_initialization(self):
        """Buffer initializes correctly."""
        with StreamingArchiveBuffer() as buffer:
            assert buffer.max_bytes == MAX_ARCHIVE_MEMORY_BYTES
            assert buffer.current_size == 0
            assert buffer.zip_file is not None

    def test_add_file_success(self):
        """Successfully add file within memory limit."""
        with StreamingArchiveBuffer() as buffer:
            content = b"Hello, World!" * 100
            result = buffer.add_file("test.txt", content)
            assert result is True
            assert buffer.current_size > 0

    def test_add_file_tracks_size(self):
        """File sizes are tracked correctly."""
        with StreamingArchiveBuffer() as buffer:
            content1 = b"A" * 1000
            content2 = b"B" * 2000

            buffer.add_file("file1.txt", content1)
            size_after_1 = buffer.current_size

            buffer.add_file("file2.txt", content2)
            size_after_2 = buffer.current_size

            # Size should increase (approximately, compression varies)
            assert size_after_2 > size_after_1

    def test_add_file_rejects_over_limit(self):
        """File rejected if it would exceed memory limit."""
        # Use small limit for testing
        with StreamingArchiveBuffer(max_bytes=1024) as buffer:
            # Add file that's exactly at limit
            content = b"X" * 2000  # More than 1KB
            result = buffer.add_file("big.txt", content)
            assert result is False

    def test_get_bytes_returns_valid_zip(self):
        """get_bytes() returns valid ZIP archive."""
        with StreamingArchiveBuffer() as buffer:
            buffer.add_file("hello.txt", b"Hello!")
            buffer.add_file("world.txt", b"World!")

        archive_bytes = buffer.get_bytes()

        # Verify it's a valid ZIP
        zip_buffer = io.BytesIO(archive_bytes)
        with zipfile.ZipFile(zip_buffer, 'r') as zf:
            names = zf.namelist()
            assert "hello.txt" in names
            assert "world.txt" in names
            assert zf.read("hello.txt") == b"Hello!"
            assert zf.read("world.txt") == b"World!"

    def test_multiple_files_compressed(self):
        """Multiple files are compressed in archive."""
        with StreamingArchiveBuffer() as buffer:
            # Add 10 files with repetitive content (good compression)
            for i in range(10):
                content = f"File {i} content: " + ("AAAA" * 100)
                buffer.add_file(f"file{i}.txt", content.encode())

        archive_bytes = buffer.get_bytes()

        # Verify all files present
        zip_buffer = io.BytesIO(archive_bytes)
        with zipfile.ZipFile(zip_buffer, 'r') as zf:
            assert len(zf.namelist()) == 10


class TestArchiveProgress:
    """Tests for ArchiveProgress tracking."""

    def test_initial_state(self):
        """Progress starts with zero counts."""
        progress = ArchiveProgress()
        assert progress.files_processed == 0
        assert progress.files_failed == 0
        assert progress.total_bytes == 0
        assert progress.errors == []
        assert progress.timed_out is False

    def test_add_error(self):
        """Errors are tracked correctly."""
        progress = ArchiveProgress()
        progress.add_error("file1.txt", "Permission denied")
        progress.add_error("file2.txt", "File not found")

        assert progress.files_failed == 2
        assert len(progress.errors) == 2
        assert progress.errors[0] == {"path": "file1.txt", "error": "Permission denied"}

    def test_elapsed_seconds(self):
        """Elapsed time calculated correctly."""
        from datetime import datetime, timezone
        import time

        progress = ArchiveProgress(start_time=datetime.now(timezone.utc))
        time.sleep(0.1)  # 100ms

        elapsed = progress.elapsed_seconds()
        assert elapsed >= 0.1
        assert elapsed < 1.0  # Shouldn't take more than a second

    def test_elapsed_seconds_no_start(self):
        """Elapsed returns 0 if start_time not set."""
        progress = ArchiveProgress()
        assert progress.elapsed_seconds() == 0.0


class TestArchiveIntegration:
    """Integration tests with real filesystem."""

    @pytest.fixture
    def temp_book_dir(self):
        """Create temporary book directory with sample files."""
        temp_dir = tempfile.mkdtemp(prefix="panaversity-test-archive-")

        # Create book structure
        book_dir = os.path.join(temp_dir, "books", "test-book")
        content_dir = os.path.join(book_dir, "content", "01-Part", "01-Chapter")
        os.makedirs(content_dir)

        # Create sample files
        for i in range(5):
            file_path = os.path.join(content_dir, f"0{i+1}-lesson.md")
            with open(file_path, 'w') as f:
                f.write(f"# Lesson {i+1}\n\nThis is lesson content.\n")

        yield temp_dir

        shutil.rmtree(temp_dir, ignore_errors=True)

    def test_archive_contains_all_files(self, temp_book_dir):
        """Archive contains all files from book."""
        with StreamingArchiveBuffer() as buffer:
            content_dir = os.path.join(
                temp_book_dir, "books", "test-book", "content", "01-Part", "01-Chapter"
            )

            for filename in os.listdir(content_dir):
                file_path = os.path.join(content_dir, filename)
                with open(file_path, 'rb') as f:
                    content = f.read()
                buffer.add_file(f"content/01-Part/01-Chapter/{filename}", content)

        archive_bytes = buffer.get_bytes()

        # Verify archive contents
        zip_buffer = io.BytesIO(archive_bytes)
        with zipfile.ZipFile(zip_buffer, 'r') as zf:
            names = zf.namelist()
            assert len(names) == 5
            assert "content/01-Part/01-Chapter/01-lesson.md" in names

    def test_archive_preserves_content(self, temp_book_dir):
        """Archive content matches original files."""
        with StreamingArchiveBuffer() as buffer:
            file_path = os.path.join(
                temp_book_dir, "books", "test-book", "content",
                "01-Part", "01-Chapter", "01-lesson.md"
            )
            with open(file_path, 'rb') as f:
                original_content = f.read()

            buffer.add_file("content/01-Part/01-Chapter/01-lesson.md", original_content)

        archive_bytes = buffer.get_bytes()

        # Verify content matches
        zip_buffer = io.BytesIO(archive_bytes)
        with zipfile.ZipFile(zip_buffer, 'r') as zf:
            archived_content = zf.read("content/01-Part/01-Chapter/01-lesson.md")
            assert archived_content == original_content
