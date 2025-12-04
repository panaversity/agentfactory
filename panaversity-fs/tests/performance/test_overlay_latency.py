"""Performance tests for overlay read latency (T049).

Validates SC-006: Overlay read should be <10ms compared to base read.
The goal is to ensure personalization doesn't significantly impact read performance.
"""

import pytest
import time
import json
import statistics
from panaversity_fs.tools.content import read_content, write_content
from panaversity_fs.models import ReadContentInput, WriteContentInput
from panaversity_fs.storage import get_operator


class TestOverlayLatency:
    """Performance tests for SC-006 validation.

    SC-006: Overlay reads should add minimal latency (<10ms) compared to base reads.
    """

    @pytest.fixture
    async def setup_base_and_overlay(self, setup_fs_backend, mock_context):
        """Create base content and overlay for latency testing."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "perf-test-book"
        path = f"content/01-Part/01-Chapter/01-latency-test{unique_id}.md"
        user_id = f"user-{unique_id}"

        op = get_operator()

        # Create base content with substantial size for realistic testing
        base_content = "# Latency Test Lesson\n\n" + ("This is test content. " * 100 + "\n") * 50
        await op.write(f"books/{book_id}/{path}", base_content.encode())

        # Create overlay content
        overlay_content = "# User's Personalized Lesson\n\n" + ("Personalized content. " * 100 + "\n") * 50
        write_params = WriteContentInput(
            book_id=book_id,
            path=path,
            content=overlay_content,
            user_id=user_id
        )
        await write_content(write_params, mock_context)

        return {
            "book_id": book_id,
            "path": path,
            "user_id": user_id
        }

    @pytest.mark.asyncio
    async def test_overlay_read_latency_acceptable(self, setup_base_and_overlay, mock_context):
        """SC-006: Overlay read latency should be under 10ms compared to base.

        This test measures the latency difference between:
        1. Reading base content (no user_id)
        2. Reading with overlay (user_id provided, overlay exists)
        """
        config = setup_base_and_overlay
        iterations = 20

        base_times = []
        overlay_times = []

        # Warm up - first calls may have initialization overhead
        await read_content(ReadContentInput(
            book_id=config["book_id"],
            path=config["path"]
        ), mock_context)
        await read_content(ReadContentInput(
            book_id=config["book_id"],
            path=config["path"],
            user_id=config["user_id"]
        ), mock_context)

        # Measure base read latency
        for _ in range(iterations):
            start = time.perf_counter()
            result = await read_content(ReadContentInput(
                book_id=config["book_id"],
                path=config["path"]
            ), mock_context)
            end = time.perf_counter()
            assert "content" in result or "error" not in result.lower()
            base_times.append((end - start) * 1000)  # Convert to ms

        # Measure overlay read latency
        for _ in range(iterations):
            start = time.perf_counter()
            result = await read_content(ReadContentInput(
                book_id=config["book_id"],
                path=config["path"],
                user_id=config["user_id"]
            ), mock_context)
            end = time.perf_counter()
            data = json.loads(result)
            assert data["source"] == "overlay"
            overlay_times.append((end - start) * 1000)  # Convert to ms

        avg_base = statistics.mean(base_times)
        avg_overlay = statistics.mean(overlay_times)
        latency_diff = avg_overlay - avg_base

        print(f"\nLatency Results:")
        print(f"  Base read avg: {avg_base:.2f}ms")
        print(f"  Overlay read avg: {avg_overlay:.2f}ms")
        print(f"  Difference: {latency_diff:.2f}ms")
        print(f"  Base std dev: {statistics.stdev(base_times):.2f}ms")
        print(f"  Overlay std dev: {statistics.stdev(overlay_times):.2f}ms")

        # SC-006: Overlay should add less than 10ms latency
        assert latency_diff < 10, f"Overlay adds {latency_diff:.2f}ms latency (>10ms limit)"

    @pytest.mark.asyncio
    async def test_overlay_fallback_latency_acceptable(self, setup_fs_backend, mock_context):
        """SC-006: Fallback to base (no overlay exists) should be under 10ms compared to direct base read."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "perf-test-book"
        path = f"content/01-Part/01-Chapter/01-fallback-test{unique_id}.md"
        user_id = f"user-{unique_id}"

        op = get_operator()

        # Create only base content (no overlay)
        base_content = "# Fallback Test\n\n" + ("Content block. " * 100 + "\n") * 50
        await op.write(f"books/{book_id}/{path}", base_content.encode())

        iterations = 20
        base_times = []
        fallback_times = []

        # Warm up
        await read_content(ReadContentInput(book_id=book_id, path=path), mock_context)
        await read_content(ReadContentInput(book_id=book_id, path=path, user_id=user_id), mock_context)

        # Measure direct base read
        for _ in range(iterations):
            start = time.perf_counter()
            await read_content(ReadContentInput(book_id=book_id, path=path), mock_context)
            end = time.perf_counter()
            base_times.append((end - start) * 1000)

        # Measure fallback read (with user_id but no overlay)
        for _ in range(iterations):
            start = time.perf_counter()
            result = await read_content(ReadContentInput(
                book_id=book_id,
                path=path,
                user_id=user_id
            ), mock_context)
            end = time.perf_counter()
            data = json.loads(result)
            assert data["source"] == "base"  # Fallback to base
            fallback_times.append((end - start) * 1000)

        avg_base = statistics.mean(base_times)
        avg_fallback = statistics.mean(fallback_times)
        latency_diff = avg_fallback - avg_base

        print(f"\nFallback Latency Results:")
        print(f"  Direct base avg: {avg_base:.2f}ms")
        print(f"  Fallback avg: {avg_fallback:.2f}ms")
        print(f"  Difference: {latency_diff:.2f}ms")

        # SC-006: Fallback path should also add less than 10ms
        assert latency_diff < 10, f"Fallback adds {latency_diff:.2f}ms latency (>10ms limit)"

    @pytest.mark.asyncio
    async def test_many_users_overlay_isolation(self, setup_fs_backend, mock_context):
        """Verify that overlay operations remain fast even with many users."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "perf-test-book"
        path = f"content/01-Part/01-Chapter/01-multi-user{unique_id}.md"

        op = get_operator()

        # Create base content
        await op.write(f"books/{book_id}/{path}", b"# Base Content")

        # Create overlays for 10 users
        users = [f"user-{unique_id}-{i}" for i in range(10)]
        for user in users:
            write_params = WriteContentInput(
                book_id=book_id,
                path=path,
                content=f"# {user}'s content",
                user_id=user
            )
            await write_content(write_params, mock_context)

        # Measure read time for each user
        read_times = []
        for user in users:
            start = time.perf_counter()
            result = await read_content(ReadContentInput(
                book_id=book_id,
                path=path,
                user_id=user
            ), mock_context)
            end = time.perf_counter()
            data = json.loads(result)
            assert data["source"] == "overlay"
            read_times.append((end - start) * 1000)

        avg_time = statistics.mean(read_times)
        max_time = max(read_times)

        print(f"\nMulti-User Read Times:")
        print(f"  Average: {avg_time:.2f}ms")
        print(f"  Max: {max_time:.2f}ms")

        # All reads should complete quickly (under 50ms each)
        assert max_time < 50, f"Slowest read was {max_time:.2f}ms (>50ms limit)"
        assert avg_time < 20, f"Average read was {avg_time:.2f}ms (>20ms limit)"

    @pytest.mark.asyncio
    async def test_write_overlay_latency(self, setup_fs_backend, mock_context):
        """Verify overlay write latency is acceptable."""
        import uuid
        unique_id = str(uuid.uuid4())[:8]
        book_id = "perf-test-book"
        user_id = f"user-{unique_id}"

        op = get_operator()

        iterations = 10
        write_times = []

        for i in range(iterations):
            # Use unique path for each write to avoid hash requirement
            path = f"content/01-Part/01-Chapter/01-write{unique_id}{i}.md"

            # Create base content
            await op.write(f"books/{book_id}/{path}", b"# Base Content")

            content = f"# Iteration {i}\n\nContent for iteration {i}."
            start = time.perf_counter()
            result = await write_content(WriteContentInput(
                book_id=book_id,
                path=path,
                content=content,
                user_id=user_id
            ), mock_context)
            end = time.perf_counter()

            assert "success" in result
            write_times.append((end - start) * 1000)

        avg_time = statistics.mean(write_times)
        max_time = max(write_times)

        print(f"\nOverlay Write Times:")
        print(f"  Average: {avg_time:.2f}ms")
        print(f"  Max: {max_time:.2f}ms")

        # Writes should complete quickly (under 100ms each)
        assert max_time < 100, f"Slowest write was {max_time:.2f}ms (>100ms limit)"
        assert avg_time < 50, f"Average write was {avg_time:.2f}ms (>50ms limit)"
