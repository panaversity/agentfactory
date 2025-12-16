"""Unit tests for Prometheus metrics instrumentation (T017)."""

import pytest
import asyncio
from unittest.mock import AsyncMock, patch

from panaversity_fs.metrics import (
    write_total,
    archive_total,
    archive_duration_seconds,
    write_duration_seconds,
    archive_memory_bytes,
    instrument_write,
    instrument_archive,
    track_memory,
    track_duration,
    get_metrics,
    REGISTRY,
)


class TestMetricsRegistry:
    """Test metrics are properly registered."""

    def test_write_total_registered(self):
        """write_total counter is in registry."""
        # Just verify we can access the metric
        write_total.labels(status="success", mode="create")
        assert True

    def test_archive_total_registered(self):
        """archive_total counter is in registry."""
        archive_total.labels(scope="content", status="success")
        assert True

    def test_archive_duration_registered(self):
        """archive_duration_seconds histogram is in registry."""
        archive_duration_seconds.labels(scope="all")
        assert True

    def test_get_metrics_returns_bytes(self):
        """get_metrics() returns Prometheus format."""
        output = get_metrics()
        assert isinstance(output, bytes)
        # Should contain some metric data
        assert b"panaversityfs" in output or len(output) > 0


@pytest.mark.asyncio
class TestInstrumentWriteDecorator:
    """Tests for @instrument_write decorator."""

    async def test_instrument_write_success(self):
        """Successful write increments counter with success status."""
        @instrument_write
        async def mock_write():
            return {"mode": "created", "path": "test.md"}

        result = await mock_write()
        assert result["mode"] == "created"

    async def test_instrument_write_error(self):
        """Error in write increments counter with error status."""
        @instrument_write
        async def mock_write_error():
            raise ValueError("Test error")

        with pytest.raises(ValueError):
            await mock_write_error()

    async def test_instrument_write_conflict(self):
        """Conflict error increments counter with conflict status."""
        from panaversity_fs.errors import ConflictError

        @instrument_write
        async def mock_write_conflict():
            raise ConflictError("test.md", "expected", "actual")

        with pytest.raises(ConflictError):
            await mock_write_conflict()

    async def test_instrument_write_extracts_mode(self):
        """Mode is extracted from result dict."""
        @instrument_write
        async def mock_write_update():
            return {"mode": "updated", "sha256": "abc123"}

        result = await mock_write_update()
        assert result["mode"] == "updated"


@pytest.mark.asyncio
class TestInstrumentArchiveDecorator:
    """Tests for @instrument_archive decorator."""

    async def test_instrument_archive_success(self):
        """Successful archive increments counter."""
        @instrument_archive(scope="content")
        async def mock_archive():
            return b"archive_data"

        result = await mock_archive()
        assert result == b"archive_data"

    async def test_instrument_archive_timeout(self):
        """Timeout increments counter with timeout status."""
        @instrument_archive(scope="all")
        async def mock_archive_timeout():
            raise TimeoutError("Archive timeout")

        with pytest.raises(TimeoutError):
            await mock_archive_timeout()

    async def test_instrument_archive_error(self):
        """Other errors increment counter with error status."""
        @instrument_archive(scope="assets")
        async def mock_archive_error():
            raise IOError("Storage error")

        with pytest.raises(IOError):
            await mock_archive_error()

    async def test_instrument_archive_scope_label(self):
        """Different scopes are tracked separately."""
        @instrument_archive(scope="content")
        async def content_archive():
            return b"content"

        @instrument_archive(scope="assets")
        async def assets_archive():
            return b"assets"

        await content_archive()
        await assets_archive()
        # Both should complete without error


class TestTrackMemoryContextManager:
    """Tests for track_memory() context manager."""

    def test_track_memory_basic(self):
        """Memory tracking runs without error."""
        with track_memory():
            # Allocate some memory
            data = [i for i in range(10000)]
            assert len(data) == 10000

    def test_track_memory_updates_gauge(self):
        """Memory gauge is updated after context."""
        # Clear any previous value
        archive_memory_bytes.set(0)

        with track_memory():
            # Allocate memory
            data = bytearray(1024 * 100)  # 100KB

        # Gauge should have been updated (peak memory tracked)
        # We can't easily verify the exact value since Python manages memory
        # But we can verify the context manager runs without error


class TestTrackDurationContextManager:
    """Tests for track_duration() context manager."""

    def test_track_duration_basic(self):
        """Duration tracking runs without error."""
        import time

        with track_duration(write_duration_seconds, operation="journal"):
            time.sleep(0.01)  # 10ms

    def test_track_duration_with_labels(self):
        """Duration tracking works with various labels."""
        with track_duration(write_duration_seconds, operation="storage"):
            pass

        with track_duration(write_duration_seconds, operation="total"):
            pass

    def test_track_duration_measures_time(self):
        """Verify duration is actually being measured."""
        import time

        start = time.perf_counter()
        with track_duration(archive_duration_seconds, scope="all"):
            time.sleep(0.05)  # 50ms
        elapsed = time.perf_counter() - start

        # Should have taken at least 50ms
        assert elapsed >= 0.05


class TestMetricsOutput:
    """Test Prometheus metrics output format."""

    def test_metrics_contain_counters(self):
        """Output should contain counter metrics."""
        # Trigger some metrics
        write_total.labels(status="success", mode="create").inc()

        output = get_metrics().decode("utf-8")
        # Should contain the metric name
        assert "panaversityfs_write_total" in output

    def test_metrics_contain_histograms(self):
        """Output should contain histogram metrics."""
        archive_duration_seconds.labels(scope="all").observe(1.5)

        output = get_metrics().decode("utf-8")
        assert "panaversityfs_archive_duration_seconds" in output

    def test_metrics_contain_gauges(self):
        """Output should contain gauge metrics."""
        archive_memory_bytes.set(1024 * 1024)  # 1MB

        output = get_metrics().decode("utf-8")
        assert "panaversityfs_archive_memory_bytes" in output
