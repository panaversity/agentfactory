"""Prometheus metrics instrumentation for PanaversityFS.

Provides metrics for monitoring success criteria (SC-001, SC-002, SC-006, SC-007).

Metrics Format:
- Counters: panaversityfs_write_total, panaversityfs_archive_total
- Histograms: panaversityfs_archive_duration_seconds, panaversityfs_write_duration_seconds
- Gauges: panaversityfs_archive_memory_bytes, panaversityfs_journal_entries_total
"""

import functools
import time
import tracemalloc
from contextlib import contextmanager
from typing import Callable, Literal, Any

from prometheus_client import Counter, Histogram, Gauge, CollectorRegistry, generate_latest


# =============================================================================
# Registry
# =============================================================================

# Create a dedicated registry for PanaversityFS metrics
# This allows isolation from any other Prometheus metrics in the process
REGISTRY = CollectorRegistry()


# =============================================================================
# Counters
# =============================================================================

# Write operations counter
# Labels: status (success, conflict, error), mode (create, update)
write_total = Counter(
    "panaversityfs_write_total",
    "Total write operations",
    ["status", "mode"],
    registry=REGISTRY,
)

# Archive operations counter
# Labels: scope (content, assets, all), status (success, timeout, error)
archive_total = Counter(
    "panaversityfs_archive_total",
    "Total archive generation operations",
    ["scope", "status"],
    registry=REGISTRY,
)

# Storage rollback counter (for SC-002)
storage_rollback_total = Counter(
    "panaversityfs_storage_rollback_total",
    "Total storage rollbacks after journal write failures",
    registry=REGISTRY,
)

# Audit chain validation counter (for SC-004)
audit_chain_check_total = Counter(
    "panaversityfs_audit_chain_check_total",
    "Total audit chain integrity checks",
    ["result"],  # valid, broken
    registry=REGISTRY,
)


# =============================================================================
# Histograms
# =============================================================================

# Archive duration histogram (for SC-001: <60s)
# Labels: scope (content, assets, all)
archive_duration_seconds = Histogram(
    "panaversityfs_archive_duration_seconds",
    "Archive generation duration in seconds",
    ["scope"],
    buckets=[1, 5, 10, 20, 30, 45, 60, 90, 120],  # Focus on 60s threshold
    registry=REGISTRY,
)

# Write duration histogram
# Labels: operation (journal, storage, total)
write_duration_seconds = Histogram(
    "panaversityfs_write_duration_seconds",
    "Write operation duration in seconds",
    ["operation"],
    buckets=[0.01, 0.05, 0.1, 0.25, 0.5, 1, 2.5, 5, 10],
    registry=REGISTRY,
)

# Overlay read latency histogram (for SC-006: <10ms additional)
overlay_latency_seconds = Histogram(
    "panaversityfs_overlay_latency_seconds",
    "Additional latency for overlay reads vs base reads",
    buckets=[0.001, 0.005, 0.010, 0.025, 0.050, 0.100],  # Focus on 10ms threshold
    registry=REGISTRY,
)


# =============================================================================
# Gauges
# =============================================================================

# Archive memory gauge (for SC-001: <64MB)
archive_memory_bytes = Gauge(
    "panaversityfs_archive_memory_bytes",
    "Current memory usage during archive generation",
    registry=REGISTRY,
)

# Journal entries gauge
# Labels: book_id
journal_entries_total = Gauge(
    "panaversityfs_journal_entries_total",
    "Number of entries in the file journal",
    ["book_id"],
    registry=REGISTRY,
)

# Delta build gauge (for SC-007)
delta_files_count = Gauge(
    "panaversityfs_delta_files_count",
    "Number of changed files in last delta build",
    registry=REGISTRY,
)


# =============================================================================
# Instrumentation Decorators
# =============================================================================

def instrument_write(func: Callable) -> Callable:
    """Decorator to instrument write operations.

    Tracks:
    - write_total counter (status, mode)
    - write_duration_seconds histogram (total)

    The decorated function should return a dict with 'mode' key ('created' or 'updated').
    Exceptions are tracked as status='error'.

    Usage:
        @instrument_write
        async def write_content(...):
            ...
            return {"mode": "created", ...}
    """
    @functools.wraps(func)
    async def wrapper(*args: Any, **kwargs: Any) -> Any:
        start_time = time.perf_counter()
        mode = "unknown"
        status = "success"

        try:
            result = await func(*args, **kwargs)
            # Extract mode from result if available
            if isinstance(result, dict) and "mode" in result:
                mode = result["mode"]
            return result
        except Exception as e:
            status = "error"
            # Check for specific error types
            from .errors import ConflictError
            if isinstance(e, ConflictError):
                status = "conflict"
            raise
        finally:
            duration = time.perf_counter() - start_time
            write_total.labels(status=status, mode=mode).inc()
            write_duration_seconds.labels(operation="total").observe(duration)

    return wrapper


def instrument_archive(scope: Literal["content", "assets", "all"] = "all") -> Callable:
    """Decorator factory to instrument archive generation.

    Tracks:
    - archive_total counter (scope, status)
    - archive_duration_seconds histogram
    - archive_memory_bytes gauge (during execution)

    Args:
        scope: Type of archive being generated

    Usage:
        @instrument_archive(scope="content")
        async def get_book_archive(...):
            ...
    """
    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        async def wrapper(*args: Any, **kwargs: Any) -> Any:
            start_time = time.perf_counter()
            status = "success"

            # Start memory tracking
            tracemalloc.start()

            try:
                result = await func(*args, **kwargs)
                return result
            except TimeoutError:
                status = "timeout"
                raise
            except Exception:
                status = "error"
                raise
            finally:
                duration = time.perf_counter() - start_time

                # Get peak memory usage
                current, peak = tracemalloc.get_traced_memory()
                tracemalloc.stop()

                # Update metrics
                archive_total.labels(scope=scope, status=status).inc()
                archive_duration_seconds.labels(scope=scope).observe(duration)
                archive_memory_bytes.set(peak)

        return wrapper
    return decorator


# =============================================================================
# Context Managers
# =============================================================================

@contextmanager
def track_memory():
    """Context manager to track memory usage during a block.

    Updates archive_memory_bytes gauge with peak memory.

    Usage:
        with track_memory():
            # Memory-intensive operation
            ...
    """
    tracemalloc.start()
    try:
        yield
    finally:
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        archive_memory_bytes.set(peak)


@contextmanager
def track_duration(histogram: Histogram, **labels: str):
    """Context manager to track duration of a block.

    Args:
        histogram: Prometheus Histogram to record to
        **labels: Labels to apply to the histogram

    Usage:
        with track_duration(write_duration_seconds, operation="journal"):
            # Timed operation
            ...
    """
    start_time = time.perf_counter()
    try:
        yield
    finally:
        duration = time.perf_counter() - start_time
        histogram.labels(**labels).observe(duration)


# =============================================================================
# Export Utilities
# =============================================================================

def get_metrics() -> bytes:
    """Get all metrics in Prometheus exposition format.

    Returns:
        Metrics data as bytes (UTF-8 encoded text)
    """
    return generate_latest(REGISTRY)


def reset_metrics() -> None:
    """Reset all metrics to initial state.

    Primarily for testing purposes.
    """
    # Note: prometheus_client doesn't have a built-in reset
    # For testing, create new registry or use REGISTRY.get_sample_value()
    pass
