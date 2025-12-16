"""Ingestion scripts for PanaversityFS.

This package provides tools for syncing book source content to PanaversityFS
storage, handling path mapping and conflict detection.

Modules:
    path_mapper: Maps source paths to PanaversityFS storage schema
    source_scanner: Scans source directories for content files
    sync_engine: Synchronizes source to storage with delta detection
    cli: Command-line interface for ingestion
"""

from scripts.ingest.path_mapper import (
    ContentType,
    MappedPath,
    map_source_to_storage,
    map_and_validate,
    validate_storage_path,
)
from scripts.ingest.source_scanner import (
    SourceFile,
    ScanResult,
    scan_source_directory,
    get_valid_files,
    filter_by_content_type,
)
from scripts.ingest.sync_engine import (
    SyncAction,
    SyncResult,
    sync_source_to_storage,
    compute_sync_plan,
    execute_sync_plan,
)

__all__ = [
    # Path mapper
    "ContentType",
    "MappedPath",
    "map_source_to_storage",
    "map_and_validate",
    "validate_storage_path",
    # Source scanner
    "SourceFile",
    "ScanResult",
    "scan_source_directory",
    "get_valid_files",
    "filter_by_content_type",
    # Sync engine
    "SyncAction",
    "SyncResult",
    "sync_source_to_storage",
    "compute_sync_plan",
    "execute_sync_plan",
]
