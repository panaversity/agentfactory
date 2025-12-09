"""Hydration scripts for PanaversityFS.

This package provides tools for hydrating Docusaurus builds with content
from PanaversityFS storage using incremental delta detection.

Modules:
    manifest: Manifest file management for tracking build state
    downloader: File downloader with streaming support
    cli: Command-line interface for hydration
"""

from scripts.hydrate.manifest import (
    ManifestFile,
    load_manifest,
    save_manifest,
    delete_manifest,
    get_default_manifest_path,
)
from scripts.hydrate.downloader import (
    DownloadResult,
    download_changed_files,
    download_all_content,
    download_delta,
    format_bytes,
)

__all__ = [
    # Manifest
    "ManifestFile",
    "load_manifest",
    "save_manifest",
    "delete_manifest",
    "get_default_manifest_path",
    # Downloader
    "DownloadResult",
    "download_changed_files",
    "download_all_content",
    "download_delta",
    "format_bytes",
]
