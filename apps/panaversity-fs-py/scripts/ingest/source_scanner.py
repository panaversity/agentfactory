"""Source directory scanner for book content.

Walks a book source directory, identifies content files and assets,
and computes hashes for change detection.
"""

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

from scripts.ingest.path_mapper import (
    ContentType,
    MappedPath,
    map_and_validate,
    ASSET_EXTENSIONS,
)


@dataclass
class SourceFile:
    """A file discovered in the source directory.

    Attributes:
        relative_path: Path relative to source root (e.g., "Part-01/Chapter-02/03-lesson.md")
        absolute_path: Full filesystem path
        content_hash: SHA256 hash of file content
        size_bytes: File size in bytes
        mapped: Result of path mapping to storage schema
    """
    relative_path: str
    absolute_path: Path
    content_hash: str
    size_bytes: int
    mapped: MappedPath


@dataclass
class ScanResult:
    """Result of scanning a source directory.

    Attributes:
        source_root: Root directory that was scanned
        files: List of discovered source files
        valid_count: Number of files with valid storage mappings
        invalid_count: Number of files with invalid mappings (skipped)
        total_bytes: Total size of valid files
        errors: List of errors encountered during scanning
    """
    source_root: Path
    files: list[SourceFile]
    valid_count: int
    invalid_count: int
    total_bytes: int
    errors: list[dict]


def compute_hash(file_path: Path, content_type: ContentType) -> str:
    """Compute SHA256 hash of a file.

    For text files (markdown, summary), normalizes content by stripping trailing whitespace
    to match server behavior. For binary files (assets), uses raw bytes.

    Args:
        file_path: Path to the file
        content_type: Type of content (MARKDOWN, SUMMARY, or ASSET)

    Returns:
        Hexadecimal SHA256 hash string
    """
    sha256 = hashlib.sha256()

    if content_type in (ContentType.MARKDOWN, ContentType.SUMMARY):
        # For text files, normalize content to match server behavior
        # Server strips trailing newlines from end of file, but preserves everything else
        content = file_path.read_text(encoding="utf-8")
        # Strip only trailing newlines (not spaces or other whitespace)
        normalized = content.rstrip("\n")
        sha256.update(normalized.encode("utf-8"))
    else:
        # For binary files (assets), use raw bytes
        with open(file_path, "rb") as f:
            for chunk in iter(lambda: f.read(8192), b""):
                sha256.update(chunk)

    return sha256.hexdigest()


def is_content_file(path: Path) -> bool:
    """Check if a path is a content file we should process.

    Args:
        path: Path to check

    Returns:
        True if this is a markdown or asset file
    """
    suffix = path.suffix.lower()

    # Markdown files
    if suffix == ".md":
        return True

    # Asset files
    if suffix in ASSET_EXTENSIONS:
        return True

    return False


def should_skip_directory(name: str) -> bool:
    """Check if a directory should be skipped during scanning.

    Args:
        name: Directory name

    Returns:
        True if directory should be skipped
    """
    # Skip hidden directories
    if name.startswith("."):
        return True

    # Skip common non-content directories
    skip_dirs = {
        "__pycache__",
        "node_modules",
        ".git",
        ".docusaurus",
        "build",
        "dist",
    }
    return name in skip_dirs


def scan_directory(source_root: Path, verbose: bool = False) -> Iterator[SourceFile]:
    """Scan a directory for content files.

    Args:
        source_root: Root directory to scan
        verbose: Print verbose output

    Yields:
        SourceFile objects for each discovered content file
    """
    source_root = source_root.resolve()

    for path in source_root.rglob("*"):
        # Skip directories
        if path.is_dir():
            continue

        # Check if any parent directory should be skipped
        skip = False
        for parent in path.relative_to(source_root).parents:
            if parent.name and should_skip_directory(parent.name):
                skip = True
                break
        if skip:
            continue

        # Check if this is a content file
        if not is_content_file(path):
            continue

        # Compute relative path
        relative_path = str(path.relative_to(source_root))

        # Map to storage path
        mapped = map_and_validate(relative_path)

        # Compute hash (normalized for text, raw for binary)
        try:
            content_hash = compute_hash(path, mapped.content_type)
            size_bytes = path.stat().st_size
        except (OSError, IOError) as e:
            if verbose:
                print(f"  Warning: Could not read {relative_path}: {e}")
            continue

        yield SourceFile(
            relative_path=relative_path,
            absolute_path=path,
            content_hash=content_hash,
            size_bytes=size_bytes,
            mapped=mapped
        )


def scan_source_directory(
    source_root: Path,
    verbose: bool = False
) -> ScanResult:
    """Scan a book source directory for content files.

    This is the main entry point for scanning. It walks the directory,
    discovers all content files, maps them to storage paths, and
    computes hashes for change detection.

    Args:
        source_root: Root directory to scan (e.g., apps/learn-app/docs/)
        verbose: Print verbose output

    Returns:
        ScanResult with all discovered files and statistics

    Raises:
        ValueError: If source_root doesn't exist or isn't a directory
    """
    source_root = Path(source_root).resolve()

    if not source_root.exists():
        raise ValueError(f"Source directory does not exist: {source_root}")

    if not source_root.is_dir():
        raise ValueError(f"Source path is not a directory: {source_root}")

    if verbose:
        print(f"Scanning: {source_root}")

    files: list[SourceFile] = []
    errors: list[dict] = []
    valid_count = 0
    invalid_count = 0
    total_bytes = 0

    for source_file in scan_directory(source_root, verbose):
        files.append(source_file)

        if source_file.mapped.valid:
            valid_count += 1
            total_bytes += source_file.size_bytes
            if verbose:
                print(f"  Found: {source_file.relative_path} -> {source_file.mapped.storage_path}")
        else:
            invalid_count += 1
            if verbose:
                print(f"  Skipped: {source_file.relative_path} ({source_file.mapped.error})")
            errors.append({
                "path": source_file.relative_path,
                "error": source_file.mapped.error
            })

    if verbose:
        print(f"Scan complete: {valid_count} valid, {invalid_count} skipped")

    return ScanResult(
        source_root=source_root,
        files=files,
        valid_count=valid_count,
        invalid_count=invalid_count,
        total_bytes=total_bytes,
        errors=errors
    )


def filter_by_content_type(
    files: list[SourceFile],
    content_type: ContentType
) -> list[SourceFile]:
    """Filter source files by content type.

    Args:
        files: List of source files to filter
        content_type: Type to filter for

    Returns:
        Filtered list of source files
    """
    return [f for f in files if f.mapped.content_type == content_type]


def get_valid_files(files: list[SourceFile]) -> list[SourceFile]:
    """Get only valid files from a list.

    Args:
        files: List of source files

    Returns:
        List of files with valid storage mappings
    """
    return [f for f in files if f.mapped.valid]
