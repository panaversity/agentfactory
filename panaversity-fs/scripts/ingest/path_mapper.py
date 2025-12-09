"""Path mapping from book-source to PanaversityFS schema.

Maps source directory structure to the PanaversityFS content schema:
- Source: Part-01/Chapter-02/03-lesson.md
- Target: content/01-Part/02-Chapter/03-lesson.md

The mapper validates paths against FR-007 (content) and FR-008 (assets) patterns.
"""

import re
from dataclasses import dataclass
from typing import Optional
from enum import Enum
from pathlib import Path


class ContentType(Enum):
    """Type of content file."""
    MARKDOWN = "markdown"
    SUMMARY = "summary"
    ASSET = "asset"
    README = "readme"
    UNKNOWN = "unknown"


@dataclass
class MappedPath:
    """Result of path mapping.

    Attributes:
        source_path: Original source path
        storage_path: Mapped PanaversityFS path
        content_type: Type of content
        valid: Whether the mapping is valid
        error: Error message if invalid
    """
    source_path: str
    storage_path: Optional[str]
    content_type: ContentType
    valid: bool
    error: Optional[str] = None


# Patterns for source path parsing
# Source format: Part-NN/Chapter-NN/NN-name.md
PART_PATTERN = re.compile(r"Part-(\d+)")
CHAPTER_PATTERN = re.compile(r"Chapter-(\d+)")
LESSON_PATTERN = re.compile(r"(\d+)-([a-zA-Z0-9-]+)(\.summary)?\.md$")

# Asset patterns
ASSET_EXTENSIONS = {".png", ".jpg", ".jpeg", ".gif", ".svg", ".webp", ".mp4", ".mp3", ".pdf"}


def _sanitize_path_component(component: str) -> str:
    """Sanitize a path component to prevent path traversal attacks.

    Args:
        component: Path component to sanitize

    Returns:
        Sanitized component safe for path construction
    """
    # Remove path traversal sequences
    component = component.replace("..", "")
    component = component.replace("/", "")
    component = component.replace("\\", "")
    # Remove null bytes
    component = component.replace("\0", "")
    return component


def map_source_to_storage(source_path: str) -> MappedPath:
    """Map a source path to PanaversityFS storage path.

    Args:
        source_path: Path relative to book-source/docs/ (e.g., "Part-01/Chapter-02/03-lesson.md")

    Returns:
        MappedPath with storage_path and validity information

    Mapping Rules:
        - Part-NN/Chapter-NN/NN-name.md → content/NN-Part/NN-Chapter/NN-name.md
        - Part-NN/Chapter-NN/NN-name.summary.md → content/NN-Part/NN-Chapter/NN-name.summary.md
        - Part-NN/Chapter-NN/img/*.png → static/img/*.png
        - README.md files → ignored (not synced)
    """
    # Normalize path separators
    source_path = source_path.replace("\\", "/")

    # Skip README files
    if source_path.lower().endswith("readme.md"):
        return MappedPath(
            source_path=source_path,
            storage_path=None,
            content_type=ContentType.README,
            valid=False,
            error="README files are not synced"
        )

    # Check for asset files
    lower_path = source_path.lower()
    for ext in ASSET_EXTENSIONS:
        if lower_path.endswith(ext):
            return _map_asset_path(source_path)

    # Handle markdown content
    if source_path.endswith(".md"):
        return _map_content_path(source_path)

    # Unknown file type
    return MappedPath(
        source_path=source_path,
        storage_path=None,
        content_type=ContentType.UNKNOWN,
        valid=False,
        error=f"Unknown file type: {source_path}"
    )


def _map_content_path(source_path: str) -> MappedPath:
    """Map a markdown content path."""
    parts = source_path.split("/")

    # Need at least Part/Chapter/file.md
    if len(parts) < 3:
        return MappedPath(
            source_path=source_path,
            storage_path=None,
            content_type=ContentType.MARKDOWN,
            valid=False,
            error=f"Path too short, expected Part/Chapter/file.md structure: {source_path}"
        )

    # Extract Part number
    part_match = PART_PATTERN.match(parts[0])
    if not part_match:
        return MappedPath(
            source_path=source_path,
            storage_path=None,
            content_type=ContentType.MARKDOWN,
            valid=False,
            error=f"Invalid Part format, expected 'Part-NN': {parts[0]}"
        )
    part_num = part_match.group(1).zfill(2)

    # Extract Chapter number
    chapter_match = CHAPTER_PATTERN.match(parts[1])
    if not chapter_match:
        return MappedPath(
            source_path=source_path,
            storage_path=None,
            content_type=ContentType.MARKDOWN,
            valid=False,
            error=f"Invalid Chapter format, expected 'Chapter-NN': {parts[1]}"
        )
    chapter_num = chapter_match.group(1).zfill(2)

    # Extract filename
    filename = parts[-1]
    lesson_match = LESSON_PATTERN.match(filename)

    if not lesson_match:
        return MappedPath(
            source_path=source_path,
            storage_path=None,
            content_type=ContentType.MARKDOWN,
            valid=False,
            error=f"Invalid filename format, expected 'NN-name.md' or 'NN-name.summary.md': {filename}"
        )

    lesson_num = lesson_match.group(1).zfill(2)
    lesson_name = lesson_match.group(2)
    is_summary = lesson_match.group(3) is not None

    # Sanitize lesson_name to prevent path traversal
    lesson_name = _sanitize_path_component(lesson_name)

    # Build storage path
    storage_path = f"content/{part_num}-Part/{chapter_num}-Chapter/{lesson_num}-{lesson_name}"
    if is_summary:
        storage_path += ".summary"
    storage_path += ".md"

    # Validate that resolved path stays within content directory
    try:
        resolved = Path(storage_path).resolve()
        content_dir = Path("content").resolve()
        # Check if resolved path is relative to content directory
        resolved.relative_to(content_dir)
    except (ValueError, RuntimeError):
        return MappedPath(
            source_path=source_path,
            storage_path=None,
            content_type=ContentType.MARKDOWN,
            valid=False,
            error=f"Path traversal detected or invalid path: {source_path}"
        )

    content_type = ContentType.SUMMARY if is_summary else ContentType.MARKDOWN

    return MappedPath(
        source_path=source_path,
        storage_path=storage_path,
        content_type=content_type,
        valid=True
    )


def _map_asset_path(source_path: str) -> MappedPath:
    """Map an asset path."""
    parts = source_path.split("/")

    # Find the asset type directory (img, slides, videos, audio)
    # Note: "img" variants normalize to "images" to match server storage
    asset_type_aliases = {
        "img": "images",
        "image": "images",
        "images": "images",
        "slides": "slides",
        "videos": "videos",
        "audio": "audio"
    }
    asset_type = None
    asset_filename = None

    for i, part in enumerate(parts):
        normalized = asset_type_aliases.get(part.lower())
        if normalized:
            asset_type = normalized
            # Everything after asset type is the filename (may include subdirs)
            asset_filename = "/".join(parts[i + 1:])
            break

    if not asset_type or not asset_filename:
        # No recognized asset type directory, use "images" as default
        asset_type = "images"
        asset_filename = parts[-1]

    # Sanitize asset filename components to prevent path traversal
    sanitized_parts = [_sanitize_path_component(p) for p in asset_filename.split("/")]
    asset_filename = "/".join(sanitized_parts)

    storage_path = f"static/{asset_type}/{asset_filename}"

    # Validate that resolved path stays within static directory
    try:
        resolved = Path(storage_path).resolve()
        static_dir = Path("static").resolve()
        resolved.relative_to(static_dir)
    except (ValueError, RuntimeError):
        return MappedPath(
            source_path=source_path,
            storage_path=None,
            content_type=ContentType.ASSET,
            valid=False,
            error=f"Path traversal detected or invalid path: {source_path}"
        )

    return MappedPath(
        source_path=source_path,
        storage_path=storage_path,
        content_type=ContentType.ASSET,
        valid=True
    )


def validate_storage_path(storage_path: str) -> tuple[bool, Optional[str]]:
    """Validate a storage path against FR-007/FR-008 patterns.

    Args:
        storage_path: PanaversityFS storage path to validate

    Returns:
        Tuple of (is_valid, error_message)
    """
    # Content path pattern (FR-007)
    content_pattern = re.compile(
        r"^content/\d{2}-[A-Za-z0-9-]+/\d{2}-[A-Za-z0-9-]+/\d{2}-[a-z0-9-]+(\.summary)?\.md$"
    )

    # Asset path pattern (FR-008)
    # Note: Accept both "img" and "images" for backwards compatibility
    asset_pattern = re.compile(
        r"^static/(img|images|slides|videos|audio)/.+"
    )

    if storage_path.startswith("content/"):
        if content_pattern.match(storage_path):
            return True, None
        return False, f"Content path does not match schema: {storage_path}"

    if storage_path.startswith("static/"):
        if asset_pattern.match(storage_path):
            return True, None
        return False, f"Asset path does not match schema: {storage_path}"

    return False, f"Unknown path type: {storage_path}"


def map_and_validate(source_path: str) -> MappedPath:
    """Map source path and validate against schema.

    Combines mapping and validation in one step.

    Args:
        source_path: Source file path

    Returns:
        MappedPath with validation applied
    """
    mapped = map_source_to_storage(source_path)

    if not mapped.valid:
        return mapped

    # Validate against schema
    is_valid, error = validate_storage_path(mapped.storage_path)

    if not is_valid:
        return MappedPath(
            source_path=source_path,
            storage_path=mapped.storage_path,
            content_type=mapped.content_type,
            valid=False,
            error=error
        )

    return mapped
