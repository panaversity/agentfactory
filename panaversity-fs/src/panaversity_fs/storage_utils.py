"""Storage utility functions for PanaversityFS.

Provides helpers for:
- File hash computation (SHA256 for conflict detection)
- Path validation and sanitization
- Content type detection
"""

import hashlib
import re
from pathlib import Path


def compute_sha256(content: bytes) -> str:
    """Compute SHA256 hash of content.

    Used for conflict detection in write_content tool (FR-008).

    Args:
        content: Binary content to hash.

    Returns:
        str: Lowercase hex-encoded SHA256 hash (64 characters).

    Example:
        ```python
        content = b"# Hello World"
        hash_value = compute_sha256(content)
        # "a591a6d40bf420404a011733cfb7b190d62c65bf0bcda32b57b277d9ad9f146e"
        ```
    """
    return hashlib.sha256(content).hexdigest()


def validate_path(path: str) -> bool:
    """Validate file path for security and correctness.

    Checks:
    - No directory traversal attempts (../)
    - No absolute paths
    - Only alphanumeric, hyphens, underscores, slashes, dots
    - No consecutive slashes
    - Not empty

    Args:
        path: File path to validate.

    Returns:
        bool: True if path is valid, False otherwise.

    Example:
        ```python
        validate_path("lessons/part-1/chapter-01/lesson-01.md")  # True
        validate_path("../etc/passwd")  # False
        validate_path("/absolute/path")  # False
        validate_path("lessons//double-slash")  # False
        ```
    """
    if not path or not path.strip():
        return False

    # No absolute paths
    if path.startswith('/'):
        return False

    # No directory traversal
    if '..' in path:
        return False

    # No consecutive slashes
    if '//' in path:
        return False

    # Only allowed characters: a-z, A-Z, 0-9, -, _, /, .
    if not re.match(r'^[a-zA-Z0-9/_.-]+$', path):
        return False

    return True


def sanitize_filename(filename: str) -> str:
    """Sanitize filename by removing or replacing unsafe characters.

    Replaces spaces with hyphens, removes special characters except dots/hyphens.

    Args:
        filename: Original filename.

    Returns:
        str: Sanitized filename safe for storage.

    Example:
        ```python
        sanitize_filename("My File (v2).pdf")  # "My-File-v2.pdf"
        sanitize_filename("test@#$%.png")  # "test.png"
        ```
    """
    # Replace spaces with hyphens
    filename = filename.replace(' ', '-')

    # Remove all characters except alphanumeric, dots, hyphens
    filename = re.sub(r'[^a-zA-Z0-9.-]', '', filename)

    # Remove consecutive hyphens
    filename = re.sub(r'-+', '-', filename)

    # Remove leading/trailing hyphens or dots
    filename = filename.strip('-.')

    return filename


def get_mime_type(filename: str) -> str:
    """Guess MIME type from filename extension.

    Uses common educational content file types.

    Args:
        filename: Filename with extension.

    Returns:
        str: MIME type string.

    Example:
        ```python
        get_mime_type("lesson.md")  # "text/markdown"
        get_mime_type("slides.pdf")  # "application/pdf"
        get_mime_type("diagram.png")  # "image/png"
        ```
    """
    extension = Path(filename).suffix.lower()

    mime_types = {
        # Markdown
        '.md': 'text/markdown',

        # Images
        '.png': 'image/png',
        '.jpg': 'image/jpeg',
        '.jpeg': 'image/jpeg',
        '.gif': 'image/gif',
        '.svg': 'image/svg+xml',
        '.webp': 'image/webp',

        # Documents
        '.pdf': 'application/pdf',

        # Videos
        '.mp4': 'video/mp4',
        '.webm': 'video/webm',
        '.mov': 'video/quicktime',

        # Audio
        '.mp3': 'audio/mpeg',
        '.wav': 'audio/wav',
        '.ogg': 'audio/ogg',

        # YAML
        '.yaml': 'application/x-yaml',
        '.yml': 'application/x-yaml',

        # JSON
        '.json': 'application/json',
        '.jsonl': 'application/jsonlines',
    }

    return mime_types.get(extension, 'application/octet-stream')


def build_cdn_url(base_url: str, book_id: str, asset_type: str, filename: str) -> str:
    """Build CDN URL for asset (FR-013).

    Format: https://cdn.panaversity.com/books/[book-id]/assets/[type]/[filename]

    Args:
        base_url: CDN base URL (e.g., "https://cdn.panaversity.com")
        book_id: Book identifier
        asset_type: Asset type (slides/images/videos/audio)
        filename: Asset filename

    Returns:
        str: Complete CDN URL

    Example:
        ```python
        url = build_cdn_url(
            "https://cdn.panaversity.com",
            "ai-native-python",
            "images",
            "diagram.png"
        )
        # "https://cdn.panaversity.com/books/ai-native-python/assets/images/diagram.png"
        ```
    """
    # Remove trailing slash from base_url
    base_url = base_url.rstrip('/')

    return f"{base_url}/books/{book_id}/assets/{asset_type}/{filename}"


