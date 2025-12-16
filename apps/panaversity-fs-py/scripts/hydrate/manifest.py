"""Manifest file management for incremental builds.

The manifest tracks the state of a previous build, enabling delta detection
for subsequent builds. By comparing the stored manifest hash with the current
state in PanaversityFS, we can determine which files have changed.

Manifest file format:
{
    "book_id": "ai-native-python",
    "manifest_hash": "abc123...",
    "timestamp": "2025-12-08T00:00:00Z",
    "file_count": 100
}
"""

import json
import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from pydantic import BaseModel, Field


logger = logging.getLogger(__name__)


class ManifestFile(BaseModel):
    """Manifest file model for tracking build state.

    Attributes:
        book_id: Identifier of the book
        manifest_hash: SHA256 hash representing the book's content state
        timestamp: ISO 8601 timestamp of when manifest was created
        file_count: Number of files in the book at manifest creation
    """
    book_id: str
    manifest_hash: str
    timestamp: str = Field(default_factory=lambda: datetime.now(timezone.utc).isoformat())
    file_count: int = 0

    @classmethod
    def create(cls, book_id: str, manifest_hash: str, file_count: int) -> "ManifestFile":
        """Create a new manifest with current timestamp."""
        return cls(
            book_id=book_id,
            manifest_hash=manifest_hash,
            timestamp=datetime.now(timezone.utc).isoformat(),
            file_count=file_count
        )


def load_manifest(path: Path) -> Optional[ManifestFile]:
    """Load manifest from file.

    Args:
        path: Path to manifest file (e.g., .panaversity/manifest.json)

    Returns:
        ManifestFile if file exists and is valid, None otherwise.
        Returns None for missing, empty, or corrupt files without raising.
    """
    if not path.exists():
        return None

    try:
        content = path.read_text(encoding="utf-8")
        if not content.strip():
            logger.warning(f"Manifest file is empty: {path}")
            # Delete empty manifest to prevent caching issues
            path.unlink(missing_ok=True)
            return None

        data = json.loads(content)
        return ManifestFile.model_validate(data)

    except json.JSONDecodeError as e:
        logger.warning(f"Corrupt manifest (invalid JSON): {path}: {e}")
        # Delete corrupt manifest to trigger full rebuild
        path.unlink(missing_ok=True)
        return None

    except (ValueError, KeyError) as e:
        logger.warning(f"Corrupt manifest (invalid structure): {path}: {e}")
        # Delete corrupt manifest to trigger full rebuild
        path.unlink(missing_ok=True)
        return None

    except UnicodeDecodeError as e:
        logger.warning(f"Corrupt manifest (encoding error): {path}: {e}")
        # Delete corrupt manifest to trigger full rebuild
        path.unlink(missing_ok=True)
        return None

    except Exception as e:
        logger.error(f"Unexpected error loading manifest: {path}: {e}")
        # Don't delete on unexpected errors - might be permission issue
        return None


def save_manifest(path: Path, manifest: ManifestFile) -> None:
    """Save manifest to file.

    Creates parent directories if they don't exist.

    Args:
        path: Path to manifest file
        manifest: ManifestFile to save
    """
    # Ensure parent directory exists
    path.parent.mkdir(parents=True, exist_ok=True)

    # Write manifest as formatted JSON
    content = manifest.model_dump_json(indent=2)
    path.write_text(content, encoding="utf-8")


def delete_manifest(path: Path) -> bool:
    """Delete manifest file if it exists.

    Args:
        path: Path to manifest file

    Returns:
        True if file was deleted, False if it didn't exist
    """
    if path.exists():
        path.unlink()
        return True
    return False


def get_default_manifest_path() -> Path:
    """Get the default manifest file path.

    Returns:
        Path to .panaversity/manifest.json in current directory
    """
    return Path(".panaversity/manifest.json")
