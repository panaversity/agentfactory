"""Custom exceptions for PanaversityFS.

Provides specific error types for different failure scenarios with actionable messages.
"""


class PanaversityFSError(Exception):
    """Base exception for all PanaversityFS errors."""
    pass


class ConfigurationError(PanaversityFSError):
    """Configuration validation failed."""
    pass


class StorageError(PanaversityFSError):
    """Storage backend operation failed."""
    pass


class ContentNotFoundError(PanaversityFSError):
    """Requested content file does not exist."""

    def __init__(self, path: str, suggested_paths: list[str] | None = None):
        self.path = path
        self.suggested_paths = suggested_paths or []

        message = f"Content not found: {path}"
        if suggested_paths:
            message += f"\n\nSuggested valid paths:\n" + "\n".join(f"  - {p}" for p in suggested_paths[:5])

        super().__init__(message)


class ConflictError(PanaversityFSError):
    """Concurrent modification detected via file hash mismatch (FR-008)."""

    def __init__(self, path: str, expected_hash: str, actual_hash: str):
        self.path = path
        self.expected_hash = expected_hash
        self.actual_hash = actual_hash

        message = (
            f"Conflict detected for {path}\n"
            f"Expected hash: {expected_hash}\n"
            f"Actual hash: {actual_hash}\n\n"
            f"Suggestion: Read current content, merge changes, and retry write with new hash"
        )
        super().__init__(message)


class AssetTooLargeError(PanaversityFSError):
    """Asset exceeds maximum size limit (FR-010: 100MB)."""

    def __init__(self, size_mb: float, max_size_mb: int = 100):
        self.size_mb = size_mb
        self.max_size_mb = max_size_mb

        message = (
            f"Asset too large: {size_mb:.2f}MB (max: {max_size_mb}MB)\n\n"
            f"Suggestion: For assets ≥10MB, use presigned URL upload pattern"
        )
        super().__init__(message)


class InvalidPathError(PanaversityFSError):
    """Path validation failed (security or format issue)."""

    def __init__(self, path: str, reason: str):
        self.path = path
        self.reason = reason

        message = f"Invalid path: {path}\nReason: {reason}"
        super().__init__(message)


class AuthenticationError(PanaversityFSError):
    """API key authentication failed."""

    def __init__(self, message: str = "Invalid or missing API key"):
        super().__init__(message)


class BookNotFoundError(PanaversityFSError):
    """Book not found in registry."""

    def __init__(self, book_id: str, available_books: list[str] | None = None):
        self.book_id = book_id
        self.available_books = available_books or []

        message = f"Book not found: {book_id}"
        if available_books:
            message += f"\n\nAvailable books:\n" + "\n".join(f"  - {b}" for b in available_books[:10])

        super().__init__(message)


class AssetNotFoundError(PanaversityFSError):
    """Asset not found in storage."""

    def __init__(self, book_id: str, asset_type: str, filename: str, available_assets: list[str] | None = None):
        self.book_id = book_id
        self.asset_type = asset_type
        self.filename = filename
        self.available_assets = available_assets or []

        message = f"Asset not found: {book_id}/assets/{asset_type}/{filename}"
        if available_assets:
            message += f"\n\nAvailable {asset_type} assets:\n" + "\n".join(f"  - {a}" for a in available_assets[:5])

        super().__init__(message)


class SummaryNotFoundError(PanaversityFSError):
    """Summary file not found."""

    def __init__(self, path: str):
        self.path = path

        message = (
            f"Summary not found at {path}\n\n"
            f"Suggestion: Use add_summary tool to create it first"
        )
        super().__init__(message)


class ArchiveGenerationError(PanaversityFSError):
    """Archive generation failed or timed out (FR-031)."""

    def __init__(self, book_id: str, reason: str):
        self.book_id = book_id
        self.reason = reason

        message = (
            f"Archive generation failed for book: {book_id}\n"
            f"Reason: {reason}\n\n"
            f"Suggestion: Use individual file download tools (read_content, get_asset)"
        )
        super().__init__(message)
