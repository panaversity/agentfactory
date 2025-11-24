"""Pydantic domain models for PanaversityFS.

Defines all entity models used throughout the system:
- Content operations (lessons, markdown files)
- Asset management (images, slides, videos, audio)
- Summary management (chapter summaries)
- Registry (book catalog)
- Audit trail (operation logs)
"""

from pydantic import BaseModel, Field, field_validator, ConfigDict
from typing import Literal
from datetime import datetime
from enum import Enum


# ============================================================================
# Enums
# ============================================================================

class AssetType(str, Enum):
    """Asset type categories (FR-011)."""
    SLIDES = "slides"
    IMAGES = "images"
    VIDEOS = "videos"
    AUDIO = "audio"


class OperationType(str, Enum):
    """Audit log operation types."""
    READ_CONTENT = "read_content"
    WRITE_CONTENT = "write_content"
    DELETE_CONTENT = "delete_content"
    UPLOAD_ASSET = "upload_asset"
    GET_ASSET = "get_asset"
    LIST_ASSETS = "list_assets"
    ADD_SUMMARY = "add_summary"
    UPDATE_SUMMARY = "update_summary"
    GET_SUMMARY = "get_summary"
    LIST_SUMMARIES = "list_summaries"
    GET_BOOK_ARCHIVE = "get_book_archive"
    LIST_BOOKS = "list_books"
    GLOB_SEARCH = "glob_search"
    GREP_SEARCH = "grep_search"


class OperationStatus(str, Enum):
    """Audit log operation status."""
    SUCCESS = "success"
    ERROR = "error"
    CONFLICT = "conflict"


# ============================================================================
# Content Models
# ============================================================================

class ContentMetadata(BaseModel):
    """Metadata for lesson content files.

    Returned by read_content tool (FR-009).
    """
    model_config = ConfigDict(str_strip_whitespace=True)

    file_size: int = Field(..., description="File size in bytes", ge=0)
    last_modified: datetime = Field(..., description="Last modification timestamp")
    storage_backend: Literal["fs", "s3", "supabase"] = Field(..., description="Storage backend used")
    file_hash_sha256: str = Field(..., description="SHA256 hash for conflict detection", min_length=64, max_length=64)
    content: str = Field(..., description="Markdown content with YAML frontmatter")


# ============================================================================
# Asset Models
# ============================================================================

class AssetMetadata(BaseModel):
    """Metadata for binary assets (FR-012).

    Returned by get_asset and list_assets tools.
    """
    model_config = ConfigDict(str_strip_whitespace=True)

    cdn_url: str = Field(..., description="Public CDN URL for asset access")
    file_size: int = Field(..., description="File size in bytes", ge=0, le=100*1024*1024)  # Max 100MB
    mime_type: str = Field(..., description="MIME type (e.g., 'image/png', 'application/pdf')")
    upload_timestamp: datetime = Field(..., description="When asset was uploaded")
    uploaded_by_agent_id: str = Field(..., description="Agent ID that uploaded the asset")
    asset_type: AssetType = Field(..., description="Asset category")
    filename: str = Field(..., description="Original filename")


# ============================================================================
# Summary Models
# ============================================================================

class SummaryMetadata(BaseModel):
    """Metadata for chapter summary files (FR-016).

    Returned by get_summary and list_summaries tools.
    """
    model_config = ConfigDict(str_strip_whitespace=True)

    file_size: int = Field(..., description="File size in bytes", ge=0)
    last_modified: datetime = Field(..., description="Last modification timestamp")
    storage_backend: Literal["fs", "s3", "supabase"] = Field(..., description="Storage backend used")
    file_hash_sha256: str = Field(..., description="SHA256 hash", min_length=64, max_length=64)
    path: str = Field(..., description="Full path to summary file")


# ============================================================================
# Registry Models
# ============================================================================

class BookStatus(str, Enum):
    """Book registry status (FR-024)."""
    ACTIVE = "active"
    ARCHIVED = "archived"
    MIGRATING = "migrating"


class BookMetadata(BaseModel):
    """Book registry entry (FR-024).

    Stored in registry.yaml and returned by list_books tool.
    """
    model_config = ConfigDict(str_strip_whitespace=True)

    book_id: str = Field(..., description="Unique book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    title: str = Field(..., description="Human-readable book title", min_length=1, max_length=200)
    storage_backend: Literal["fs", "s3", "supabase"] = Field(..., description="Storage backend for this book")
    created_at: datetime = Field(..., description="When book was added to registry")
    status: BookStatus = Field(default=BookStatus.ACTIVE, description="Book status")

    @field_validator('book_id')
    @classmethod
    def validate_book_id(cls, v: str) -> str:
        """Validate book_id is lowercase with hyphens only."""
        if not v.islower():
            raise ValueError("book_id must be lowercase")
        return v


# ============================================================================
# Audit Trail Models
# ============================================================================

class AuditEntry(BaseModel):
    """JSONL audit log entry (FR-018).

    Written to .audit/YYYY-MM-DD.jsonl for all operations.
    """
    model_config = ConfigDict(str_strip_whitespace=True)

    timestamp: datetime = Field(..., description="Operation timestamp (ISO 8601)")
    agent_id: str = Field(..., description="Agent/user ID performing operation")
    operation: OperationType = Field(..., description="Operation type")
    path: str = Field(..., description="File path affected by operation")
    status: OperationStatus = Field(..., description="Operation result status")
    error_message: str | None = Field(default=None, description="Error details if status=error")
    execution_time_ms: int | None = Field(default=None, description="Operation execution time", ge=0)


# ============================================================================
# Tool Input Models (for MCP tool parameters)
# ============================================================================

class ReadContentInput(BaseModel):
    """Input model for read_content tool."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    path: str = Field(..., description="Lesson path relative to book root (e.g., 'lessons/part-1/chapter-01/lesson-01.md')", min_length=1, max_length=255)


class WriteContentInput(BaseModel):
    """Input model for write_content tool (upsert semantics).

    Supports both create and update operations:
    - If file_hash provided: Update with conflict detection
    - If file_hash omitted: Create or overwrite
    """
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    path: str = Field(..., description="Lesson path relative to book root", min_length=1, max_length=255)
    content: str = Field(..., description="Markdown content with YAML frontmatter", min_length=1, max_length=500_000)
    file_hash: str | None = Field(default=None, description="SHA256 hash for conflict detection (if updating)", min_length=64, max_length=64)


class DeleteContentInput(BaseModel):
    """Input model for delete_content tool."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    path: str = Field(..., description="Lesson path to delete", min_length=1, max_length=255)


class UploadAssetInput(BaseModel):
    """Input model for upload_asset tool (hybrid pattern).

    Supports two upload methods:
    - Direct upload (binary_data provided, <10MB)
    - Presigned URL (file_size provided, ≥10MB)
    """
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    asset_type: AssetType = Field(..., description="Asset category (slides/images/videos/audio)")
    filename: str = Field(..., description="Asset filename with extension", min_length=1, max_length=255)
    binary_data: str | None = Field(default=None, description="Base64-encoded binary data for direct upload (<10MB)")
    file_size: int | None = Field(default=None, description="File size in bytes for presigned URL request (≥10MB)", ge=10*1024*1024, le=100*1024*1024)


class GetAssetInput(BaseModel):
    """Input model for get_asset tool."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    asset_type: AssetType = Field(..., description="Asset category")
    filename: str = Field(..., description="Asset filename", min_length=1, max_length=255)


class ListAssetsInput(BaseModel):
    """Input model for list_assets tool."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    asset_type: AssetType | None = Field(default=None, description="Filter by asset type (optional)")


# ============================================================================
# Summary Input Models
# ============================================================================

class AddSummaryInput(BaseModel):
    """Input model for add_summary tool (FR-014)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    chapter_id: str = Field(..., description="Chapter identifier (e.g., 'chapter-01')", pattern=r'^chapter-\d{2}$')
    content: str = Field(..., description="Summary markdown content", min_length=1, max_length=100_000)


class UpdateSummaryInput(BaseModel):
    """Input model for update_summary tool (FR-015)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    chapter_id: str = Field(..., description="Chapter identifier (e.g., 'chapter-01')", pattern=r'^chapter-\d{2}$')
    content: str = Field(..., description="Summary markdown content", min_length=1, max_length=100_000)


class GetSummaryInput(BaseModel):
    """Input model for get_summary tool (FR-016)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    chapter_id: str = Field(..., description="Chapter identifier (e.g., 'chapter-01')", pattern=r'^chapter-\d{2}$')


class ListSummariesInput(BaseModel):
    """Input model for list_summaries tool (FR-017)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    chapter_id: str | None = Field(default=None, description="Filter by chapter ID (optional)", pattern=r'^chapter-\d{2}$')
