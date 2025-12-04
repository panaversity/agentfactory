"""Pydantic domain models for PanaversityFS.

Defines all entity models used throughout the system:
- Content operations (lessons and summaries - ADR-0018)
- Asset management (images, slides, videos, audio)
- Registry (book catalog)
- Audit trail (operation logs)

Note: Summary files use content tools with .summary.md naming convention (ADR-0018).
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


class ContentScope(str, Enum):
    """Scope for read_content operations (Option B enhancement).

    - FILE: Read single file (default, original behavior)
    - CHAPTER: Read all .md files in the chapter directory
    - PART: Read all .md files in the part directory (all chapters)
    - BOOK: Read all .md files in the entire book's content/ directory
    """
    FILE = "file"
    CHAPTER = "chapter"
    PART = "part"
    BOOK = "book"


class OperationType(str, Enum):
    """Audit log operation types (9 tools per ADR-0018)."""
    READ_CONTENT = "read_content"
    WRITE_CONTENT = "write_content"
    DELETE_CONTENT = "delete_content"
    UPLOAD_ASSET = "upload_asset"
    GET_ASSET = "get_asset"
    LIST_ASSETS = "list_assets"
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
    When include_binary=true on get_asset, binary_data field is populated.
    """
    model_config = ConfigDict(str_strip_whitespace=True)

    cdn_url: str = Field(..., description="Public CDN URL for asset access")
    file_size: int = Field(..., description="File size in bytes", ge=0, le=100*1024*1024)  # Max 100MB
    mime_type: str = Field(..., description="MIME type (e.g., 'image/png', 'application/pdf')")
    upload_timestamp: datetime = Field(..., description="When asset was uploaded")
    uploaded_by_agent_id: str = Field(..., description="Agent ID that uploaded the asset")
    asset_type: AssetType = Field(..., description="Asset category")
    filename: str = Field(..., description="Original filename")
    binary_data: str | None = Field(default=None, description="Base64-encoded binary data (only when include_binary=true)")


# ============================================================================
# Registry Models
# ============================================================================

class BookEntry(BaseModel):
    """Book entry returned by list_books tool (FR-024).

    Dynamically discovered from books/ directory structure.
    """
    model_config = ConfigDict(str_strip_whitespace=True)

    book_id: str = Field(..., description="Unique book identifier (directory name)", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    storage_backend: Literal["fs", "s3", "supabase"] = Field(..., description="Storage backend serving this book")


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
    """Input model for read_content tool.

    Supports single file or bulk reads via scope parameter:
    - scope=file (default): Read single file at path
    - scope=chapter: Read all .md files in the chapter (path should be chapter directory)
    - scope=part: Read all .md files in the part (path should be part directory)
    - scope=book: Read all .md files in the entire book's content/ directory
    """
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    path: str = Field(default="content", description="Content path relative to book root (e.g., 'content/01-Part/01-Chapter/01-lesson.md' for file, 'content' for book scope)", min_length=1, max_length=255)
    scope: ContentScope = Field(default=ContentScope.FILE, description="Read scope: 'file' (single file), 'chapter' (all .md in chapter), 'part' (all .md in part), 'book' (all .md in book)")


class WriteContentInput(BaseModel):
    """Input model for write_content tool (upsert semantics).

    Supports both create and update operations (FR-003, FR-004, FR-005):
    - If expected_hash provided: Update with conflict detection (FR-003)
    - If expected_hash omitted AND file exists: REJECTED with HASH_REQUIRED (FR-004)
    - If expected_hash omitted AND file doesn't exist: Create operation (FR-005)
    """
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    path: str = Field(..., description="Lesson path relative to book root", min_length=1, max_length=255)
    content: str = Field(..., description="Markdown content with YAML frontmatter", min_length=1, max_length=500_000)
    expected_hash: str | None = Field(default=None, description="SHA256 hash for conflict detection. REQUIRED when updating existing files (FR-004)", min_length=64, max_length=64)


class DeleteContentInput(BaseModel):
    """Input model for delete_content tool."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    path: str = Field(..., description="Lesson path to delete", min_length=1, max_length=255)


class UploadAssetInput(BaseModel):
    """Input model for upload_asset tool (hybrid pattern).

    Supports two upload methods based on PANAVERSITY_MAX_DIRECT_UPLOAD_MB config (default 10MB):
    - Direct upload (binary_data provided, below threshold)
    - Presigned URL (file_size provided, at or above threshold)
    """
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    asset_type: AssetType = Field(..., description="Asset category (slides/images/videos/audio)")
    filename: str = Field(..., description="Asset filename with extension", min_length=1, max_length=255)
    binary_data: str | None = Field(default=None, description="Base64-encoded binary data for direct upload (below config threshold)")
    file_size: int | None = Field(default=None, description="File size in bytes for presigned URL request (at or above config threshold)", ge=10*1024*1024, le=100*1024*1024)


class GetAssetInput(BaseModel):
    """Input model for get_asset tool.

    By default returns metadata + CDN URL. Use include_binary=true to also
    return base64-encoded binary data (for Docusaurus plugin or direct download).
    """
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    asset_type: AssetType = Field(..., description="Asset category")
    filename: str = Field(..., description="Asset filename", min_length=1, max_length=255)
    include_binary: bool = Field(default=False, description="Include base64-encoded binary data in response (default: false, metadata only)")


class ListAssetsInput(BaseModel):
    """Input model for list_assets tool."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    asset_type: AssetType | None = Field(default=None, description="Filter by asset type (optional)")


# ============================================================================
# Registry Input Models
# ============================================================================

class ListBooksInput(BaseModel):
    """Input model for list_books tool (FR-024)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    # Optional: include hierarchical structure of each book
    include_structure: Literal["none", "content", "assets", "all"] | None = Field(
        default=None,
        description="Include book structure: 'none' (default), 'content' (parts/chapters/lessons), 'assets' (images/slides), 'all' (both)"
    )
    # Optional: filter to specific book
    book_id: str | None = Field(
        default=None,
        description="Filter to specific book (optional)",
        pattern=r'^[a-z0-9-]+$',
        min_length=3,
        max_length=50
    )


# ============================================================================
# Search Input Models
# ============================================================================

class GlobSearchInput(BaseModel):
    """Input model for glob_search tool (FR-026)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    pattern: str = Field(..., description="Glob pattern (e.g., '**/*.md', 'assets/images/**/*.png')", min_length=1, max_length=255)
    all_books: bool = Field(default=False, description="Search across all books (default: single book)")


class GrepSearchInput(BaseModel):
    """Input model for grep_search tool (FR-027)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    pattern: str = Field(..., description="Regex pattern to search for", min_length=1, max_length=500)
    all_books: bool = Field(default=False, description="Search across all books (default: single book)")
    max_results: int = Field(default=100, description="Maximum results to return", ge=1, le=1000)


# ============================================================================
# Bulk Operations Input Models
# ============================================================================

class ArchiveScope(str, Enum):
    """Scope for get_book_archive operations.

    - ALL: Archive entire book (content + assets) - may timeout for large books
    - CONTENT: Archive only content/ directory (markdown files) - faster
    - ASSETS: Archive only static/ directory (images, slides, etc.)
    """
    ALL = "all"
    CONTENT = "content"
    ASSETS = "assets"


class GetBookArchiveInput(BaseModel):
    """Input model for get_book_archive tool (FR-029)."""
    model_config = ConfigDict(str_strip_whitespace=True, extra='forbid')

    book_id: str = Field(..., description="Book identifier", pattern=r'^[a-z0-9-]+$', min_length=3, max_length=50)
    scope: ArchiveScope = Field(
        default=ArchiveScope.CONTENT,
        description="Archive scope: 'content' (markdown only, default), 'assets' (images/slides), 'all' (entire book - may timeout)"
    )
