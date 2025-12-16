"""SQLAlchemy models for PanaversityFS metadata journal and audit log."""

from datetime import datetime
from typing import Optional

from sqlalchemy import String, Text, Index, CheckConstraint, Integer, TIMESTAMP
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column


class Base(DeclarativeBase):
    """Base class for all SQLAlchemy models."""
    pass


class FileJournal(Base):
    """Tracks current state of every file for conflict detection and delta builds.

    Primary key: (book_id, path, user_id)

    Invariant R2: sha256 matches actual storage content

    Attributes:
        book_id: Identifier for the book
        path: Relative path within the book (e.g., content/01-intro/01-lesson/01-hello.md)
        user_id: User ID for overlays, "__base__" for base content
        sha256: SHA-256 hash of file content for conflict detection
        last_written_at: Timestamp of last write
        storage_backend: Which storage backend holds the file (s3, fs, supabase)
    """
    __tablename__ = "file_journal"

    book_id: Mapped[str] = mapped_column(String(255), primary_key=True)
    path: Mapped[str] = mapped_column(String(1024), primary_key=True)
    user_id: Mapped[str] = mapped_column(String(255), primary_key=True, default="__base__")
    sha256: Mapped[str] = mapped_column(String(64), nullable=False)
    last_written_at: Mapped[datetime] = mapped_column(TIMESTAMP(timezone=True), default=datetime.utcnow)
    storage_backend: Mapped[str] = mapped_column(String(50), default="s3")

    __table_args__ = (
        Index("ix_file_journal_book_path", "book_id", "path"),
    )


class ManifestSnapshot(Base):
    """Stores historical manifest snapshots for delta build computation (FR-025/026).

    Each snapshot captures the complete state of a book's base content at a point
    in time, enabling plan_build to compute true deltas by comparing current state
    to any previous manifest.

    Attributes:
        manifest_hash: SHA-256 hash of the manifest (primary key)
        book_id: Identifier for the book
        created_at: When this manifest was captured
        file_count: Number of files in this manifest
        content_json: JSON map of {path: sha256} for all files in manifest
    """
    __tablename__ = "manifest_snapshot"

    manifest_hash: Mapped[str] = mapped_column(String(64), primary_key=True)
    book_id: Mapped[str] = mapped_column(String(255), nullable=False, index=True)
    created_at: Mapped[datetime] = mapped_column(TIMESTAMP(timezone=True), default=datetime.utcnow)
    file_count: Mapped[int] = mapped_column(Integer, default=0)
    content_json: Mapped[str] = mapped_column(Text, nullable=False)  # JSON: {path: sha256}

    __table_args__ = (
        Index("ix_manifest_book_created", "book_id", "created_at"),
    )


class AuditLog(Base):
    """Append-only audit trail with hash chain integrity.

    Invariant R6: entry[n].new_hash == entry[n+1].prev_hash
    Invariant R7: agent_id != 'system' and agent_id != ''

    The hash chain links consecutive operations on the same (book_id, path, user_id)
    tuple, enabling tamper detection and complete provenance tracking.

    Attributes:
        id: Auto-incrementing primary key
        timestamp: When the operation occurred
        agent_id: ID of the agent that performed the operation (never 'system' or empty)
        operation: Type of operation (create, update, delete, read)
        book_id: Identifier for the book
        path: Relative path within the book
        user_id: User ID for overlays, "__base__" for base content
        prev_hash: SHA-256 hash from previous entry on same (book_id, path, user_id)
        new_hash: SHA-256 hash of content after this operation (null for delete)
        status: Operation status (success, error)
        error_message: Error details if status is 'error'
        execution_time_ms: How long the operation took in milliseconds
    """
    __tablename__ = "audit_log"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, autoincrement=True)
    timestamp: Mapped[datetime] = mapped_column(TIMESTAMP(timezone=True), default=datetime.utcnow, index=True)
    agent_id: Mapped[str] = mapped_column(String(255), nullable=False, index=True)
    operation: Mapped[str] = mapped_column(String(50), nullable=False)
    book_id: Mapped[str] = mapped_column(String(255), nullable=False, index=True)
    path: Mapped[str] = mapped_column(String(1024), nullable=False)
    user_id: Mapped[str] = mapped_column(String(255), default="__base__")
    prev_hash: Mapped[Optional[str]] = mapped_column(String(64), nullable=True)
    new_hash: Mapped[Optional[str]] = mapped_column(String(64), nullable=True)
    status: Mapped[str] = mapped_column(String(50), default="success")
    error_message: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    execution_time_ms: Mapped[Optional[int]] = mapped_column(Integer, nullable=True)

    __table_args__ = (
        CheckConstraint("agent_id != 'system'", name="ck_agent_not_system"),
        CheckConstraint("agent_id != ''", name="ck_agent_not_empty"),
        Index("ix_audit_book_path_user", "book_id", "path", "user_id"),
    )
