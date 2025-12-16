"""Unit tests for FileJournal database model (T009)."""

import pytest
import asyncio
from datetime import datetime

from sqlalchemy import select
from sqlalchemy.exc import IntegrityError

from panaversity_fs.database import FileJournal, AuditLog, get_session, init_db, reset_engine


@pytest.fixture(scope="function")
async def db_session():
    """Create a fresh database session for each test.

    Uses in-memory SQLite to ensure isolation between tests.
    Handles rollback for tests that expect integrity errors.
    """
    import os
    from sqlalchemy.ext.asyncio import create_async_engine, async_sessionmaker, AsyncSession
    from sqlalchemy.pool import StaticPool
    from panaversity_fs.database.models import Base
    from panaversity_fs.database import connection

    # Use shared in-memory SQLite for this test
    test_engine = create_async_engine(
        "sqlite+aiosqlite:///:memory:",
        echo=False,
        poolclass=StaticPool,
        connect_args={"check_same_thread": False}
    )

    # Create all tables
    async with test_engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

    # Create session factory
    test_factory = async_sessionmaker(
        test_engine,
        class_=AsyncSession,
        expire_on_commit=False
    )

    # Monkey-patch _create_engine to return our test engine
    original_create_engine = connection._create_engine
    connection._create_engine = lambda: test_engine

    async with test_factory() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            # Don't re-raise - test already handled the expected exception

    # Cleanup
    connection._create_engine = original_create_engine
    await test_engine.dispose()


@pytest.mark.asyncio
class TestFileJournalCRUD:
    """Basic CRUD operations for FileJournal."""

    async def test_create_journal_entry(self, db_session):
        """Create a new journal entry."""
        entry = FileJournal(
            book_id="test-book",
            path="content/01-intro/01-basics/01-hello.md",
            user_id="__base__",
            sha256="abc123def456" * 4,  # 64 char hash
            storage_backend="s3",
        )
        db_session.add(entry)
        await db_session.flush()

        # Query back
        result = await db_session.execute(
            select(FileJournal).where(FileJournal.book_id == "test-book")
        )
        fetched = result.scalar_one()

        assert fetched.book_id == "test-book"
        assert fetched.path == "content/01-intro/01-basics/01-hello.md"
        assert fetched.user_id == "__base__"
        assert fetched.sha256 == "abc123def456" * 4
        assert fetched.storage_backend == "s3"
        assert isinstance(fetched.last_written_at, datetime)

    async def test_read_journal_entry_by_composite_key(self, db_session):
        """Read entry using composite primary key (book_id, path, user_id)."""
        entry = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",
            sha256="a" * 64,
        )
        db_session.add(entry)
        await db_session.flush()

        # Query by composite key
        result = await db_session.execute(
            select(FileJournal).where(
                FileJournal.book_id == "book1",
                FileJournal.path == "content/01-p/01-c/01-l.md",
                FileJournal.user_id == "__base__",
            )
        )
        fetched = result.scalar_one()
        assert fetched.sha256 == "a" * 64

    async def test_update_journal_entry(self, db_session):
        """Update an existing journal entry."""
        entry = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",
            sha256="old_hash" + "0" * 56,
        )
        db_session.add(entry)
        await db_session.flush()

        # Update hash
        entry.sha256 = "new_hash" + "1" * 56
        entry.last_written_at = datetime.utcnow()
        await db_session.flush()

        # Verify update
        result = await db_session.execute(
            select(FileJournal).where(FileJournal.book_id == "book1")
        )
        fetched = result.scalar_one()
        assert fetched.sha256 == "new_hash" + "1" * 56

    async def test_delete_journal_entry(self, db_session):
        """Delete a journal entry."""
        entry = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",
            sha256="a" * 64,
        )
        db_session.add(entry)
        await db_session.flush()

        # Delete
        await db_session.delete(entry)
        await db_session.flush()

        # Verify deletion
        result = await db_session.execute(
            select(FileJournal).where(FileJournal.book_id == "book1")
        )
        assert result.scalar_one_or_none() is None


@pytest.mark.asyncio
class TestFileJournalConstraints:
    """Test database constraints and indexes."""

    async def test_primary_key_uniqueness(self, db_session):
        """Duplicate primary key should fail."""
        entry1 = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",
            sha256="a" * 64,
        )
        entry2 = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",  # Same composite key
            sha256="b" * 64,
        )
        db_session.add(entry1)
        await db_session.flush()

        db_session.add(entry2)
        with pytest.raises(IntegrityError):
            await db_session.flush()

    async def test_overlay_entries_separate_from_base(self, db_session):
        """Same path can exist for base and overlay (different user_id)."""
        base_entry = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",
            sha256="base_hash" + "0" * 55,
        )
        overlay_entry = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="user123",  # Different user_id
            sha256="overlay_hash" + "1" * 52,
        )

        db_session.add(base_entry)
        db_session.add(overlay_entry)
        await db_session.flush()

        # Both should exist
        result = await db_session.execute(
            select(FileJournal).where(
                FileJournal.book_id == "book1",
                FileJournal.path == "content/01-p/01-c/01-l.md",
            )
        )
        entries = list(result.scalars())
        assert len(entries) == 2

    async def test_sha256_not_nullable(self, db_session):
        """sha256 field should not accept None."""
        entry = FileJournal(
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",
            sha256=None,  # Should fail
        )
        db_session.add(entry)
        with pytest.raises(IntegrityError):
            await db_session.flush()


@pytest.mark.asyncio
class TestAuditLogConstraints:
    """Test AuditLog model constraints (R6, R7)."""

    async def test_create_audit_log_entry(self, db_session):
        """Create a valid audit log entry."""
        entry = AuditLog(
            agent_id="claude-code",
            operation="create",
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
            user_id="__base__",
            prev_hash=None,
            new_hash="new" + "0" * 61,
            status="success",
        )
        db_session.add(entry)
        await db_session.flush()

        assert entry.id is not None
        assert entry.id > 0

    async def test_reject_system_agent_id(self, db_session):
        """Reject agent_id='system' (R7 invariant)."""
        entry = AuditLog(
            agent_id="system",  # Should fail
            operation="create",
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
        )
        db_session.add(entry)
        with pytest.raises(IntegrityError):
            await db_session.flush()

    async def test_reject_empty_agent_id(self, db_session):
        """Reject empty agent_id (R7 invariant)."""
        entry = AuditLog(
            agent_id="",  # Should fail
            operation="create",
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
        )
        db_session.add(entry)
        with pytest.raises(IntegrityError):
            await db_session.flush()

    async def test_valid_agent_ids_accepted(self, db_session):
        """Valid agent IDs should be accepted."""
        valid_ids = ["claude-code", "gpt-4", "agent-123", "my_agent"]

        for i, agent_id in enumerate(valid_ids):
            entry = AuditLog(
                agent_id=agent_id,
                operation="read",
                book_id="book1",
                path=f"content/01-p/01-c/0{i+1}-l.md",
            )
            db_session.add(entry)

        await db_session.flush()

        result = await db_session.execute(select(AuditLog))
        entries = list(result.scalars())
        assert len(entries) == len(valid_ids)

    async def test_audit_log_autoincrement(self, db_session):
        """Audit log IDs should auto-increment."""
        entry1 = AuditLog(
            agent_id="agent1",
            operation="create",
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
        )
        entry2 = AuditLog(
            agent_id="agent2",
            operation="update",
            book_id="book1",
            path="content/01-p/01-c/01-l.md",
        )

        db_session.add(entry1)
        await db_session.flush()
        db_session.add(entry2)
        await db_session.flush()

        assert entry2.id > entry1.id
