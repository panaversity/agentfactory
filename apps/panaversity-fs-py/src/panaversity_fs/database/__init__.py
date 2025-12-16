"""Database layer for PanaversityFS metadata journal and audit log.

This module provides:
- SQLAlchemy models (FileJournal, AuditLog) for metadata persistence
- Async session management with PostgreSQL (asyncpg) and SQLite (aiosqlite)
- Database initialization utilities

Usage:
    from panaversity_fs.database import get_session, FileJournal, AuditLog

    async with get_session() as session:
        entry = FileJournal(book_id="my-book", path="content/01/01/01.md", sha256="...")
        session.add(entry)
        # Auto-commits on exit
"""

from .models import FileJournal, AuditLog, Base
from .connection import get_session, get_engine, init_db, reset_engine

__all__ = [
    "FileJournal",
    "AuditLog",
    "Base",
    "get_session",
    "get_engine",
    "init_db",
    "reset_engine",
]
