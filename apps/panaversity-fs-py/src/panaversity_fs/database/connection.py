"""Async database connection management for PanaversityFS.

Design for cloud databases (Neon, Supabase, PlanetScale):
- Uses NullPool to avoid keeping connections open
- Each request gets a fresh connection from the cloud pooler
- Connections are released immediately after use
- No server-side connection pooling (cloud handles this)

This pattern works well with:
- Serverless (Cloud Run, Lambda) - no warm connections needed
- Neon with PgBouncer - external pooling
- Supabase - built-in connection pooling
"""

from contextlib import asynccontextmanager
from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import (
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
    AsyncEngine,
)
from sqlalchemy.pool import NullPool

from ..config import get_config
from .models import Base


def _create_engine() -> AsyncEngine:
    """Create a new async engine configured for cloud databases.

    Uses NullPool so connections are not held open between requests.
    Cloud database providers (Neon, Supabase) handle connection pooling.

    Returns:
        AsyncEngine: SQLAlchemy async engine with NullPool.
    """
    config = get_config()
    return create_async_engine(
        config.effective_database_url,
        echo=config.log_level == "DEBUG",
        poolclass=NullPool,  # No local pooling - cloud handles it
    )


@asynccontextmanager
async def get_session() -> AsyncGenerator[AsyncSession, None]:
    """Get an async database session with automatic cleanup.

    Creates a fresh connection for each request and disposes it after.
    This pattern is ideal for:
    - Serverless environments (no persistent connections)
    - Cloud databases with external pooling (Neon, Supabase)
    - Stateless HTTP servers

    Yields:
        AsyncSession: Database session that commits on success, rolls back on error.

    Example:
        async with get_session() as session:
            result = await session.execute(select(FileJournal))
            # Auto-commits on exit, rolls back on exception
            # Connection is released immediately after
    """
    engine = _create_engine()
    factory = async_sessionmaker(
        engine,
        class_=AsyncSession,
        expire_on_commit=False,
    )

    try:
        async with factory() as session:
            try:
                yield session
                await session.commit()
            except Exception:
                await session.rollback()
                raise
    finally:
        # Dispose engine to release connection back to cloud pooler
        # Only dispose for NullPool (production) - StaticPool (tests) manages its own lifecycle
        pool_class = type(engine.pool).__name__
        if pool_class == "AsyncAdaptedNullPool":
            await engine.dispose()


async def init_db() -> None:
    """Initialize database tables (for development/testing).

    Creates all tables defined in models.py if they don't exist.
    For production, use Alembic migrations instead.
    """
    engine = _create_engine()
    try:
        async with engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
    finally:
        await engine.dispose()


# Legacy functions for test compatibility
_test_engine: AsyncEngine | None = None


def get_engine() -> AsyncEngine:
    """Get engine for testing. Creates new engine each call in production."""
    global _test_engine
    if _test_engine is None:
        _test_engine = _create_engine()
    return _test_engine


async def reset_engine() -> None:
    """Reset the test engine (for testing only).

    Call this between tests to ensure clean state.
    """
    global _test_engine
    if _test_engine is not None:
        await _test_engine.dispose()
        _test_engine = None
