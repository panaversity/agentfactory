"""PostgreSQL-based Store implementation for ChatKit."""

import asyncio
import logging
from datetime import datetime
from typing import Any

from pydantic import BaseModel
from sqlalchemy import text
from sqlalchemy.ext.asyncio import (
    AsyncEngine,
    AsyncSession,
    async_sessionmaker,
    create_async_engine,
)

from chatkit.store import NotFoundError, Store
from chatkit.types import Attachment, Page, ThreadItem, ThreadMetadata

from .config import StoreConfig
from .context import RequestContext

logger = logging.getLogger(__name__)


class ThreadData(BaseModel):
    """Wrapper for thread serialization."""

    thread: ThreadMetadata


class ItemData(BaseModel):
    """Wrapper for item serialization."""

    item: ThreadItem


class AttachmentData(BaseModel):
    """Wrapper for attachment serialization."""

    attachment: Attachment


class PostgresStore(Store[RequestContext]):
    """
    Production-ready PostgreSQL store for ChatKit.

    Features:
    - Async connection pooling
    - User-based multi-tenancy
    - JSON serialization for flexible schema evolution
    - Proper indexing for performance
    - Statement timeouts
    - Error handling and logging
    """

    def __init__(
        self,
        config: StoreConfig | None = None,
        engine: AsyncEngine | None = None,
    ):
        """
        Initialize the PostgreSQL store.

        Args:
            config: Store configuration. If None, loads from environment.
            engine: Optional pre-configured SQLAlchemy engine.
                    If provided, config is ignored.
        """
        if engine:
            self.engine = engine
            self.config = None
        elif config:
            self.config = config
            self.engine = self._create_engine(config)
        else:
            self.config = StoreConfig()
            self.engine = self._create_engine(self.config)

        self.session_factory = async_sessionmaker(
            self.engine,
            class_=AsyncSession,
            expire_on_commit=False,
        )

        logger.info("PostgresStore initialized")

    def _create_engine(self, config: StoreConfig) -> AsyncEngine:
        """Create SQLAlchemy async engine with connection pooling."""
        return create_async_engine(
            config.database_url,
            pool_size=config.pool_size,
            max_overflow=config.max_overflow,
            pool_timeout=config.pool_timeout,
            pool_recycle=config.pool_recycle,
            pool_pre_ping=True,  # CRITICAL: Verify connections before use to prevent "connection closed" errors
            echo=False,  # Set to True for SQL debugging
            future=True,
            # Performance and reliability optimizations
            connect_args={
                "command_timeout": 30,  # 30 second command timeout for safety
                "server_settings": {
                    "application_name": "chatkit_store",
                    "jit": "off",  # Disable JIT for faster queries
                    "statement_timeout": str(
                        config.statement_timeout
                    ),  # Apply configured timeout
                },
            },
        )

    async def initialize_schema(self) -> None:
        """
        Create database schema and tables if they don't exist.

        Should be called during application startup.
        Uses caching to avoid re-initialization.
        """
        schema = self.config.schema_name if self.config else "chatkit"

        # Check if schema already exists to avoid unnecessary work
        async with self.engine.connect() as conn:
            result = await conn.execute(
                text("""
                SELECT EXISTS (
                    SELECT FROM information_schema.schemata
                    WHERE schema_name = :schema_name
                )
            """),
                {"schema_name": schema},
            )

            schema_exists = result.scalar()
            if schema_exists:
                logger.info(
                    f"Schema '{schema}' already exists, skipping initialization"
                )
                return

        logger.info(f"Initializing schema '{schema}'...")

        async with self.engine.begin() as conn:
            # Create schema
            await conn.execute(text(f"CREATE SCHEMA IF NOT EXISTS {schema}"))

            # Create threads table
            await conn.execute(
                text(f"""
                CREATE TABLE IF NOT EXISTS {schema}.threads (
                    id TEXT PRIMARY KEY,
                    user_id TEXT NOT NULL,
                    organization_id TEXT,
                    created_at TIMESTAMPTZ NOT NULL,
                    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                    data JSONB NOT NULL,
                    CONSTRAINT threads_id_user_unique UNIQUE (id, user_id)
                )
            """)
            )

            # Create items table
            await conn.execute(
                text(f"""
                CREATE TABLE IF NOT EXISTS {schema}.items (
                    id TEXT PRIMARY KEY,
                    thread_id TEXT NOT NULL,
                    user_id TEXT NOT NULL,
                    organization_id TEXT,
                    created_at TIMESTAMPTZ NOT NULL,
                    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                    data JSONB NOT NULL,
                    CONSTRAINT items_id_user_unique UNIQUE (id, user_id)
                )
            """)
            )

            # Create attachments table
            await conn.execute(
                text(f"""
                CREATE TABLE IF NOT EXISTS {schema}.attachments (
                    id TEXT PRIMARY KEY,
                    user_id TEXT NOT NULL,
                    organization_id TEXT,
                    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                    data JSONB NOT NULL,
                    CONSTRAINT attachments_id_user_unique UNIQUE (id, user_id)
                )
            """)
            )

            # Create indexes for performance
            await conn.execute(
                text(f"""
                CREATE INDEX IF NOT EXISTS idx_threads_user_created
                ON {schema}.threads (user_id, created_at DESC)
            """)
            )

            await conn.execute(
                text(f"""
                CREATE INDEX IF NOT EXISTS idx_threads_org_created
                ON {schema}.threads (organization_id, created_at DESC)
                WHERE organization_id IS NOT NULL
            """)
            )

            await conn.execute(
                text(f"""
                CREATE INDEX IF NOT EXISTS idx_items_thread_created
                ON {schema}.items (thread_id, created_at)
            """)
            )

            await conn.execute(
                text(f"""
                CREATE INDEX IF NOT EXISTS idx_items_user_thread
                ON {schema}.items (user_id, thread_id)
            """)
            )

            await conn.execute(
                text(f"""
                CREATE INDEX IF NOT EXISTS idx_attachments_user
                ON {schema}.attachments (user_id, created_at DESC)
            """)
            )

            await conn.commit()

        logger.info(f"Database schema '{schema}' initialized successfully")

        # Warm up the connection pool
        await self._warm_connection_pool()

    async def _warm_connection_pool(self) -> None:
        """Warm up the connection pool to reduce cold start latency."""
        try:
            # Create a few connections to warm up the pool
            warmup_tasks = []
            for _ in range(min(3, self.config.pool_size if self.config else 5)):
                warmup_tasks.append(self._warm_single_connection())

            await asyncio.gather(*warmup_tasks, return_exceptions=True)
            logger.info("Connection pool warmed up")
        except Exception as e:
            logger.warning(f"Failed to warm connection pool: {e}")

    async def _warm_single_connection(self) -> None:
        """Warm up a single connection."""
        async with self.engine.connect() as conn:
            await conn.execute(text("SELECT 1"))

    def _get_table_name(self, table: str) -> str:
        """Get fully qualified table name with schema."""
        schema = self.config.schema_name if self.config else "chatkit"
        return f"{schema}.{table}"

    async def load_thread(
        self, thread_id: str, context: RequestContext
    ) -> ThreadMetadata:
        """Load a thread by ID with user isolation."""
        async with self.session_factory() as session:
            result = await session.execute(
                text(f"""
                    SELECT data FROM {self._get_table_name("threads")}
                    WHERE id = :thread_id AND user_id = :user_id
                """),
                {"thread_id": thread_id, "user_id": context.user_id},
            )
            row = result.first()

            if not row:
                raise NotFoundError(f"Thread {thread_id} not found")

            # PostgreSQL JSONB is automatically parsed to dict by asyncpg
            # row[0] is already the full ThreadData dict: {"thread": {...}}
            return ThreadData.model_validate(row[0]).thread

    async def save_thread(
        self, thread: ThreadMetadata, context: RequestContext
    ) -> None:
        """Save or update a thread."""
        thread_data = ThreadData(thread=thread)

        async with self.session_factory() as session:
            try:
                # Upsert pattern for PostgreSQL
                await session.execute(
                    text(f"""
                        INSERT INTO {self._get_table_name("threads")}
                        (id, user_id, organization_id, created_at, updated_at, data)
                        VALUES (:id, :user_id, :org_id, :created_at, NOW(), :data)
                        ON CONFLICT (id, user_id) DO UPDATE SET
                            data = EXCLUDED.data,
                            updated_at = NOW()
                    """),
                    {
                        "id": thread.id,
                        "user_id": context.user_id,
                        "org_id": context.organization_id,
                        "created_at": thread.created_at,
                        "data": thread_data.model_dump_json(),
                    },
                )
                await session.commit()
            except Exception as e:
                await session.rollback()
                logger.error(f"Failed to save thread {thread.id}: {e}")
                raise

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: RequestContext,
    ) -> Page[ThreadItem]:
        """Load paginated thread items."""
        async with self.session_factory() as session:
            created_after: datetime | None = None

            if after:
                # Get created_at timestamp of the 'after' item
                result = await session.execute(
                    text(f"""
                        SELECT created_at FROM {self._get_table_name("items")}
                        WHERE id = :after_id AND user_id = :user_id
                    """),
                    {"after_id": after, "user_id": context.user_id},
                )
                row = result.first()
                if not row:
                    raise NotFoundError(f"Item {after} not found")
                created_after = row[0]

            # Build query
            params: dict[str, Any] = {
                "thread_id": thread_id,
                "user_id": context.user_id,
                "limit": limit + 1,
            }

            if created_after:
                comparison = ">" if order == "asc" else "<"
                params["created_after"] = created_after
                created_clause = f"AND created_at {comparison} :created_after"
            else:
                created_clause = ""

            result = await session.execute(
                text(f"""
                    SELECT id, data FROM {self._get_table_name("items")}
                    WHERE thread_id = :thread_id AND user_id = :user_id
                    {created_clause}
                    ORDER BY created_at {order}
                    LIMIT :limit
                """),
                params,
            )

            items = [
                # PostgreSQL JSONB is automatically parsed to dict by asyncpg
                # row[1] is already the full ItemData dict: {"item": {...}}
                ItemData.model_validate(row[1]).item
                for row in result.fetchall()
            ]

            has_more = len(items) > limit
            if has_more:
                items = items[:limit]

            next_after = items[-1].id if (has_more and items) else None

            return Page[ThreadItem](
                data=items,
                has_more=has_more,
                after=next_after,
            )

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: RequestContext
    ) -> None:
        """Add a new item to a thread."""
        item_data = ItemData(item=item)

        async with self.session_factory() as session:
            await session.execute(
                text(f"""
                    INSERT INTO {self._get_table_name("items")}
                    (id, thread_id, user_id, organization_id, created_at, data)
                    VALUES (:id, :thread_id, :user_id, :org_id, :created_at, :data)
                """),
                {
                    "id": item.id,
                    "thread_id": thread_id,
                    "user_id": context.user_id,
                    "org_id": context.organization_id,
                    "created_at": item.created_at,
                    "data": item_data.model_dump_json(),
                },
            )
            await session.commit()

    async def save_item(
        self, thread_id: str, item: ThreadItem, context: RequestContext
    ) -> None:
        """Update an existing item."""
        item_data = ItemData(item=item)

        async with self.session_factory() as session:
            result = await session.execute(
                text(f"""
                    UPDATE {self._get_table_name("items")}
                    SET data = :data, updated_at = NOW()
                    WHERE id = :id AND thread_id = :thread_id AND user_id = :user_id
                """),
                {
                    "id": item.id,
                    "thread_id": thread_id,
                    "user_id": context.user_id,
                    "data": item_data.model_dump_json(),
                },
            )
            await session.commit()

            if result.rowcount == 0:
                raise NotFoundError(f"Item {item.id} not found in thread {thread_id}")

    async def load_item(
        self, thread_id: str, item_id: str, context: RequestContext
    ) -> ThreadItem:
        """Load a specific item by ID."""
        async with self.session_factory() as session:
            result = await session.execute(
                text(f"""
                    SELECT data FROM {self._get_table_name("items")}
                    WHERE id = :item_id
                    AND thread_id = :thread_id
                    AND user_id = :user_id
                """),
                {
                    "item_id": item_id,
                    "thread_id": thread_id,
                    "user_id": context.user_id,
                },
            )
            row = result.first()

            if not row:
                raise NotFoundError(f"Item {item_id} not found in thread {thread_id}")

            # PostgreSQL JSONB is automatically parsed to dict by asyncpg
            # row[0] is already the full ItemData dict: {"item": {...}}
            return ItemData.model_validate(row[0]).item

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: RequestContext
    ) -> None:
        """Delete an item from a thread."""
        async with self.session_factory() as session:
            await session.execute(
                text(f"""
                    DELETE FROM {self._get_table_name("items")}
                    WHERE id = :item_id
                    AND thread_id = :thread_id
                    AND user_id = :user_id
                """),
                {
                    "item_id": item_id,
                    "thread_id": thread_id,
                    "user_id": context.user_id,
                },
            )
            await session.commit()

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: RequestContext,
    ) -> Page[ThreadMetadata]:
        """Load paginated list of threads for a user."""
        async with self.session_factory() as session:
            created_after: datetime | None = None

            if after:
                result = await session.execute(
                    text(f"""
                        SELECT created_at FROM {self._get_table_name("threads")}
                        WHERE id = :after_id AND user_id = :user_id
                    """),
                    {"after_id": after, "user_id": context.user_id},
                )
                row = result.first()
                if not row:
                    raise NotFoundError(f"Thread {after} not found")
                created_after = row[0]

            params: dict[str, Any] = {
                "user_id": context.user_id,
                "limit": limit + 1,
            }

            if created_after:
                comparison = ">" if order == "asc" else "<"
                params["created_after"] = created_after
                created_clause = f"AND created_at {comparison} :created_after"
            else:
                created_clause = ""

            result = await session.execute(
                text(f"""
                    SELECT data FROM {self._get_table_name("threads")}
                    WHERE user_id = :user_id
                    {created_clause}
                    ORDER BY created_at {order}
                    LIMIT :limit
                """),
                params,
            )

            threads = [
                # PostgreSQL JSONB is automatically parsed to dict by asyncpg
                # row[0] is already the full ThreadData dict: {"thread": {...}}
                ThreadData.model_validate(row[0]).thread
                for row in result.fetchall()
            ]

            has_more = len(threads) > limit
            if has_more:
                threads = threads[:limit]

            next_after = threads[-1].id if (has_more and threads) else None

            return Page[ThreadMetadata](
                data=threads,
                has_more=has_more,
                after=next_after,
            )

    async def delete_thread(self, thread_id: str, context: RequestContext) -> None:
        """Delete a thread and all its items."""
        async with self.session_factory() as session:
            # Delete items first
            await session.execute(
                text(f"""
                    DELETE FROM {self._get_table_name("items")}
                    WHERE thread_id = :thread_id AND user_id = :user_id
                """),
                {"thread_id": thread_id, "user_id": context.user_id},
            )

            # Delete thread
            await session.execute(
                text(f"""
                    DELETE FROM {self._get_table_name("threads")}
                    WHERE id = :thread_id AND user_id = :user_id
                """),
                {"thread_id": thread_id, "user_id": context.user_id},
            )

            await session.commit()

    async def save_attachment(
        self, attachment: Attachment, context: RequestContext
    ) -> None:
        """Save attachment metadata."""
        attachment_data = AttachmentData(attachment=attachment)

        async with self.session_factory() as session:
            await session.execute(
                text(f"""
                    INSERT INTO {self._get_table_name("attachments")}
                    (id, user_id, organization_id, data)
                    VALUES (:id, :user_id, :org_id, :data)
                    ON CONFLICT (id, user_id) DO UPDATE SET
                        data = EXCLUDED.data
                """),
                {
                    "id": attachment.id,
                    "user_id": context.user_id,
                    "org_id": context.organization_id,
                    "data": attachment_data.model_dump_json(),
                },
            )
            await session.commit()

    async def load_attachment(
        self, attachment_id: str, context: RequestContext
    ) -> Attachment:
        """Load attachment metadata."""
        async with self.session_factory() as session:
            result = await session.execute(
                text(f"""
                    SELECT data FROM {self._get_table_name("attachments")}
                    WHERE id = :attachment_id AND user_id = :user_id
                """),
                {"attachment_id": attachment_id, "user_id": context.user_id},
            )
            row = result.first()

            if not row:
                raise NotFoundError(f"Attachment {attachment_id} not found")

            # PostgreSQL JSONB is automatically parsed to dict by asyncpg
            # row[0] is already the full AttachmentData dict: {"attachment": {...}}
            return AttachmentData.model_validate(row[0]).attachment

    async def delete_attachment(
        self, attachment_id: str, context: RequestContext
    ) -> None:
        """Delete attachment metadata."""
        async with self.session_factory() as session:
            await session.execute(
                text(f"""
                    DELETE FROM {self._get_table_name("attachments")}
                    WHERE id = :attachment_id AND user_id = :user_id
                """),
                {"attachment_id": attachment_id, "user_id": context.user_id},
            )
            await session.commit()

    async def close(self) -> None:
        """Close the database connection pool."""
        await self.engine.dispose()
        logger.info("PostgresStore connection pool closed")
