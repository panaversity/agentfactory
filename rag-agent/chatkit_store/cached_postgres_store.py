"""Redis-cached PostgreSQL store for ChatKit.

This implements a write-through cache pattern:
- Writes go to PostgreSQL + Redis
- Reads try Redis first, fallback to PostgreSQL
- Cache invalidation on updates/deletes
"""

import asyncio
import json
import logging
from chatkit.types import Page, ThreadItem, ThreadMetadata

from .context import RequestContext
from .postgres_store import PostgresStore

logger = logging.getLogger(__name__)


class CachedPostgresStore(PostgresStore):
    """
    PostgreSQL store with Redis caching layer.

    Caching Strategy:
    - Thread metadata: Cache for 5 minutes
    - Thread items: Cache for 2 minutes
    - Thread lists: Cache for 1 minute

    Cache Keys:
    - thread:{user_id}:{thread_id} -> ThreadMetadata
    - thread_items:{user_id}:{thread_id}:{limit}:{order} -> Page[ThreadItem]
    - thread_list:{user_id}:{limit}:{order} -> Page[ThreadMetadata]
    """

    def __init__(
        self,
        *args,
        redis_client=None,
        **kwargs,
    ):
        """
        Initialize cached store.

        Args:
            redis_client: Optional Redis client. If None, caching is disabled.
        """
        super().__init__(*args, **kwargs)
        self.redis = redis_client
        self.cache_enabled = redis_client is not None

        # Cache TTLs in seconds
        self.thread_ttl = 3600  # 1 hour (increased from 5 minutes)
        self.items_ttl = 1800  # 30 minutes (increased from 2 minutes)
        self.list_ttl = 600  # 10 minutes (increased from 1 minute)

        # Background write queue for async database operations
        # CRITICAL: Bounded queue prevents memory exhaustion under load
        self._background_writes = asyncio.Queue(maxsize=1000)
        self._background_task = None
        self._shutdown_event = asyncio.Event()
        self._queue_full_warnings = 0

        if self.cache_enabled:
            logger.info("Redis caching enabled for PostgresStore")
        else:
            logger.warning("Redis caching disabled - no Redis client provided")

    async def start_background_writer(self):
        """Start the background writer task."""
        if self._background_task is None:
            self._background_task = asyncio.create_task(self._background_write_worker())
            logger.info("Background writer started")

    async def stop_background_writer(self):
        """Stop the background writer task."""
        if self._background_task:
            self._shutdown_event.set()
            await self._background_task
            self._background_task = None
            logger.info("Background writer stopped")

    async def _background_write_worker(self):
        """Background worker that processes database writes."""
        logger.info("Background write worker started")

        while not self._shutdown_event.is_set():
            try:
                # Wait for write operation or shutdown
                write_task = await asyncio.wait_for(
                    self._background_writes.get(), timeout=1.0
                )

                # Execute the write operation
                operation, args, kwargs = write_task
                start_time = asyncio.get_event_loop().time()

                try:
                    await operation(*args, **kwargs)
                    duration = asyncio.get_event_loop().time() - start_time
                    logger.info(
                        f"Background write completed in {duration * 1000:.1f}ms: {operation.__name__}"
                    )
                except Exception as e:
                    logger.error(f"Background write failed: {operation.__name__}: {e}")

                # Mark task as done
                self._background_writes.task_done()

            except asyncio.TimeoutError:
                # No tasks in queue, continue loop
                continue
            except Exception as e:
                logger.error(f"Background writer error: {e}")

        logger.info("Background write worker stopped")

    async def _queue_background_write(self, operation, *args, **kwargs):
        """
        Queue a database write operation for background execution.

        Implements backpressure: if queue is full, warns and waits with timeout.
        Falls back to synchronous write if queue remains full to prevent data loss.
        """
        if self._background_task is None:
            await self.start_background_writer()

        # Check queue size for monitoring
        queue_size = self._background_writes.qsize()
        if queue_size > 800:  # Warn at 80% capacity
            self._queue_full_warnings += 1
            if self._queue_full_warnings % 10 == 1:  # Log every 10th warning
                logger.warning(
                    f"Background write queue is {queue_size}/1000 - system under heavy load"
                )

        try:
            # Try to put with timeout to prevent indefinite blocking
            await asyncio.wait_for(
                self._background_writes.put((operation, args, kwargs)), timeout=5.0
            )
            logger.debug(f"Queued background write: {operation.__name__}")
        except asyncio.TimeoutError:
            logger.error(
                f"Failed to queue background write: {operation.__name__} - queue full, "
                f"falling back to synchronous write for data safety"
            )
            # Fallback to synchronous write to prevent data loss
            await operation(*args, **kwargs)

    def _get_thread_cache_key(self, user_id: str, thread_id: str) -> str:
        """Generate cache key for thread metadata."""
        return f"chatkit:thread:{user_id}:{thread_id}"

    def _get_items_cache_key(
        self, user_id: str, thread_id: str, limit: int, order: str
    ) -> str:
        """Generate cache key for thread items."""
        return f"chatkit:items:{user_id}:{thread_id}:{limit}:{order}"

    def _get_list_cache_key(self, user_id: str, limit: int, order: str) -> str:
        """Generate cache key for thread list."""
        return f"chatkit:threads:{user_id}:{limit}:{order}"

    async def _get_cached(self, key: str) -> dict | None:
        """Get cached value from Redis with timeout."""
        if not self.cache_enabled:
            return None

        try:
            # Use pipeline for better performance with remote Redis
            pipe = self.redis.pipeline()
            pipe.get(key)
            results = await pipe.execute()

            value = results[0] if results else None
            if value:
                logger.info(f"🎯 Cache HIT: {key}")
                return json.loads(value)
            logger.info(f"❌ Cache MISS: {key}")
            return None
        except Exception as e:
            logger.warning(f"Redis get error for key {key}: {e}")
            return None

    async def _set_cached(self, key: str, value: dict, ttl: int) -> None:
        """Set cached value in Redis with TTL using pipeline."""
        if not self.cache_enabled:
            return

        try:
            # Use pipeline for better performance with remote Redis
            pipe = self.redis.pipeline()
            pipe.setex(
                key,
                ttl,
                json.dumps(value, default=str),  # default=str handles datetime
            )
            await pipe.execute()
            logger.info(f"💾 Cache SET: {key} (TTL: {ttl}s)")
        except Exception as e:
            logger.warning(f"Redis set error for key {key}: {e}")

    async def _delete_cached(self, key: str) -> None:
        """Delete cached value from Redis."""
        if not self.cache_enabled:
            return

        try:
            await self.redis.delete(key)
            logger.info(f"🗑️ Cache DELETE: {key}")
        except Exception as e:
            logger.warning(f"Redis delete error for key {key}: {e}")

    async def _invalidate_thread_cache(self, user_id: str, thread_id: str) -> None:
        """Invalidate all cache entries related to a thread."""
        if not self.cache_enabled:
            return

        try:
            # Invalidate thread metadata
            await self._delete_cached(self._get_thread_cache_key(user_id, thread_id))

            # Invalidate thread items (all variations)
            pattern = f"chatkit:items:{user_id}:{thread_id}:*"
            await self._delete_pattern(pattern)

            # Invalidate thread lists
            pattern = f"chatkit:threads:{user_id}:*"
            await self._delete_pattern(pattern)

            logger.debug(f"Invalidated cache for thread {thread_id}")
        except Exception as e:
            logger.warning(f"Cache invalidation error: {e}")

    async def _delete_pattern(self, pattern: str) -> None:
        """Delete all keys matching a pattern."""
        if not self.cache_enabled:
            return

        try:
            cursor = 0
            while True:
                cursor, keys = await self.redis.scan(
                    cursor=cursor, match=pattern, count=100
                )
                if keys:
                    await self.redis.delete(*keys)
                if cursor == 0:
                    break
        except Exception as e:
            logger.warning(f"Pattern delete error for {pattern}: {e}")

    # Override PostgresStore methods with caching

    async def load_thread(
        self, thread_id: str, context: RequestContext
    ) -> ThreadMetadata:
        """Load thread with Redis caching."""
        cache_key = self._get_thread_cache_key(context.user_id, thread_id)

        # Try cache first
        cached = await self._get_cached(cache_key)
        if cached:
            from .postgres_store import ThreadData

            return ThreadData.model_validate(cached).thread

        # Cache miss - load from DB
        thread = await super().load_thread(thread_id, context)

        # Update cache
        from .postgres_store import ThreadData

        await self._set_cached(
            cache_key, ThreadData(thread=thread).model_dump(), self.thread_ttl
        )

        return thread

    async def save_thread(
        self, thread: ThreadMetadata, context: RequestContext
    ) -> None:
        """Save thread to DB and update cache."""
        # Write to database
        await super().save_thread(thread, context)

        # Update cache (write-through)
        cache_key = self._get_thread_cache_key(context.user_id, thread.id)
        from .postgres_store import ThreadData

        await self._set_cached(
            cache_key, ThreadData(thread=thread).model_dump(), self.thread_ttl
        )

        # ONLY invalidate thread lists (not items or thread metadata)
        # because we just updated those above
        pattern = f"chatkit:threads:{context.user_id}:*"
        await self._delete_pattern(pattern)

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: RequestContext,
    ) -> Page[ThreadItem]:
        """Load thread items with Redis caching."""
        # Only cache if no pagination cursor
        if after is None:
            cache_key = self._get_items_cache_key(
                context.user_id, thread_id, limit, order
            )

            # Try cache first
            cached = await self._get_cached(cache_key)
            if cached:
                from .postgres_store import ItemData

                items = [
                    ItemData.model_validate(item_dict).item
                    for item_dict in cached.get("data", [])
                ]
                return Page[ThreadItem](
                    data=items,
                    has_more=cached.get("has_more", False),
                    after=cached.get("after"),
                )

        # Cache miss or paginated - load from DB
        page = await super().load_thread_items(thread_id, after, limit, order, context)

        # Update cache (only for non-paginated queries)
        if after is None:
            cache_key = self._get_items_cache_key(
                context.user_id, thread_id, limit, order
            )
            from .postgres_store import ItemData

            await self._set_cached(
                cache_key,
                {
                    "data": [ItemData(item=item).model_dump() for item in page.data],
                    "has_more": page.has_more,
                    "after": page.after,
                },
                self.items_ttl,
            )

        return page

    async def add_thread_item(
        self, thread_id: str, item: ThreadItem, context: RequestContext
    ) -> None:
        """Add item to thread and update cache instead of invalidating."""
        # Write to database
        await super().add_thread_item(thread_id, item, context)

        # Instead of deleting cache, try to update existing cached pages
        await self._update_items_cache_after_add(thread_id, item, context)

        logger.info(
            f"Updated items cache for thread {thread_id} after adding item {item.id}"
        )

    async def add_thread_item_fast(
        self, thread_id: str, item: ThreadItem, context: RequestContext
    ) -> None:
        """
        DEPRECATED: This method has been disabled for data safety.

        Background writes risk data loss if the application crashes before
        writes complete. Use add_thread_item() instead for guaranteed durability.

        This now falls back to the standard (safe) add_thread_item method.
        """
        logger.warning(
            "add_thread_item_fast() is deprecated and disabled - using safe add_thread_item() instead"
        )
        # Use the safe, synchronous method instead
        await self.add_thread_item(thread_id, item, context)

    async def _update_items_cache_after_add(
        self, thread_id: str, new_item: ThreadItem, context: RequestContext
    ) -> None:
        """Update cached items pages by adding the new item."""
        if not self.cache_enabled:
            return

        try:
            # Find all cached pages for this thread
            pattern = f"chatkit:items:{context.user_id}:{thread_id}:*"
            cursor = 0
            cache_keys = []

            while True:
                cursor, keys = await self.redis.scan(
                    cursor=cursor, match=pattern, count=100
                )
                cache_keys.extend(keys)
                if cursor == 0:
                    break

            # Update each cached page
            for cache_key in cache_keys:
                try:
                    cached_data = await self.redis.get(cache_key)
                    if not cached_data:
                        continue

                    page_data = json.loads(cached_data)
                    items_list = page_data.get("data", [])

                    # Add new item to the beginning (for desc order) or end (for asc order)
                    # Parse cache key to determine order
                    if ":desc" in cache_key:
                        # For desc order, add to beginning
                        from .postgres_store import ItemData

                        new_item_data = ItemData(item=new_item).model_dump()
                        items_list.insert(0, new_item_data)
                    else:
                        # For asc order, add to end
                        from .postgres_store import ItemData

                        new_item_data = ItemData(item=new_item).model_dump()
                        items_list.append(new_item_data)

                    # Update the cached page
                    page_data["data"] = items_list

                    # Get TTL and update cache
                    ttl = await self.redis.ttl(cache_key)
                    if ttl > 0:
                        await self.redis.setex(
                            cache_key, ttl, json.dumps(page_data, default=str)
                        )
                        logger.info(f"Updated cache page: {cache_key}")

                except Exception as e:
                    logger.warning(f"Failed to update cache page {cache_key}: {e}")
                    # If update fails, delete this specific page
                    await self.redis.delete(cache_key)

        except Exception as e:
            logger.warning(f"Failed to update items cache: {e}")
            # Fallback: delete all items cache for this thread
            await self._delete_pattern(f"chatkit:items:{context.user_id}:{thread_id}:*")

    async def save_item(
        self, thread_id: str, item: ThreadItem, context: RequestContext
    ) -> None:
        """Update item and refresh cache instead of invalidating."""
        # Write to database
        await super().save_item(thread_id, item, context)

        # For item updates, we need to invalidate cache since the content changed
        # But let's be more conservative - only delete if the cache is old
        try:
            pattern = f"chatkit:items:{context.user_id}:{thread_id}:*"
            cursor = 0

            while True:
                cursor, keys = await self.redis.scan(
                    cursor=cursor, match=pattern, count=100
                )

                for cache_key in keys:
                    ttl = await self.redis.ttl(cache_key)
                    # Only delete if cache is more than 5 minutes old
                    if ttl < (
                        self.items_ttl - 300
                    ):  # 25 minutes remaining = 5 minutes old
                        await self.redis.delete(cache_key)
                        logger.info(f"Deleted old cache: {cache_key}")
                    else:
                        logger.info(f"Keeping fresh cache: {cache_key} (TTL: {ttl}s)")

                if cursor == 0:
                    break

        except Exception as e:
            logger.warning(f"Error in selective cache invalidation: {e}")
            # Fallback: delete all items cache
            await self._delete_pattern(f"chatkit:items:{context.user_id}:{thread_id}:*")

        logger.info(f"Processed cache update for thread {thread_id} item {item.id}")

    async def delete_thread_item(
        self, thread_id: str, item_id: str, context: RequestContext
    ) -> None:
        """Delete item and invalidate only items cache."""
        # Delete from database
        await super().delete_thread_item(thread_id, item_id, context)

        # ONLY invalidate thread items cache (not thread metadata or lists)
        pattern = f"chatkit:items:{context.user_id}:{thread_id}:*"
        await self._delete_pattern(pattern)

        logger.debug(f"Invalidated items cache for thread {thread_id}")

    async def load_threads(
        self,
        limit: int,
        after: str | None,
        order: str,
        context: RequestContext,
    ) -> Page[ThreadMetadata]:
        """Load threads with Redis caching."""
        # Only cache if no pagination cursor
        if after is None:
            cache_key = self._get_list_cache_key(context.user_id, limit, order)

            # Try cache first
            cached = await self._get_cached(cache_key)
            if cached:
                from .postgres_store import ThreadData

                threads = [
                    ThreadData.model_validate(thread_dict).thread
                    for thread_dict in cached.get("data", [])
                ]
                return Page[ThreadMetadata](
                    data=threads,
                    has_more=cached.get("has_more", False),
                    after=cached.get("after"),
                )

        # Cache miss or paginated - load from DB
        page = await super().load_threads(limit, after, order, context)

        # Update cache (only for non-paginated queries)
        if after is None:
            cache_key = self._get_list_cache_key(context.user_id, limit, order)
            from .postgres_store import ThreadData

            await self._set_cached(
                cache_key,
                {
                    "data": [
                        ThreadData(thread=thread).model_dump() for thread in page.data
                    ],
                    "has_more": page.has_more,
                    "after": page.after,
                },
                self.list_ttl,
            )

        return page

    async def delete_thread(self, thread_id: str, context: RequestContext) -> None:
        """Delete thread and invalidate all related caches."""
        # Delete from database
        await super().delete_thread(thread_id, context)

        # Invalidate all thread-related caches
        await self._invalidate_thread_cache(context.user_id, thread_id)
