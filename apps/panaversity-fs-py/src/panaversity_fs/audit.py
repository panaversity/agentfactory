"""Database-backed audit logging for PanaversityFS.

Implements append-only audit log with hash chain integrity per FR-020, FR-021, FR-022, FR-023.
Uses PostgreSQL/SQLite via SQLAlchemy for production-grade audit trail.
"""

import sys
from datetime import datetime, timezone
from typing import Optional

from sqlalchemy import select, and_, desc
from sqlalchemy.exc import SQLAlchemyError

from panaversity_fs.database.connection import get_session
from panaversity_fs.database.models import AuditLog
from panaversity_fs.models import OperationType, OperationStatus
from panaversity_fs.config import get_config


# Default agent ID for unauthenticated dev mode
DEV_MODE_AGENT_ID = "dev-mode-agent"


def get_agent_id_from_context() -> str:
    """Extract agent_id from MCP request context (FR-021).

    Uses MCP SDK's auth_context_var to retrieve the authenticated user from the
    current request. The auth middleware populates this contextvar when a valid
    Bearer token is provided.

    Behavior:
    - If authenticated: Returns client_id from the verified AccessToken
    - If not authenticated AND auth is enabled: Raises RuntimeError (FR-021 enforcement)
    - If not authenticated AND auth is disabled (dev mode): Returns DEV_MODE_AGENT_ID

    Returns:
        Agent ID from authenticated token, or dev-mode fallback

    Raises:
        RuntimeError: If auth is enabled but no authenticated user found
    """
    # Import here to avoid circular imports and allow graceful fallback
    try:
        from mcp.server.auth.middleware.auth_context import get_access_token
        access_token = get_access_token()
    except ImportError:
        # MCP SDK not available (shouldn't happen in normal operation)
        access_token = None

    if access_token is not None:
        # Authenticated request - use client_id from verified token
        return access_token.client_id

    # No authenticated token - check if auth is required
    config = get_config()
    if config.auth_enabled:
        # FR-021: Reject unauthenticated requests in production
        raise RuntimeError(
            "FR-021 VIOLATION: Unauthenticated request in production mode. "
            "All operations require authenticated agent identity. "
            "Ensure request includes valid Authorization: Bearer <token> header."
        )

    # Dev mode - return placeholder
    return DEV_MODE_AGENT_ID


def _parse_path_components(full_path: str) -> tuple[str, str, str]:
    """Parse full path into (book_id, relative_path, user_id).

    Handles both base paths and overlay paths:
    - Base: books/{book_id}/content/... -> (book_id, content/..., "__base__")
    - Overlay: books/{book_id}/users/{user_id}/content/... -> (book_id, content/..., user_id)

    Args:
        full_path: Full storage path

    Returns:
        Tuple of (book_id, relative_path, user_id)
    """
    parts = full_path.split("/")

    if len(parts) < 2 or parts[0] != "books":
        return ("unknown", full_path, "__base__")

    book_id = parts[1]

    # Check for overlay path: books/{book_id}/users/{user_id}/...
    if len(parts) > 4 and parts[2] == "users":
        user_id = parts[3]
        relative_path = "/".join(parts[4:])
        return (book_id, relative_path, user_id)

    # Base path: books/{book_id}/...
    relative_path = "/".join(parts[2:]) if len(parts) > 2 else ""
    return (book_id, relative_path, "__base__")


async def log_operation(
    operation: OperationType,
    path: str,
    status: OperationStatus,
    agent_id: str | None = None,
    error_message: str | None = None,
    execution_time_ms: int | None = None,
    new_hash: str | None = None,
    book_id: str | None = None,
    user_id: str | None = None,
) -> None:
    """Log operation to audit database with hash chain integrity (FR-020, FR-022, FR-023).

    Append-only INSERT (FR-023) with hash chain linking consecutive operations
    on the same (book_id, path, user_id) tuple (FR-022).

    FR-021 Compliance:
    - Automatically extracts agent_id from MCP auth context (via contextvar)
    - In production (auth enabled), rejects if no authenticated identity
    - In dev mode (auth disabled), uses "dev-mode-agent" placeholder

    Args:
        operation: Operation type performed
        path: File path affected (full path or relative)
        status: Operation result status
        agent_id: Agent/user ID performing operation (optional - auto-extracted if not provided)
        error_message: Error details if status=error
        execution_time_ms: Operation execution time in milliseconds
        new_hash: SHA256 hash of content after operation (null for delete/read)
        book_id: Book identifier (extracted from path if not provided)
        user_id: User ID for overlay operations (extracted from path if not provided)

    Example:
        ```python
        # Agent ID is auto-extracted from authenticated request context
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path="books/ai-native-python/content/01-Part/01-Chapter/01-lesson.md",
            status=OperationStatus.SUCCESS,
            execution_time_ms=45,
            new_hash="abc123..."
        )
        ```
    """
    # FR-021: Use agent_id if provided, otherwise extract from auth context
    if agent_id is None:
        agent_id = get_agent_id_from_context()
    try:
        # ALWAYS parse path to get relative path for consistent storage
        # This ensures verify_hash_chain can find entries regardless of how path was passed
        parsed_book_id, relative_path, parsed_user_id = _parse_path_components(path)

        # Use explicitly provided values if available, otherwise use parsed values
        book_id = book_id or parsed_book_id
        user_id = user_id or parsed_user_id

        async with get_session() as session:
            # Query prev_hash from previous entry on same (book_id, path, user_id) (FR-022)
            prev_hash_stmt = select(AuditLog.new_hash).where(
                and_(
                    AuditLog.book_id == book_id,
                    AuditLog.path == relative_path,
                    AuditLog.user_id == user_id
                )
            ).order_by(desc(AuditLog.timestamp), desc(AuditLog.id)).limit(1)

            result = await session.execute(prev_hash_stmt)
            prev_hash_row = result.scalar_one_or_none()

            # Create audit entry (append-only INSERT per FR-023)
            audit_entry = AuditLog(
                timestamp=datetime.now(timezone.utc),
                agent_id=agent_id,
                operation=operation.value,
                book_id=book_id,
                path=relative_path,
                user_id=user_id,
                prev_hash=prev_hash_row,
                new_hash=new_hash,
                status=status.value,
                error_message=error_message,
                execution_time_ms=execution_time_ms
            )

            session.add(audit_entry)
            # Commit happens on context exit

    except SQLAlchemyError as e:
        # Audit logging failures should not block operations
        # Log to stderr but don't raise
        print(f"WARNING: Audit log write failed (database): {e}", file=sys.stderr)
    except Exception as e:
        # Catch-all for unexpected errors
        print(f"WARNING: Audit log write failed: {e}", file=sys.stderr)


async def query_audit_log(
    start_date: str | None = None,
    end_date: str | None = None,
    agent_id: str | None = None,
    operation: OperationType | None = None,
    status: OperationStatus | None = None,
    book_id: str | None = None,
    path_pattern: str | None = None,
    limit: int = 100
) -> list[AuditLog]:
    """Query audit log entries with filters (FR-024).

    Queries database for audit entries matching filter criteria.

    Args:
        start_date: Start date (YYYY-MM-DD) - optional
        end_date: End date (YYYY-MM-DD, inclusive) - optional
        agent_id: Filter by agent ID (optional)
        operation: Filter by operation type (optional)
        status: Filter by status (optional)
        book_id: Filter by book ID (optional)
        path_pattern: Filter by path pattern with SQL LIKE syntax (optional)
        limit: Maximum entries to return (default: 100, max per FR-024)

    Returns:
        list[AuditLog]: Matching audit entries (most recent first)

    Example:
        ```python
        entries = await query_audit_log(
            start_date="2025-11-24",
            end_date="2025-11-24",
            agent_id="claude-lesson-writer-7",
            status=OperationStatus.SUCCESS,
            limit=50
        )
        ```
    """
    from datetime import timedelta

    try:
        async with get_session() as session:
            # Build query with dynamic filters
            stmt = select(AuditLog)

            conditions = []

            # Date range filter
            if start_date:
                start_dt = datetime.strptime(start_date, "%Y-%m-%d").replace(tzinfo=timezone.utc)
                conditions.append(AuditLog.timestamp >= start_dt)

            if end_date:
                # End of day for inclusive end date
                end_dt = datetime.strptime(end_date, "%Y-%m-%d").replace(tzinfo=timezone.utc)
                end_dt = end_dt + timedelta(days=1) - timedelta(microseconds=1)
                conditions.append(AuditLog.timestamp <= end_dt)

            # Agent ID filter
            if agent_id:
                conditions.append(AuditLog.agent_id == agent_id)

            # Operation type filter
            if operation:
                conditions.append(AuditLog.operation == operation.value)

            # Status filter
            if status:
                conditions.append(AuditLog.status == status.value)

            # Book ID filter
            if book_id:
                conditions.append(AuditLog.book_id == book_id)

            # Path pattern filter (SQL LIKE)
            if path_pattern:
                conditions.append(AuditLog.path.like(path_pattern))

            # Apply all conditions
            if conditions:
                stmt = stmt.where(and_(*conditions))

            # Order by timestamp descending (most recent first)
            stmt = stmt.order_by(desc(AuditLog.timestamp), desc(AuditLog.id))

            # Apply limit
            stmt = stmt.limit(limit)

            # Execute query
            result = await session.execute(stmt)
            entries = result.scalars().all()

            return list(entries)

    except SQLAlchemyError as e:
        print(f"WARNING: Audit log query failed: {e}", file=sys.stderr)
        return []


async def verify_hash_chain(book_id: str, path: str, user_id: str = "__base__") -> dict:
    """Verify hash chain integrity for a specific file's audit trail (FR-022).

    Checks that entry[n].new_hash == entry[n+1].prev_hash for all consecutive entries.

    Args:
        book_id: Book identifier
        path: Relative path within book
        user_id: User ID for overlays (default: "__base__")

    Returns:
        dict with:
            - valid: bool - Whether hash chain is intact
            - entries: int - Number of entries checked
            - breaks: list - Indices where chain breaks (if any)
    """
    try:
        async with get_session() as session:
            stmt = select(AuditLog).where(
                and_(
                    AuditLog.book_id == book_id,
                    AuditLog.path == path,
                    AuditLog.user_id == user_id
                )
            ).order_by(AuditLog.timestamp, AuditLog.id)

            result = await session.execute(stmt)
            entries = result.scalars().all()

            if len(entries) <= 1:
                return {"valid": True, "entries": len(entries), "breaks": []}

            breaks = []
            for i in range(len(entries) - 1):
                current = entries[i]
                next_entry = entries[i + 1]

                # Chain rule: current.new_hash == next.prev_hash
                if current.new_hash != next_entry.prev_hash:
                    breaks.append({
                        "index": i,
                        "current_new_hash": current.new_hash,
                        "next_prev_hash": next_entry.prev_hash
                    })

            return {
                "valid": len(breaks) == 0,
                "entries": len(entries),
                "breaks": breaks
            }

    except SQLAlchemyError as e:
        print(f"WARNING: Hash chain verification failed: {e}", file=sys.stderr)
        return {"valid": False, "entries": 0, "breaks": [], "error": str(e)}
