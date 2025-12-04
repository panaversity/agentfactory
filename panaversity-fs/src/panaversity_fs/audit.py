"""JSONL audit logging for PanaversityFS.

Implements direct JSONL writes with eventual consistency (FR-018).
Audit entries are written to .audit/YYYY-MM-DD.jsonl files.
"""

import aiofiles
import json
from datetime import datetime, timezone
from pathlib import Path
from panaversity_fs.models import AuditEntry, OperationType, OperationStatus
from panaversity_fs.storage import get_operator


async def log_operation(
    operation: OperationType,
    path: str,
    agent_id: str,
    status: OperationStatus,
    error_message: str | None = None,
    execution_time_ms: int | None = None
) -> None:
    """Log operation to JSONL audit file.

    Writes to .audit/YYYY-MM-DD.jsonl in storage backend.
    Uses async append with eventual consistency (line interleaving acceptable per FR-018).

    Args:
        operation: Operation type performed
        path: File path affected
        agent_id: Agent/user ID performing operation
        status: Operation result status
        error_message: Error details if status=error
        execution_time_ms: Operation execution time in milliseconds

    Example:
        ```python
        await log_operation(
            operation=OperationType.WRITE_CONTENT,
            path="books/ai-native-python/lessons/part-1/chapter-01/lesson-01.md",
            agent_id="agent-123",
            status=OperationStatus.SUCCESS,
            execution_time_ms=45
        )
        ```
    """
    try:
        # Create audit entry
        entry = AuditEntry(
            timestamp=datetime.now(timezone.utc),
            agent_id=agent_id,
            operation=operation,
            path=path,
            status=status,
            error_message=error_message,
            execution_time_ms=execution_time_ms
        )

        # Get today's audit file path
        today = datetime.now(timezone.utc).strftime("%Y-%m-%d")
        audit_path = f".audit/{today}.jsonl"

        # Serialize to JSONL (single line)
        jsonl_line = entry.model_dump_json() + "\n"

        # Write to storage backend via OpenDAL
        # Note: This is an append operation. We read existing content,
        # append new line, and write back. Not atomic, but acceptable per FR-018.
        op = get_operator()

        try:
            # Read existing content
            existing_content = await op.read(audit_path)
            new_content = existing_content + jsonl_line.encode('utf-8')
        except Exception:
            # File doesn't exist yet, create it
            new_content = jsonl_line.encode('utf-8')

        # Write back (upsert)
        await op.write(audit_path, new_content)

    except Exception as e:
        # Audit logging failures should not block operations
        # Log to stderr but don't raise
        import sys
        print(f"WARNING: Audit log write failed: {e}", file=sys.stderr)


async def query_audit_log(
    start_date: str,
    end_date: str,
    agent_id: str | None = None,
    operation: OperationType | None = None,
    status: OperationStatus | None = None,
    limit: int = 100
) -> list[AuditEntry]:
    """Query audit log entries with filters (FR-020).

    Scans JSONL files for date range and applies filters.

    Args:
        start_date: Start date (YYYY-MM-DD)
        end_date: End date (YYYY-MM-DD, inclusive)
        agent_id: Filter by agent ID (optional)
        operation: Filter by operation type (optional)
        status: Filter by status (optional)
        limit: Maximum entries to return (default: 100, max per FR-020)

    Returns:
        list[AuditEntry]: Matching audit entries (most recent first)

    Example:
        ```python
        entries = await query_audit_log(
            start_date="2025-11-24",
            end_date="2025-11-24",
            agent_id="agent-123",
            status=OperationStatus.SUCCESS,
            limit=50
        )
        ```
    """
    entries: list[AuditEntry] = []
    op = get_operator()

    # Parse date range
    from datetime import datetime, timedelta
    start = datetime.strptime(start_date, "%Y-%m-%d")
    end = datetime.strptime(end_date, "%Y-%m-%d")

    # Iterate through date range
    current = start
    while current <= end:
        date_str = current.strftime("%Y-%m-%d")
        audit_path = f".audit/{date_str}.jsonl"

        try:
            # Read JSONL file
            content = await op.read(audit_path)
            lines = content.decode('utf-8').strip().split('\n')

            # Parse each line
            for line in lines:
                if not line.strip():
                    continue

                try:
                    data = json.loads(line)
                    entry = AuditEntry(**data)

                    # Apply filters
                    if agent_id and entry.agent_id != agent_id:
                        continue
                    if operation and entry.operation != operation:
                        continue
                    if status and entry.status != status:
                        continue

                    entries.append(entry)

                except Exception as e:
                    # Skip malformed lines
                    print(f"WARNING: Skipping malformed audit entry: {e}", file=sys.stderr)
                    continue

        except Exception:
            # File doesn't exist for this date, skip
            pass

        # Move to next day
        current += timedelta(days=1)

    # Sort by timestamp (most recent first)
    entries.sort(key=lambda e: e.timestamp, reverse=True)

    # Apply limit
    return entries[:limit]


async def rotate_audit_logs() -> None:
    """Rotate audit logs at midnight (FR-019).

    This is a no-op for direct JSONL approach - rotation happens naturally
    by using date-based filenames (.audit/YYYY-MM-DD.jsonl).

    This function exists for API compatibility and future enhancements
    (e.g., archiving logs older than 30 days).
    """
    # No-op for direct JSONL approach
    # Future: Could implement archiving of old logs here
    pass
