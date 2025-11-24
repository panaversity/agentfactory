"""Summary management tools for PanaversityFS.

Implements 4 MCP tools for chapter summary management:
- add_summary: Create new summary file
- update_summary: Overwrite existing summary file
- get_summary: Read summary with metadata
- list_summaries: List all summaries for book or chapter
"""

from panaversity_fs.server import mcp
from panaversity_fs.models import (
    AddSummaryInput, UpdateSummaryInput, GetSummaryInput, ListSummariesInput,
    SummaryMetadata, OperationType, OperationStatus
)
from panaversity_fs.storage import get_operator
from panaversity_fs.storage_utils import compute_sha256, validate_path
from panaversity_fs.errors import ContentNotFoundError, InvalidPathError
from panaversity_fs.audit import log_operation
from panaversity_fs.config import get_config
from datetime import datetime, timezone
import json


@mcp.tool(
    name="add_summary",
    annotations={
        "title": "Create Chapter Summary",
        "readOnlyHint": False,
        "destructiveHint": False,
        "idempotentHint": False,
        "openWorldHint": False
    }
)
async def add_summary(params: AddSummaryInput) -> str:
    """Create new chapter summary file (FR-014).

    Creates summary markdown at path: books/[book-id]/chapters/[chapter-id]/.summary.md

    Args:
        params (AddSummaryInput): Validated input containing:
            - book_id (str): Book identifier
            - chapter_id (str): Chapter identifier (e.g., 'chapter-01')
            - content (str): Summary markdown content

    Returns:
        str: JSON response with success confirmation

    Example:
        ```
        Input: {
          "book_id": "ai-native-python",
          "chapter_id": "chapter-01",
          "content": "# Chapter 1 Summary\n\nKey concepts:\n- Introduction to Python\n- Basic syntax\n"
        }
        Output: {
          "status": "success",
          "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
          "file_size": 123,
          "file_hash": "a591a6d40bf420404a011733cfb7b190..."
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Build storage path
        summary_path = f"books/{params.book_id}/chapters/{params.chapter_id}/.summary.md"

        # Get operator
        op = get_operator()

        # Check if summary already exists (add should create new, not overwrite)
        try:
            await op.stat(summary_path)
            # If stat succeeds, file exists
            return json.dumps({
                "status": "error",
                "message": f"Summary already exists at {summary_path}. Use update_summary to modify existing summaries.",
                "path": summary_path
            }, indent=2)
        except:
            # File doesn't exist, proceed with create
            pass

        # Write content
        content_bytes = params.content.encode('utf-8')
        await op.write(summary_path, content_bytes)

        # Get metadata
        metadata = await op.stat(summary_path)
        file_hash = compute_sha256(content_bytes)

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.ADD_SUMMARY,
            path=summary_path,
            agent_id="system",  # TODO: Get from auth context
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        # Build response
        response = {
            "status": "success",
            "path": summary_path,
            "file_size": metadata.content_length,
            "file_hash": file_hash
        }

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.ADD_SUMMARY,
            path=f"books/{params.book_id}/chapters/{params.chapter_id}/.summary.md",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error adding summary: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="update_summary",
    annotations={
        "title": "Update Chapter Summary",
        "readOnlyHint": False,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def update_summary(params: UpdateSummaryInput) -> str:
    """Update existing chapter summary file (FR-015).

    Overwrites summary markdown at path: books/[book-id]/chapters/[chapter-id]/.summary.md

    Args:
        params (UpdateSummaryInput): Validated input containing:
            - book_id (str): Book identifier
            - chapter_id (str): Chapter identifier (e.g., 'chapter-01')
            - content (str): Summary markdown content

    Returns:
        str: JSON response with success confirmation

    Example:
        ```
        Input: {
          "book_id": "ai-native-python",
          "chapter_id": "chapter-01",
          "content": "# Chapter 1 Summary (Updated)\n\nRevised key concepts:\n- Introduction to Python\n"
        }
        Output: {
          "status": "success",
          "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
          "file_size": 145,
          "file_hash": "b592b7e41cf531515b012844dgc8c201...",
          "mode": "updated"
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Build storage path
        summary_path = f"books/{params.book_id}/chapters/{params.chapter_id}/.summary.md"

        # Get operator
        op = get_operator()

        # Write content (overwrite if exists, create if not)
        content_bytes = params.content.encode('utf-8')
        await op.write(summary_path, content_bytes)

        # Get metadata
        metadata = await op.stat(summary_path)
        file_hash = compute_sha256(content_bytes)

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.UPDATE_SUMMARY,
            path=summary_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        # Build response
        response = {
            "status": "success",
            "path": summary_path,
            "file_size": metadata.content_length,
            "file_hash": file_hash,
            "mode": "updated"
        }

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.UPDATE_SUMMARY,
            path=f"books/{params.book_id}/chapters/{params.chapter_id}/.summary.md",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error updating summary: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="get_summary",
    annotations={
        "title": "Get Chapter Summary",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def get_summary(params: GetSummaryInput) -> str:
    """Get chapter summary with metadata (FR-016).

    Returns summary content plus metadata: file_size, last_modified, storage_backend, file_hash_sha256, path.

    Args:
        params (GetSummaryInput): Validated input containing:
            - book_id (str): Book identifier
            - chapter_id (str): Chapter identifier (e.g., 'chapter-01')

    Returns:
        str: JSON response with summary metadata and content

    Example:
        ```
        Input: {
          "book_id": "ai-native-python",
          "chapter_id": "chapter-01"
        }
        Output: {
          "file_size": 145,
          "last_modified": "2025-11-24T12:00:00Z",
          "storage_backend": "fs",
          "file_hash_sha256": "b592b7e41cf531515b012844dgc8c201...",
          "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
          "content": "# Chapter 1 Summary\n\nKey concepts:\n..."
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Build storage path
        summary_path = f"books/{params.book_id}/chapters/{params.chapter_id}/.summary.md"

        # Get operator
        op = get_operator()
        config = get_config()

        # Read content
        try:
            content_bytes = await op.read(summary_path)
            content = content_bytes.decode('utf-8')
        except:
            raise ContentNotFoundError(summary_path)

        # Get metadata
        metadata = await op.stat(summary_path)

        # Compute hash
        file_hash = compute_sha256(content_bytes)

        # Build response
        response = {
            "file_size": metadata.content_length,
            "last_modified": metadata.last_modified.isoformat(),
            "storage_backend": config.storage_backend,
            "file_hash_sha256": file_hash,
            "path": summary_path,
            "content": content
        }

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.GET_SUMMARY,
            path=summary_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return json.dumps(response, indent=2)

    except ContentNotFoundError:
        # Log error
        await log_operation(
            operation=OperationType.GET_SUMMARY,
            path=f"books/{params.book_id}/chapters/{params.chapter_id}/.summary.md",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message="Summary not found"
        )

        raise

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.GET_SUMMARY,
            path=f"books/{params.book_id}/chapters/{params.chapter_id}/.summary.md",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error getting summary: {type(e).__name__}: {str(e)}"


@mcp.tool(
    name="list_summaries",
    annotations={
        "title": "List Chapter Summaries",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def list_summaries(params: ListSummariesInput) -> str:
    """List all summaries for book or chapter (FR-017).

    Returns array of summary metadata: [{path, file_size, last_modified}]

    Args:
        params (ListSummariesInput): Validated input containing:
            - book_id (str): Book identifier
            - chapter_id (str | None): Optional chapter filter

    Returns:
        str: JSON array of summary metadata

    Example:
        ```
        # List all summaries in book
        Input: {"book_id": "ai-native-python"}
        Output: [
          {
            "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
            "file_size": 145,
            "last_modified": "2025-11-24T12:00:00Z"
          },
          {
            "path": "books/ai-native-python/chapters/chapter-02/.summary.md",
            "file_size": 203,
            "last_modified": "2025-11-24T13:00:00Z"
          }
        ]

        # List specific chapter summary
        Input: {"book_id": "ai-native-python", "chapter_id": "chapter-01"}
        Output: [
          {
            "path": "books/ai-native-python/chapters/chapter-01/.summary.md",
            "file_size": 145,
            "last_modified": "2025-11-24T12:00:00Z"
          }
        ]
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Get operator
        op = get_operator()

        # Determine search pattern
        if params.chapter_id:
            # List specific chapter summary
            search_path = f"books/{params.book_id}/chapters/{params.chapter_id}/"
        else:
            # List all chapter summaries
            search_path = f"books/{params.book_id}/chapters/"

        # Collect summaries
        summaries = []

        try:
            # List directory contents
            entries = await op.list(search_path)

            async for entry in entries:
                # Check if this is a summary file or a chapter directory containing summary
                if entry.path.endswith('/.summary.md'):
                    # Direct match: this is a summary file
                    try:
                        metadata = await op.stat(entry.path)

                        summary_info = {
                            "path": entry.path,
                            "file_size": metadata.content_length,
                            "last_modified": metadata.last_modified.isoformat()
                        }

                        summaries.append(summary_info)
                    except:
                        # Skip files that can't be accessed
                        continue

                elif entry.path.endswith('/') and not params.chapter_id:
                    # This is a chapter directory, check for .summary.md inside
                    try:
                        summary_file_path = f"{entry.path}.summary.md"
                        metadata = await op.stat(summary_file_path)

                        summary_info = {
                            "path": summary_file_path,
                            "file_size": metadata.content_length,
                            "last_modified": metadata.last_modified.isoformat()
                        }

                        summaries.append(summary_info)
                    except:
                        # This chapter doesn't have a summary, skip
                        continue

        except Exception:
            # Directory doesn't exist or can't be accessed
            pass

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.LIST_SUMMARIES,
            path=search_path,
            agent_id="system",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time
        )

        return json.dumps(summaries, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.LIST_SUMMARIES,
            path=f"books/{params.book_id}/chapters/",
            agent_id="system",
            status=OperationStatus.ERROR,
            error_message=str(e)
        )

        return f"Error listing summaries: {type(e).__name__}: {str(e)}"
