"""Delta build detection tools for PanaversityFS.

Implements two delta detection tools:
- delta_build: Timestamp-based detection (files modified since timestamp)
- plan_build: Manifest-hash-based detection (FR-025, FR-026, FR-027)

Both support incremental Docusaurus builds.
"""

from mcp.server.fastmcp.server import Context

from panaversity_fs.app import mcp
from panaversity_fs.models import DeltaBuildInput, PlanBuildInput, OperationType, OperationStatus
from panaversity_fs.storage import get_operator
from panaversity_fs.storage_utils import compute_sha256
from panaversity_fs.audit import log_operation
from panaversity_fs.database.connection import get_session
from panaversity_fs.database.models import FileJournal, ManifestSnapshot
from sqlalchemy import select
from datetime import datetime, timezone
import json
import hashlib
from typing import Optional


@mcp.tool(
    name="delta_build",
    annotations={
        "title": "Delta Build Detection",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def delta_build(params: DeltaBuildInput, ctx: Context) -> str:
    """Detect files changed since a given timestamp for incremental builds (FR-025).

    Queries the FileJournal to find all content files modified after the specified
    timestamp. This enables incremental builds in Docusaurus - only rebuild pages
    that have actually changed.

    Args:
        params (DeltaBuildInput): Validated input containing:
            - book_id (str): Book identifier
            - since (str): ISO 8601 timestamp (e.g., '2025-01-01T00:00:00Z')
            - include_content (bool): Include file content (default: False)
            - user_id (str | None): Include user's overlay changes

    Returns:
        str: JSON response with changed files list

    Example:
        ```
        # Get files changed since last build
        Input: {
          "book_id": "ai-native-python",
          "since": "2025-01-01T00:00:00Z"
        }
        Output: {
          "changed_count": 3,
          "since": "2025-01-01T00:00:00Z",
          "changed_files": [
            {
              "path": "content/01-Part/01-Chapter/01-lesson.md",
              "sha256": "abc123...",
              "last_modified": "2025-01-02T10:30:00Z",
              "namespace": "base"
            },
            ...
          ]
        }

        # Include content for direct processing
        Input: {
          "book_id": "ai-native-python",
          "since": "2025-01-01T00:00:00Z",
          "include_content": true
        }
        Output: {
          "changed_count": 1,
          "changed_files": [
            {
              "path": "content/01-Part/01-Chapter/01-lesson.md",
              "sha256": "abc123...",
              "last_modified": "2025-01-02T10:30:00Z",
              "content": "# Lesson 1\\n\\n..."
            }
          ]
        }

        # Include user overlay changes
        Input: {
          "book_id": "ai-native-python",
          "since": "2025-01-01T00:00:00Z",
          "user_id": "user123"
        }
        Output: {
          "changed_count": 2,
          "changed_files": [
            {"path": "...", "namespace": "base", ...},
            {"path": "...", "namespace": "overlay", ...}
          ]
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        # Parse the since timestamp
        since_dt = datetime.fromisoformat(params.since.replace('Z', '+00:00'))

        # Get operator for optional content reading
        op = get_operator()

        changed_files = []

        async with get_session() as session:
            # Query base content changes
            stmt = select(FileJournal).where(
                FileJournal.book_id == params.book_id,
                FileJournal.user_id == "__base__",
                FileJournal.last_written_at > since_dt
            ).order_by(FileJournal.last_written_at.desc())

            result = await session.execute(stmt)
            base_entries = result.scalars().all()

            for entry in base_entries:
                file_info = {
                    "path": entry.path,
                    "sha256": entry.sha256,
                    "last_modified": entry.last_written_at.isoformat() if entry.last_written_at else None,
                    "namespace": "base"
                }

                # Optionally include content
                if params.include_content:
                    try:
                        full_path = f"books/{params.book_id}/{entry.path}"
                        content_bytes = await op.read(full_path)
                        file_info["content"] = content_bytes.decode('utf-8')
                    except Exception:
                        file_info["content"] = None
                        file_info["error"] = "Could not read content"

                changed_files.append(file_info)

            # Query overlay changes if user_id provided
            if params.user_id:
                overlay_stmt = select(FileJournal).where(
                    FileJournal.book_id == params.book_id,
                    FileJournal.user_id == params.user_id,
                    FileJournal.last_written_at > since_dt
                ).order_by(FileJournal.last_written_at.desc())

                overlay_result = await session.execute(overlay_stmt)
                overlay_entries = overlay_result.scalars().all()

                for entry in overlay_entries:
                    file_info = {
                        "path": entry.path,
                        "sha256": entry.sha256,
                        "last_modified": entry.last_written_at.isoformat() if entry.last_written_at else None,
                        "namespace": "overlay",
                        "user_id": params.user_id
                    }

                    # Optionally include content
                    if params.include_content:
                        try:
                            full_path = f"books/{params.book_id}/users/{params.user_id}/{entry.path}"
                            content_bytes = await op.read(full_path)
                            file_info["content"] = content_bytes.decode('utf-8')
                        except Exception:
                            file_info["content"] = None
                            file_info["error"] = "Could not read content"

                    changed_files.append(file_info)

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.DELTA_BUILD,
            path=f"books/{params.book_id}/",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time,
            book_id=params.book_id,
            user_id=params.user_id
        )

        # Build response
        response = {
            "changed_count": len(changed_files),
            "since": params.since,
            "book_id": params.book_id,
            "changed_files": changed_files
        }

        if params.user_id:
            response["user_id"] = params.user_id
            response["base_changes"] = sum(1 for f in changed_files if f["namespace"] == "base")
            response["overlay_changes"] = sum(1 for f in changed_files if f["namespace"] == "overlay")

        return json.dumps(response, indent=2)

    except ValueError as e:
        # Invalid timestamp format
        await log_operation(
            operation=OperationType.DELTA_BUILD,
            path=f"books/{params.book_id}/",
            status=OperationStatus.ERROR,
            error_message=f"Invalid timestamp format: {str(e)}",
            book_id=params.book_id
        )
        return json.dumps({
            "error": f"Invalid timestamp format: {str(e)}",
            "expected_format": "ISO 8601 (e.g., 2025-01-01T00:00:00Z)"
        }, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.DELTA_BUILD,
            path=f"books/{params.book_id}/",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id
        )

        return f"Error in delta build: {type(e).__name__}: {str(e)}"


def compute_manifest_hash(entries: list[FileJournal]) -> str:
    """Compute deterministic manifest hash from FileJournal entries.

    Algorithm (from spec):
    1. Sort entries lexicographically by path
    2. Concatenate: "{path}:{sha256}\\n" for each entry
    3. Hash: SHA256 of concatenated string

    Args:
        entries: List of FileJournal entries (should be filtered to base only)

    Returns:
        SHA256 hex digest of manifest
    """
    # Sort by path for deterministic ordering
    sorted_entries = sorted(entries, key=lambda e: e.path)

    # Build manifest string
    manifest_lines = [f"{e.path}:{e.sha256}" for e in sorted_entries]
    manifest_string = "\n".join(manifest_lines)

    # Compute hash
    return hashlib.sha256(manifest_string.encode('utf-8')).hexdigest()


async def get_or_create_manifest_snapshot(
    session,
    book_id: str,
    manifest_hash: str,
    current_state: dict[str, str]
) -> None:
    """Store manifest snapshot if it doesn't exist.

    This enables future delta computations by persisting the file state
    at each unique manifest hash.

    Args:
        session: Database session
        book_id: Book identifier
        manifest_hash: Computed manifest hash
        current_state: Dict of {path: sha256} for all files
    """
    # Check if snapshot already exists
    stmt = select(ManifestSnapshot).where(
        ManifestSnapshot.manifest_hash == manifest_hash
    )
    result = await session.execute(stmt)
    existing = result.scalar_one_or_none()

    if existing is None:
        # Create new snapshot
        snapshot = ManifestSnapshot(
            manifest_hash=manifest_hash,
            book_id=book_id,
            file_count=len(current_state),
            content_json=json.dumps(current_state, sort_keys=True)
        )
        session.add(snapshot)
        await session.commit()


async def get_manifest_state(session, manifest_hash: str) -> Optional[dict[str, str]]:
    """Retrieve file state from a stored manifest snapshot.

    Args:
        session: Database session
        manifest_hash: The manifest hash to look up

    Returns:
        Dict of {path: sha256} if found, None if not found
    """
    stmt = select(ManifestSnapshot).where(
        ManifestSnapshot.manifest_hash == manifest_hash
    )
    result = await session.execute(stmt)
    snapshot = result.scalar_one_or_none()

    if snapshot is None:
        return None

    return json.loads(snapshot.content_json)


@mcp.tool(
    name="plan_build",
    annotations={
        "title": "Plan Build (Manifest Delta)",
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False
    }
)
async def plan_build(params: PlanBuildInput, ctx: Context) -> str:
    """Plan incremental build using manifest hash comparison (FR-025, FR-026, FR-027).

    Computes current manifest hash and compares to target to determine what changed.
    This is more reliable than timestamp-based detection for CI/CD workflows.

    **Key Feature**: Returns ONLY changed files when target_manifest_hash is provided,
    enabling minimal downloads for incremental CI/CD builds.

    Manifest Hash: Deterministic SHA256 of all base content paths and their hashes,
    sorted lexicographically. Same content state = same manifest hash.

    Args:
        params (PlanBuildInput): Validated input containing:
            - book_id (str): Book identifier
            - target_manifest_hash (str | None): Previous build's manifest hash

    Returns:
        str: JSON response per FR-026 format:
            {
                "status": "unchanged" | "changed",
                "manifest_hash": "current_hash",
                "files": [{"path": "...", "current_hash": "...", "target_hash": "..."}],
                "changed_count": N,
                "total_files": M
            }

    Example:
        ```
        # First build (no target hash) - returns all files
        Input: {"book_id": "ai-native-python"}
        Output: {
          "status": "changed",
          "manifest_hash": "abc123...",
          "files": [
            {"path": "content/01-Part/01-Chapter/01-lesson.md", "current_hash": "def456..."},
            ...
          ],
          "changed_count": 50,
          "total_files": 50
        }

        # Incremental build - returns ONLY changed files (FR-025 scenario 1)
        Input: {
          "book_id": "ai-native-python",
          "target_manifest_hash": "old123..."
        }
        Output: {
          "status": "changed",
          "manifest_hash": "abc123...",
          "files": [
            {"path": "content/01-Part/02-Chapter/03-lesson.md", "current_hash": "new789...", "target_hash": "old456..."}
          ],
          "changed_count": 1,
          "total_files": 50
        }

        # No changes since last build
        Input: {
          "book_id": "ai-native-python",
          "target_manifest_hash": "abc123..."
        }
        Output: {
          "status": "unchanged",
          "manifest_hash": "abc123...",
          "files": [],
          "changed_count": 0,
          "total_files": 50
        }
        ```
    """
    start_time = datetime.now(timezone.utc)

    try:
        async with get_session() as session:
            # Query all base content for this book
            stmt = select(FileJournal).where(
                FileJournal.book_id == params.book_id,
                FileJournal.user_id == "__base__"
            ).order_by(FileJournal.path)

            result = await session.execute(stmt)
            current_entries = list(result.scalars().all())

            # Compute current manifest hash
            current_manifest_hash = compute_manifest_hash(current_entries)

            # Build current state map
            current_state = {e.path: e.sha256 for e in current_entries}

            # Store current manifest snapshot for future delta computations
            await get_or_create_manifest_snapshot(
                session, params.book_id, current_manifest_hash, current_state
            )

            # Track if target manifest was not found (for warning in response)
            target_not_found = False

            # Determine status and files to return
            if params.target_manifest_hash is None:
                # First build - return all files
                status = "changed"
                files = [
                    {"path": path, "current_hash": sha256}
                    for path, sha256 in sorted(current_state.items())
                ]
                changed_count = len(files)

            elif params.target_manifest_hash == current_manifest_hash:
                # No changes - same manifest hash means identical state
                status = "unchanged"
                files = []
                changed_count = 0

            else:
                # Changed - compute actual delta by comparing to stored target state
                target_state = await get_manifest_state(session, params.target_manifest_hash)

                if target_state is None:
                    # Target manifest not found - client may have stale hash
                    # Return all files with a warning (fallback to full build)
                    target_not_found = True
                    status = "changed"
                    files = [
                        {"path": path, "current_hash": sha256, "target_hash": None}
                        for path, sha256 in sorted(current_state.items())
                    ]
                    changed_count = len(files)
                else:
                    # Compute actual delta: files that differ between current and target
                    files = []

                    # Find modified and new files
                    for path, current_hash in current_state.items():
                        target_hash = target_state.get(path)
                        if target_hash is None:
                            # New file (exists in current, not in target)
                            files.append({
                                "path": path,
                                "current_hash": current_hash,
                                "target_hash": None,
                                "change_type": "added"
                            })
                        elif target_hash != current_hash:
                            # Modified file (hash changed)
                            files.append({
                                "path": path,
                                "current_hash": current_hash,
                                "target_hash": target_hash,
                                "change_type": "modified"
                            })
                        # else: unchanged, don't include

                    # Find deleted files (exist in target, not in current)
                    for path, target_hash in target_state.items():
                        if path not in current_state:
                            files.append({
                                "path": path,
                                "current_hash": None,
                                "target_hash": target_hash,
                                "change_type": "deleted"
                            })

                    # Sort by path for consistent ordering
                    files.sort(key=lambda f: f["path"])
                    changed_count = len(files)
                    status = "changed" if changed_count > 0 else "unchanged"

        # Log success
        execution_time = int((datetime.now(timezone.utc) - start_time).total_seconds() * 1000)
        await log_operation(
            operation=OperationType.PLAN_BUILD,
            path=f"books/{params.book_id}/",
            status=OperationStatus.SUCCESS,
            execution_time_ms=execution_time,
            book_id=params.book_id
        )

        # Build response per FR-026
        response = {
            "status": status,
            "manifest_hash": current_manifest_hash,
            "files": files,
            "changed_count": changed_count,
            "total_files": len(current_entries),
            "book_id": params.book_id
        }

        if params.target_manifest_hash:
            response["target_manifest_hash"] = params.target_manifest_hash

        # Add warning if target manifest was not found
        if target_not_found:
            response["warning"] = (
                f"Target manifest '{params.target_manifest_hash}' not found in snapshot history. "
                "Returning all files. This may happen if the target was from before "
                "manifest snapshots were enabled, or from a different environment."
            )

        return json.dumps(response, indent=2)

    except Exception as e:
        # Log error
        await log_operation(
            operation=OperationType.PLAN_BUILD,
            path=f"books/{params.book_id}/",
            status=OperationStatus.ERROR,
            error_message=str(e),
            book_id=params.book_id
        )

        return f"Error in plan_build: {type(e).__name__}: {str(e)}"
