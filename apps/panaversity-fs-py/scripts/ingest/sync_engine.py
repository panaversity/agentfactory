"""Sync engine for book source to PanaversityFS.

Handles the synchronization logic including:
- Computing deltas between source and storage
- Uploading new/changed files
- Handling conflicts
- Progress tracking
"""

import asyncio
import base64
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

from scripts.common.mcp_client import MCPClient, MCPToolError
from scripts.ingest.source_scanner import SourceFile, ScanResult, get_valid_files
from scripts.ingest.path_mapper import ContentType


@dataclass
class SyncAction:
    """A single sync action to perform.

    Attributes:
        source_file: The source file to sync
        action: Type of action (add, update, skip)
        reason: Why this action was chosen
        existing_hash: For updates, the current hash in storage (for conflict detection)
    """
    source_file: SourceFile
    action: str  # "add", "update", "skip"
    reason: Optional[str] = None
    existing_hash: Optional[str] = None


@dataclass
class SyncResult:
    """Result of a sync operation.

    Attributes:
        files_added: Number of files added
        files_updated: Number of files updated
        files_skipped: Number of files skipped (unchanged)
        files_failed: Number of files that failed
        bytes_transferred: Total bytes uploaded
        errors: List of error details
    """
    files_added: int = 0
    files_updated: int = 0
    files_skipped: int = 0
    files_failed: int = 0
    bytes_transferred: int = 0
    errors: list[dict] = field(default_factory=list)

    def add_success(self, action: str, size: int) -> None:
        """Record a successful sync."""
        if action == "add":
            self.files_added += 1
        elif action == "update":
            self.files_updated += 1
        self.bytes_transferred += size

    def add_skip(self) -> None:
        """Record a skipped file."""
        self.files_skipped += 1

    def add_failure(self, path: str, error: str) -> None:
        """Record a failed sync."""
        self.files_failed += 1
        self.errors.append({"path": path, "error": error})

    @property
    def total_processed(self) -> int:
        """Total files processed."""
        return self.files_added + self.files_updated + self.files_skipped + self.files_failed


def format_bytes(size: int) -> str:
    """Format bytes as human-readable string."""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024:
            return f"{size:.1f}{unit}"
        size /= 1024
    return f"{size:.1f}TB"


async def get_storage_hashes(
    client: MCPClient,
    book_id: str,
    paths: list[str],
    source_files: list[SourceFile],
    verbose: bool = False
) -> dict[str, Optional[str]]:
    """Get content hashes for existing files in storage.

    Args:
        client: MCP client instance
        book_id: Book identifier
        paths: Storage paths to check
        source_files: Corresponding source files (for content type detection)
        verbose: Print verbose output

    Returns:
        Dict mapping storage_path -> content_hash (or None if not found)
    """
    hashes: dict[str, Optional[str]] = {}

    # Create path -> source_file mapping
    path_to_source = {f.mapped.storage_path: f for f in source_files}

    for path in paths:
        source_file = path_to_source.get(path)

        # Query server for hash (works for both content/ and static/ paths in v1)
        try:
            result = await client.read_content(book_id, path)

            # For assets, read_content returns {"file_hash_sha256": "...", "exists": true/false}
            # For markdown, it returns full ContentMetadata with "file_hash_sha256" field
            if "exists" in result:
                # Asset path response
                if result["exists"]:
                    hashes[path] = result.get("file_hash_sha256")
                else:
                    hashes[path] = None  # Asset not in FileJournal yet
            else:
                # Markdown path response
                content_hash = result.get("file_hash_sha256")
                if content_hash:
                    hashes[path] = content_hash
                else:
                    hashes[path] = None
        except MCPToolError:
            # File doesn't exist in storage
            hashes[path] = None
        except Exception as e:
            if verbose:
                print(f"  Warning: Could not check {path}: {e}")
            hashes[path] = None

    return hashes


async def compute_sync_plan(
    client: MCPClient,
    book_id: str,
    scan_result: ScanResult,
    verbose: bool = False
) -> list[SyncAction]:
    """Compute the sync plan by comparing source with storage.

    Args:
        client: MCP client instance
        book_id: Book identifier
        scan_result: Result of source directory scan
        verbose: Print verbose output

    Returns:
        List of sync actions to perform
    """
    valid_files = get_valid_files(scan_result.files)

    if not valid_files:
        return []

    if verbose:
        print(f"Computing sync plan for {len(valid_files)} files...")

    # Get storage paths for all valid files
    storage_paths = [f.mapped.storage_path for f in valid_files if f.mapped.storage_path]

    # Batch check existing hashes (with concurrency limit)
    storage_hashes = await get_storage_hashes(client, book_id, storage_paths, valid_files, verbose)

    # Build sync plan
    actions: list[SyncAction] = []

    for source_file in valid_files:
        storage_path = source_file.mapped.storage_path
        if not storage_path:
            continue

        existing_hash = storage_hashes.get(storage_path)

        if existing_hash is None:
            # File doesn't exist in storage - add
            actions.append(SyncAction(
                source_file=source_file,
                action="add",
                reason="New file",
                existing_hash=None
            ))
        elif existing_hash != source_file.content_hash:
            # File exists but content changed - update
            actions.append(SyncAction(
                source_file=source_file,
                action="update",
                reason="Content changed",
                existing_hash=existing_hash
            ))
        else:
            # File unchanged - skip
            actions.append(SyncAction(
                source_file=source_file,
                action="skip",
                reason="No changes",
                existing_hash=existing_hash
            ))

    if verbose:
        adds = sum(1 for a in actions if a.action == "add")
        updates = sum(1 for a in actions if a.action == "update")
        skips = sum(1 for a in actions if a.action == "skip")
        print(f"Sync plan: {adds} add, {updates} update, {skips} skip")

    return actions


async def sync_file(
    client: MCPClient,
    book_id: str,
    source_file: SourceFile,
    action: str,
    existing_hash: Optional[str] = None,
    verbose: bool = False
) -> tuple[bool, int, Optional[str]]:
    """Sync a single file to PanaversityFS.

    Args:
        client: MCP client instance
        book_id: Book identifier
        source_file: Source file to sync
        action: Type of action ("add" or "update")
        existing_hash: For updates, the current hash in storage (for conflict detection)
        verbose: Print verbose output

    Returns:
        Tuple of (success, bytes_transferred, error_message)
    """
    import hashlib

    storage_path = source_file.mapped.storage_path
    if not storage_path:
        return False, 0, "No storage path"

    try:
        # Read source content
        if source_file.mapped.content_type == ContentType.ASSET:
            # Binary file - read and base64 encode
            with open(source_file.absolute_path, "rb") as f:
                binary_data = base64.b64encode(f.read()).decode("ascii")

            # Determine asset type from path
            # Valid MCP tool values: "images", "slides", "videos", "audio"
            asset_type = "images"  # default
            path_parts = source_file.relative_path.lower().split("/")
            for part in path_parts:
                if part in {"img", "image", "images"}:
                    asset_type = "images"
                    break
                elif part in {"slides", "videos", "audio"}:
                    asset_type = part
                    break

            # Upload asset
            result = await client.upload_asset(
                book_id=book_id,
                asset_type=asset_type,
                filename=source_file.absolute_path.name,
                binary_data=binary_data
            )

            # Server now tracks assets in FileJournal automatically
            # No need for local cache - hash is stored in PostgreSQL
        else:
            # Text file - read as UTF-8
            content = source_file.absolute_path.read_text(encoding="utf-8")

            # Write content (with expected_hash for updates to enable conflict detection)
            write_result = await client.write_content(
                book_id=book_id,
                path=storage_path,
                content=content,
                expected_hash=existing_hash  # Pass expected hash for updates
            )

            # Verify upload using the file_hash returned by write_content
            # This is more reliable than reading back, as it's the hash of what was actually stored
            returned_hash = write_result.get("file_hash")
            if not returned_hash:
                error_msg = "Upload verification failed: no file_hash in response"
                if verbose:
                    print(f"  FAILED: {source_file.relative_path} - {error_msg}")
                return False, 0, error_msg

            # Verify by reading back and checking the returned hash matches
            try:
                verify_result = await client.read_content(book_id, storage_path)
                stored_hash = verify_result.get("file_hash_sha256", "")

                if stored_hash != returned_hash:
                    error_msg = f"Upload verification failed: hash mismatch (write returned: {returned_hash[:16]}..., read returned: {stored_hash[:16]}...)"
                    if verbose:
                        print(f"  FAILED: {source_file.relative_path} - {error_msg}")
                    return False, 0, error_msg

            except Exception as verify_error:
                # Verification failed - treat upload as failed
                error_msg = f"Upload verification failed: {verify_error}"
                if verbose:
                    print(f"  FAILED: {source_file.relative_path} - {error_msg}")
                return False, 0, error_msg

        if verbose:
            action_verb = "Added" if action == "add" else "Updated"
            print(f"  {action_verb}: {source_file.relative_path} ({format_bytes(source_file.size_bytes)})")

        return True, source_file.size_bytes, None

    except MCPToolError as e:
        error_msg = str(e)
        if verbose:
            print(f"  FAILED: {source_file.relative_path} - {error_msg}")
        return False, 0, error_msg

    except Exception as e:
        error_msg = f"{type(e).__name__}: {e}"
        if verbose:
            print(f"  FAILED: {source_file.relative_path} - {error_msg}")
        return False, 0, error_msg


async def execute_sync_plan(
    client: MCPClient,
    book_id: str,
    actions: list[SyncAction],
    verbose: bool = False,
    concurrency: int = 5
) -> SyncResult:
    """Execute a sync plan.

    Args:
        client: MCP client instance
        book_id: Book identifier
        actions: List of sync actions to perform
        verbose: Print verbose output
        concurrency: Maximum concurrent uploads

    Returns:
        SyncResult with statistics
    """
    result = SyncResult()

    # Filter to only actionable items
    actionable = [a for a in actions if a.action in ("add", "update")]

    if not actionable:
        # Nothing to sync
        result.files_skipped = sum(1 for a in actions if a.action == "skip")
        return result

    total = len(actionable)

    if verbose:
        print(f"Syncing {total} files to PanaversityFS...")

    # Use semaphore to limit concurrency
    semaphore = asyncio.Semaphore(concurrency)

    async def sync_with_semaphore(sync_action: SyncAction, index: int) -> None:
        async with semaphore:
            if not verbose:
                # Progress indicator for non-verbose mode
                print(f"\rSyncing {index + 1}/{total}...", end="", flush=True)

            success, size, error = await sync_file(
                client, book_id, sync_action.source_file, sync_action.action, sync_action.existing_hash, verbose
            )

            if success:
                result.add_success(sync_action.action, size)
            else:
                result.add_failure(sync_action.source_file.relative_path, error or "Unknown error")

    # Execute all syncs with concurrency limit
    tasks = [
        sync_with_semaphore(action, i)
        for i, action in enumerate(actionable)
    ]
    await asyncio.gather(*tasks)

    if not verbose:
        print()  # New line after progress

    # Count skips
    result.files_skipped = sum(1 for a in actions if a.action == "skip")

    return result


async def sync_source_to_storage(
    client: MCPClient,
    book_id: str,
    scan_result: ScanResult,
    verbose: bool = False,
    dry_run: bool = False
) -> SyncResult:
    """Sync source directory to PanaversityFS storage.

    This is the main entry point for syncing. It computes the delta
    between source and storage, then uploads changed files.

    Args:
        client: MCP client instance
        book_id: Book identifier
        scan_result: Result of source directory scan
        verbose: Print verbose output
        dry_run: If True, compute plan but don't execute

    Returns:
        SyncResult with statistics
    """
    # Compute sync plan
    actions = await compute_sync_plan(client, book_id, scan_result, verbose)

    if dry_run:
        # Just report what would happen
        result = SyncResult()
        result.files_added = sum(1 for a in actions if a.action == "add")
        result.files_updated = sum(1 for a in actions if a.action == "update")
        result.files_skipped = sum(1 for a in actions if a.action == "skip")
        result.bytes_transferred = sum(
            a.source_file.size_bytes
            for a in actions
            if a.action in ("add", "update")
        )

        if verbose:
            print(f"Dry run: would sync {result.files_added + result.files_updated} files")
            for action in actions:
                if action.action != "skip":
                    print(f"  {action.action}: {action.source_file.relative_path}")

        return result

    # Execute sync
    return await execute_sync_plan(client, book_id, actions, verbose)
