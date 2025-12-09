"""CLI for ingesting book source to PanaversityFS.

This command syncs content from a book source directory to PanaversityFS
storage. It handles path mapping, hash-based change detection, and
incremental uploads.

Usage:
    ingest-book --book-id ai-native-python --source-dir ./book-source/docs
    ingest-book --book-id ai-native-python --source-dir ./book-source/docs --dry-run
    ingest-book --book-id ai-native-python --source-dir ./book-source/docs --verbose
"""

import asyncio
import os
import sys
from pathlib import Path

import click

from scripts.common.mcp_client import MCPClient, MCPConfig, MCPError
from scripts.ingest.source_scanner import scan_source_directory
from scripts.ingest.sync_engine import (
    sync_source_to_storage,
    format_bytes,
)


def get_book_id_from_env() -> str | None:
    """Get book ID from environment variable."""
    return os.environ.get("PANAVERSITY_BOOK_ID")


def get_source_dir_from_env() -> Path | None:
    """Get source directory from environment variable."""
    path = os.environ.get("PANAVERSITY_SOURCE_DIR")
    return Path(path) if path else None


@click.command("ingest")
@click.option(
    "--book-id",
    envvar="PANAVERSITY_BOOK_ID",
    required=True,
    help="Book identifier (or set PANAVERSITY_BOOK_ID env var)"
)
@click.option(
    "--source-dir",
    type=click.Path(exists=True, file_okay=False, path_type=Path),
    envvar="PANAVERSITY_SOURCE_DIR",
    required=True,
    help="Source directory containing book content (e.g., ./book-source/docs)"
)
@click.option(
    "--dry-run",
    is_flag=True,
    default=False,
    help="Show what would be synced without making changes"
)
@click.option(
    "--verbose", "-v",
    is_flag=True,
    default=False,
    help="Verbose output"
)
def ingest(
    book_id: str,
    source_dir: Path,
    dry_run: bool,
    verbose: bool
):
    """Ingest book source content to PanaversityFS.

    Scans the source directory for content files (markdown and assets),
    compares with PanaversityFS storage, and uploads any new or changed files.

    Examples:

        # First sync (uploads all content)
        ingest-book --book-id ai-native-python --source-dir ./book-source/docs

        # Subsequent syncs (uploads only changes)
        ingest-book --book-id ai-native-python --source-dir ./book-source/docs

        # Preview what would be synced
        ingest-book --book-id ai-native-python --source-dir ./book-source/docs --dry-run
    """
    # Run async main
    try:
        result = asyncio.run(_ingest_async(
            book_id=book_id,
            source_dir=source_dir,
            dry_run=dry_run,
            verbose=verbose
        ))
        sys.exit(0 if result else 1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)
    except MCPError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except ValueError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


async def _ingest_async(
    book_id: str,
    source_dir: Path,
    dry_run: bool,
    verbose: bool
) -> bool:
    """Async implementation of ingestion."""

    if verbose:
        print(f"Book ID: {book_id}")
        print(f"Source directory: {source_dir}")
        if dry_run:
            print("Mode: DRY RUN (no changes will be made)")

    # Step 1: Scan source directory
    print("Scanning source directory...")
    scan_result = scan_source_directory(source_dir, verbose)

    if scan_result.valid_count == 0:
        print("No valid content files found.")
        if scan_result.invalid_count > 0:
            print(f"({scan_result.invalid_count} files skipped due to invalid paths)")
        return True

    print(f"Found {scan_result.valid_count} files ({format_bytes(scan_result.total_bytes)})")
    if scan_result.invalid_count > 0:
        print(f"({scan_result.invalid_count} files skipped)")

    # Step 2: Connect to MCP server and sync
    config = MCPConfig.from_env()
    if verbose:
        print(f"Connecting to: {config.base_url}")

    async with MCPClient(config) as client:
        # Sync to storage
        result = await sync_source_to_storage(
            client=client,
            book_id=book_id,
            scan_result=scan_result,
            verbose=verbose,
            dry_run=dry_run
        )

    # Step 3: Report results
    if dry_run:
        print(f"\nDry run summary:")
        print(f"  Would add: {result.files_added} files")
        print(f"  Would update: {result.files_updated} files")
        print(f"  Unchanged: {result.files_skipped} files")
        print(f"  Total transfer: {format_bytes(result.bytes_transferred)}")
    else:
        print(f"\nSync complete:")
        print(f"  Added: {result.files_added} files")
        print(f"  Updated: {result.files_updated} files")
        print(f"  Unchanged: {result.files_skipped} files")
        print(f"  Transferred: {format_bytes(result.bytes_transferred)}")

        if result.files_failed > 0:
            print(f"\nWARNING: {result.files_failed} files failed", file=sys.stderr)
            for error in result.errors[:5]:
                print(f"  - {error['path']}: {error['error']}", file=sys.stderr)
            if len(result.errors) > 5:
                print(f"  ... and {len(result.errors) - 5} more", file=sys.stderr)
            return False

    return True


def main():
    """Entry point for CLI."""
    ingest()


if __name__ == "__main__":
    main()
