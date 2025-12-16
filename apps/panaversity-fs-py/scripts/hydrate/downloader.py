"""File downloader for hydration.

Downloads files from PanaversityFS to local filesystem for Docusaurus builds.
Supports streaming downloads to minimize memory usage.
"""

import asyncio
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
import sys

from scripts.common.mcp_client import MCPClient, MCPToolError


@dataclass
class DownloadResult:
    """Result of a download operation.

    Attributes:
        files_downloaded: Number of files successfully downloaded
        files_failed: Number of files that failed to download
        bytes_transferred: Total bytes transferred
        errors: List of error details
    """
    files_downloaded: int = 0
    files_failed: int = 0
    bytes_transferred: int = 0
    errors: list[dict] = field(default_factory=list)

    def add_success(self, path: str, size: int) -> None:
        """Record a successful download."""
        self.files_downloaded += 1
        self.bytes_transferred += size

    def add_failure(self, path: str, error: str) -> None:
        """Record a failed download."""
        self.files_failed += 1
        self.errors.append({"path": path, "error": error})


def format_bytes(size: int) -> str:
    """Format bytes as human-readable string."""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024:
            return f"{size:.1f}{unit}"
        size /= 1024
    return f"{size:.1f}TB"


async def download_file(
    client: MCPClient,
    book_id: str,
    file_info: dict,
    output_dir: Path,
    verbose: bool = False
) -> tuple[bool, int, Optional[str]]:
    """Download a single file from PanaversityFS.

    Args:
        client: MCP client instance
        book_id: Book identifier
        file_info: File info dict with 'path' key
        output_dir: Output directory for downloaded file
        verbose: Print verbose output

    Returns:
        Tuple of (success: bool, bytes: int, error: Optional[str])
    """
    path = file_info["path"]

    try:
        # Read content from PanaversityFS
        result = await client.read_content(book_id, path)

        content = result.get("content", "")
        if content is None:
            return False, 0, "Empty content returned"

        # Determine output path
        output_path = output_dir / path

        # Create parent directories
        output_path.parent.mkdir(parents=True, exist_ok=True)

        # Write file
        output_path.write_text(content, encoding="utf-8")

        file_size = len(content.encode("utf-8"))

        if verbose:
            print(f"  Downloaded: {path} ({format_bytes(file_size)})")

        return True, file_size, None

    except MCPToolError as e:
        error_msg = str(e)
        if verbose:
            print(f"  FAILED: {path} - {error_msg}", file=sys.stderr)
        return False, 0, error_msg

    except Exception as e:
        error_msg = f"{type(e).__name__}: {e}"
        if verbose:
            print(f"  FAILED: {path} - {error_msg}", file=sys.stderr)
        return False, 0, error_msg


async def download_changed_files(
    client: MCPClient,
    book_id: str,
    files: list[dict],
    output_dir: Path,
    verbose: bool = False,
    concurrency: int = 5
) -> DownloadResult:
    """Download multiple files from PanaversityFS.

    Args:
        client: MCP client instance
        book_id: Book identifier
        files: List of file info dicts from plan_build response
        output_dir: Output directory for downloaded files
        verbose: Print verbose output
        concurrency: Maximum concurrent downloads

    Returns:
        DownloadResult with statistics
    """
    result = DownloadResult()

    if not files:
        return result

    total_files = len(files)

    if verbose:
        print(f"Downloading {total_files} files to {output_dir}...")

    # Use semaphore to limit concurrency
    semaphore = asyncio.Semaphore(concurrency)

    async def download_with_semaphore(file_info: dict, index: int) -> None:
        async with semaphore:
            if not verbose:
                # Progress indicator for non-verbose mode
                progress = f"\rDownloading {index + 1}/{total_files}..."
                print(progress, end="", flush=True)

            success, size, error = await download_file(
                client, book_id, file_info, output_dir, verbose
            )

            if success:
                result.add_success(file_info["path"], size)
            else:
                result.add_failure(file_info["path"], error or "Unknown error")

    # Download all files with concurrency limit
    tasks = [
        download_with_semaphore(file_info, i)
        for i, file_info in enumerate(files)
    ]
    await asyncio.gather(*tasks)

    if not verbose:
        print()  # New line after progress

    return result


async def download_all_content(
    client: MCPClient,
    book_id: str,
    output_dir: Path,
    verbose: bool = False
) -> tuple[DownloadResult, str]:
    """Download all content from a book.

    This is used for first builds when no manifest exists.

    Args:
        client: MCP client instance
        book_id: Book identifier
        output_dir: Output directory
        verbose: Print verbose output

    Returns:
        Tuple of (DownloadResult, manifest_hash)
    """
    # Get all files via plan_build without target hash
    plan = await client.plan_build(book_id)

    files = plan.get("files", [])
    manifest_hash = plan.get("manifest_hash", "")

    if verbose:
        print(f"First build: {len(files)} files to download")

    result = await download_changed_files(
        client, book_id, files, output_dir, verbose
    )

    return result, manifest_hash


async def download_delta(
    client: MCPClient,
    book_id: str,
    target_manifest_hash: str,
    output_dir: Path,
    verbose: bool = False
) -> tuple[DownloadResult, str, str]:
    """Download only changed files since last build.

    Args:
        client: MCP client instance
        book_id: Book identifier
        target_manifest_hash: Manifest hash from previous build
        output_dir: Output directory
        verbose: Print verbose output

    Returns:
        Tuple of (DownloadResult, status, new_manifest_hash)
        status is "unchanged" or "changed"
    """
    # Get delta via plan_build with target hash
    plan = await client.plan_build(book_id, target_manifest_hash)

    status = plan.get("status", "changed")
    files = plan.get("files", [])
    manifest_hash = plan.get("manifest_hash", "")

    if status == "unchanged":
        if verbose:
            print("No changes detected. Skipping download.")
        return DownloadResult(), status, manifest_hash

    # Handle deleted files
    deleted_files = [f for f in files if f.get("change_type") == "deleted"]
    for file_info in deleted_files:
        path = file_info["path"]
        file_path = output_dir / path
        if file_path.exists():
            file_path.unlink()
            if verbose:
                print(f"  Deleted: {path}")

    # Download added and modified files
    download_files = [f for f in files if f.get("change_type") != "deleted"]

    if verbose:
        print(f"Delta build: {len(download_files)} files changed, {len(deleted_files)} deleted")

    result = await download_changed_files(
        client, book_id, download_files, output_dir, verbose
    )

    return result, status, manifest_hash
