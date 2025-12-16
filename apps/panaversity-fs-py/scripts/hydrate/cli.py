"""CLI for hydrating Docusaurus builds from PanaversityFS.

This command downloads content from PanaversityFS to a local directory
for Docusaurus builds. It supports incremental builds by tracking a
manifest hash between builds.

Usage:
    hydrate-book --book-id ai-native-python
    hydrate-book --book-id ai-native-python --full-rebuild
    hydrate-book --book-id ai-native-python --verbose
"""

import asyncio
import os
import sys
from pathlib import Path

import click

from scripts.common.mcp_client import MCPClient, MCPConfig, MCPError
from scripts.hydrate.manifest import (
    ManifestFile,
    load_manifest,
    save_manifest,
    delete_manifest,
    get_default_manifest_path,
)
from scripts.hydrate.downloader import (
    download_all_content,
    download_delta,
    format_bytes,
)


def get_book_id_from_env() -> str | None:
    """Get book ID from environment variable."""
    return os.environ.get("PANAVERSITY_BOOK_ID")


@click.command("hydrate")
@click.option(
    "--book-id",
    envvar="PANAVERSITY_BOOK_ID",
    required=True,
    help="Book identifier (or set PANAVERSITY_BOOK_ID env var)"
)
@click.option(
    "--output-dir",
    type=click.Path(path_type=Path),
    default=Path(".docusaurus/content"),
    help="Output directory for downloaded content (default: .docusaurus/content)"
)
@click.option(
    "--manifest-file",
    type=click.Path(path_type=Path),
    default=None,
    help="Manifest file path (default: .panaversity/manifest.json)"
)
@click.option(
    "--full-rebuild",
    is_flag=True,
    default=False,
    help="Force full rebuild, ignoring manifest"
)
@click.option(
    "--verbose", "-v",
    is_flag=True,
    default=False,
    help="Verbose output"
)
def hydrate(
    book_id: str,
    output_dir: Path,
    manifest_file: Path | None,
    full_rebuild: bool,
    verbose: bool
):
    """Hydrate Docusaurus build with content from PanaversityFS.

    Downloads content from PanaversityFS storage to a local directory.
    Uses manifest-based delta detection to only download changed files
    on subsequent builds.

    Examples:

        # First build (downloads all content)
        hydrate-book --book-id ai-native-python

        # Subsequent builds (downloads only changes)
        hydrate-book --book-id ai-native-python

        # Force full rebuild
        hydrate-book --book-id ai-native-python --full-rebuild
    """
    # Run async main
    try:
        result = asyncio.run(_hydrate_async(
            book_id=book_id,
            output_dir=output_dir,
            manifest_file=manifest_file or get_default_manifest_path(),
            full_rebuild=full_rebuild,
            verbose=verbose
        ))
        sys.exit(0 if result else 1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        sys.exit(130)
    except MCPError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


async def _hydrate_async(
    book_id: str,
    output_dir: Path,
    manifest_file: Path,
    full_rebuild: bool,
    verbose: bool
) -> bool:
    """Async implementation of hydration."""

    # Load existing manifest (if not full rebuild)
    manifest: ManifestFile | None = None
    if not full_rebuild:
        manifest = load_manifest(manifest_file)
        if manifest and manifest.book_id != book_id:
            if verbose:
                print(f"Manifest book_id mismatch ({manifest.book_id} != {book_id}), treating as new build")
            manifest = None

    if verbose:
        if full_rebuild:
            print(f"Full rebuild requested for book: {book_id}")
        elif manifest:
            print(f"Incremental build for book: {book_id}")
            print(f"Previous manifest: {manifest.manifest_hash[:16]}... ({manifest.file_count} files)")
        else:
            print(f"First build for book: {book_id}")

    # Create output directory
    output_dir.mkdir(parents=True, exist_ok=True)

    # Connect to MCP server
    config = MCPConfig.from_env()
    if verbose:
        print(f"Connecting to: {config.base_url}")

    async with MCPClient(config) as client:
        if manifest and not full_rebuild:
            # Incremental build
            result, status, new_hash = await download_delta(
                client=client,
                book_id=book_id,
                target_manifest_hash=manifest.manifest_hash,
                output_dir=output_dir,
                verbose=verbose
            )

            if status == "unchanged":
                print(f"No changes. {manifest.file_count} files up to date.")
                return True

        else:
            # Full build (first build or --full-rebuild)
            if full_rebuild:
                # Clear manifest to force full download
                delete_manifest(manifest_file)

            result, new_hash = await download_all_content(
                client=client,
                book_id=book_id,
                output_dir=output_dir,
                verbose=verbose
            )

        # Check for errors
        if result.files_failed > 0:
            print(f"WARNING: {result.files_failed} files failed to download", file=sys.stderr)
            for error in result.errors[:5]:
                print(f"  - {error['path']}: {error['error']}", file=sys.stderr)
            if len(result.errors) > 5:
                print(f"  ... and {len(result.errors) - 5} more", file=sys.stderr)

        # Save new manifest
        new_manifest = ManifestFile.create(
            book_id=book_id,
            manifest_hash=new_hash,
            file_count=result.files_downloaded + (manifest.file_count if manifest else 0)
        )
        save_manifest(manifest_file, new_manifest)

        # Summary
        print(f"Downloaded {result.files_downloaded} files ({format_bytes(result.bytes_transferred)})")

        if result.files_failed > 0:
            return False

        return True


def main():
    """Entry point for CLI."""
    hydrate()


if __name__ == "__main__":
    main()
