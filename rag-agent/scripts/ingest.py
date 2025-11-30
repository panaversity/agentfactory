#!/usr/bin/env python3
"""
Production-grade ingestion CLI for RoboLearn RAG.

Features:
- Incremental updates (only re-embed changed files)
- Full re-indexing when needed
- State persistence for change detection
- Detailed progress reporting

Usage:
    python scripts/ingest.py crawl              # Discover files
    python scripts/ingest.py ingest             # Incremental update (default)
    python scripts/ingest.py ingest --full      # Full re-index
    python scripts/ingest.py ingest --recreate  # Delete collection and re-index
    python scripts/ingest.py status             # Show what would be updated
    python scripts/ingest.py info               # Show collection info
"""

import sys
from pathlib import Path

# Add rag-agent to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

import typer
from rich.console import Console
from rich.table import Table
from rich.panel import Panel

from core.config import get_settings
from rag.ingestion.crawler import DocsCrawler
from rag.ingestion.parser import LessonParser
from rag.ingestion.chunker import SectionChunker
from rag.ingestion.embedder import OpenAIEmbedder
from rag.ingestion.uploader import QdrantUploader
from models.document import compute_file_hash

console = Console()
app = typer.Typer(help="RoboLearn RAG Ingestion CLI - Production Grade")


@app.command()
def crawl(
    docs_path: str = typer.Option(None, help="Path to docs directory"),
    include_readmes: bool = typer.Option(True, help="Include README files"),
):
    """
    Discover and count markdown files in docs directory.
    """
    settings = get_settings()
    path = docs_path or settings.docs_path

    console.print(Panel(f"[bold]Crawling docs at: {path}[/bold]", title="RoboLearn Crawler"))

    try:
        crawler = DocsCrawler(path)
        counts = crawler.count(include_readmes=include_readmes)

        table = Table(title="Discovery Results")
        table.add_column("Metric", style="cyan")
        table.add_column("Count", style="green", justify="right")

        table.add_row("Modules", str(counts["modules"]))
        table.add_row("Chapters", str(counts["chapters"]))
        table.add_row("Lessons", str(counts["lessons"]))
        table.add_row("READMEs", str(counts["readmes"]))
        table.add_row("Total Files", str(counts["total"]))

        console.print(table)

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def status(
    docs_path: str = typer.Option(None, help="Path to docs directory"),
):
    """
    Show what files would be updated (without making changes).
    """
    settings = get_settings()
    path = docs_path or settings.docs_path

    console.print(Panel("[bold]Checking for changes...[/bold]", title="Ingestion Status"))

    try:
        # Load current state
        uploader = QdrantUploader()
        state = uploader.load_state()

        # Compute current file hashes
        crawler = DocsCrawler(path)
        current_files = {}
        for file in crawler.discover():
            file_hash = compute_file_hash(str(file.absolute_path))
            current_files[file.relative_path] = file_hash

        # Compare
        new_files, modified_files, deleted_files = state.get_changed_files(current_files)

        # Display results
        table = Table(title="Change Detection Results")
        table.add_column("Category", style="cyan")
        table.add_column("Count", style="green", justify="right")
        table.add_column("Action", style="yellow")

        table.add_row("New files", str(len(new_files)), "Will be indexed")
        table.add_row("Modified files", str(len(modified_files)), "Will be re-indexed")
        table.add_row("Deleted files", str(len(deleted_files)), "Will be removed")
        table.add_row("Unchanged files", str(len(current_files) - len(new_files) - len(modified_files)), "Skipped")

        console.print(table)

        if new_files:
            console.print("\n[green]New files:[/green]")
            for f in new_files[:10]:
                console.print(f"  + {f}")
            if len(new_files) > 10:
                console.print(f"  ... and {len(new_files) - 10} more")

        if modified_files:
            console.print("\n[yellow]Modified files:[/yellow]")
            for f in modified_files[:10]:
                console.print(f"  ~ {f}")
            if len(modified_files) > 10:
                console.print(f"  ... and {len(modified_files) - 10} more")

        if deleted_files:
            console.print("\n[red]Deleted files:[/red]")
            for f in deleted_files[:10]:
                console.print(f"  - {f}")
            if len(deleted_files) > 10:
                console.print(f"  ... and {len(deleted_files) - 10} more")

        # Summary
        total_changes = len(new_files) + len(modified_files) + len(deleted_files)
        if total_changes == 0:
            console.print("\n[green]No changes detected. Index is up to date.[/green]")
        else:
            console.print(f"\n[yellow]Run 'python scripts/ingest.py ingest' to apply {total_changes} changes[/yellow]")

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        import traceback
        traceback.print_exc()
        raise typer.Exit(1)


@app.command()
def ingest(
    docs_path: str = typer.Option(None, help="Path to docs directory"),
    book_id: str = typer.Option(None, help="Book identifier"),
    recreate: bool = typer.Option(False, help="Recreate collection (deletes ALL data)"),
    full: bool = typer.Option(False, help="Full re-index (ignores state, keeps collection)"),
    include_readmes: bool = typer.Option(True, help="Include README files"),
    dry_run: bool = typer.Option(False, help="Parse and chunk only, don't embed or upload"),
):
    """
    Run ingestion pipeline with incremental updates.

    By default, only processes changed files (detected via content hash).
    Use --full to re-index everything without deleting the collection.
    Use --recreate to delete and rebuild from scratch.
    """
    settings = get_settings()
    path = docs_path or settings.docs_path
    book = book_id or settings.book_id

    mode = "RECREATE" if recreate else ("FULL" if full else "INCREMENTAL")

    console.print(Panel(
        f"[bold]Ingesting docs for book: {book}[/bold]\n"
        f"Source: {path}\n"
        f"Collection: {settings.collection_name}\n"
        f"Mode: {mode}",
        title="RoboLearn Production Ingestion"
    ))

    try:
        uploader = QdrantUploader()

        # Step 1: Ensure collection exists
        console.print("\n[bold cyan]Step 1/6: Ensuring collection...[/bold cyan]")
        uploader.ensure_collection(recreate=recreate)

        # Step 2: Crawl and compute file hashes
        console.print("\n[bold cyan]Step 2/6: Crawling docs and computing hashes...[/bold cyan]")
        crawler = DocsCrawler(path)
        all_files = list(crawler.discover(include_readmes=include_readmes))

        current_files = {}
        for file in all_files:
            file_hash = compute_file_hash(str(file.absolute_path))
            current_files[file.relative_path] = file_hash

        console.print(f"  Found {len(all_files)} files")

        # Step 3: Determine what to process
        console.print("\n[bold cyan]Step 3/6: Detecting changes...[/bold cyan]")

        if recreate or full:
            # Process all files
            files_to_process = [f for f in all_files]
            files_to_delete = []
            console.print(f"  Processing all {len(files_to_process)} files ({mode} mode)")
        else:
            # Incremental: only process changed files
            state = uploader.load_state()
            new_files, modified_files, deleted_files = state.get_changed_files(current_files)

            # Get file objects for new/modified
            files_to_process = [
                f for f in all_files
                if f.relative_path in new_files or f.relative_path in modified_files
            ]
            files_to_delete = deleted_files

            console.print(f"  New: {len(new_files)}, Modified: {len(modified_files)}, Deleted: {len(deleted_files)}")
            console.print(f"  Processing {len(files_to_process)} files, skipping {len(all_files) - len(files_to_process)} unchanged")

        if not files_to_process and not files_to_delete:
            console.print("\n[green]No changes detected. Index is up to date.[/green]")
            return

        # Step 4: Delete old chunks for modified/deleted files
        if files_to_delete or (not recreate and not full):
            console.print("\n[bold cyan]Step 4/6: Cleaning up old chunks...[/bold cyan]")
            state = uploader.load_state()

            # Get chunks to delete
            to_delete = files_to_delete.copy()
            for f in files_to_process:
                if f.relative_path in state.records:
                    to_delete.append(f.relative_path)

            chunk_ids_to_delete = state.get_chunks_to_delete(to_delete)
            if chunk_ids_to_delete:
                uploader.delete_chunks(chunk_ids_to_delete)
                console.print(f"  Deleted {len(chunk_ids_to_delete)} old chunks")
            else:
                console.print("  No old chunks to delete")

        # Step 5: Parse and chunk
        console.print("\n[bold cyan]Step 5/6: Parsing and chunking...[/bold cyan]")
        parser = LessonParser()
        chunker = SectionChunker(book_id=book)

        chunks = []
        source_files = {}

        for file in files_to_process:
            try:
                lesson = parser.parse(file)
                lesson_chunks = list(chunker.chunk(lesson))
                chunks.extend(lesson_chunks)
                source_files[file.relative_path] = current_files[file.relative_path]
            except Exception as e:
                console.print(f"  [yellow]Warning: Failed to process {file.relative_path}: {e}[/yellow]")

        console.print(f"  Created {len(chunks)} chunks from {len(files_to_process)} files")

        # Show chunk distribution
        module_counts = {}
        for chunk in chunks:
            m = chunk.metadata.module
            module_counts[m] = module_counts.get(m, 0) + 1

        if module_counts:
            table = Table(title="Chunks by Module")
            table.add_column("Module", style="cyan")
            table.add_column("Chunks", style="green", justify="right")
            for module, count in sorted(module_counts.items()):
                table.add_row(module, str(count))
            console.print(table)

        if dry_run:
            console.print("\n[yellow]Dry run complete. Skipping embedding and upload.[/yellow]")
            return

        # Step 6: Embed and upload
        console.print("\n[bold cyan]Step 6/6: Embedding and uploading...[/bold cyan]")

        if chunks:
            embedder = OpenAIEmbedder()
            embedded_chunks = embedder.embed_chunks(chunks)
            console.print(f"  Embedded {len(embedded_chunks)} chunks")

            uploaded = uploader.upload(embedded_chunks, source_files=source_files)
            console.print(f"  Uploaded {uploaded} points")

        # Summary
        info = uploader.get_collection_info()
        state = uploader.load_state()

        console.print(Panel(
            f"[green]Ingestion complete![/green]\n\n"
            f"Collection: {info.get('name')}\n"
            f"Total points: {info.get('points_count')}\n"
            f"Status: {info.get('status')}\n"
            f"Files tracked: {len(state.records)}\n"
            f"Mode: {mode}",
            title="Success"
        ))

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        import traceback
        traceback.print_exc()
        raise typer.Exit(1)


@app.command()
def info():
    """
    Show Qdrant collection and ingestion state information.
    """
    settings = get_settings()

    console.print(Panel(f"[bold]Collection: {settings.collection_name}[/bold]", title="Qdrant Info"))

    try:
        uploader = QdrantUploader()

        # Collection info
        info = uploader.get_collection_info()

        if "error" in info:
            console.print(f"[red]Error: {info['error']}[/red]")
            return

        table = Table(title="Collection Stats")
        table.add_column("Metric", style="cyan")
        table.add_column("Value", style="green")

        table.add_row("Name", info.get("name", "N/A"))
        table.add_row("Points Count", str(info.get("points_count", "N/A")))
        table.add_row("Status", info.get("status", "N/A"))

        console.print(table)

        # State info
        state = uploader.load_state()

        table2 = Table(title="Ingestion State")
        table2.add_column("Metric", style="cyan")
        table2.add_column("Value", style="green")

        table2.add_row("Book ID", state.book_id)
        table2.add_row("Files Tracked", str(len(state.records)))
        table2.add_row("Total Chunks", str(state.total_chunks))
        table2.add_row("Last Update", str(state.last_incremental_update or "Never"))

        console.print(table2)

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)


@app.command()
def delete_book(
    book_id: str = typer.Argument(..., help="Book ID to delete"),
    confirm: bool = typer.Option(False, "--yes", "-y", help="Skip confirmation"),
):
    """
    Delete all points for a specific book.
    """
    if not confirm:
        confirm = typer.confirm(f"Delete all points for book '{book_id}'?")
        if not confirm:
            console.print("[yellow]Cancelled[/yellow]")
            return

    try:
        uploader = QdrantUploader()
        deleted = uploader.delete_book(book_id)
        console.print(f"[green]Deleted approximately {deleted} points for book '{book_id}'[/green]")

    except Exception as e:
        console.print(f"[red]Error: {e}[/red]")
        raise typer.Exit(1)


if __name__ == "__main__":
    app()
