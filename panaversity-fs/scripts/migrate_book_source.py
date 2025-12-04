#!/usr/bin/env python3
"""Migration script: book-source → PanaversityFS storage.

This script migrates content from the Docusaurus book-source structure
to ANY configured storage backend (fs, s3, supabase).

Backends:
- fs, s3: Uses OpenDAL for unified interface
- supabase: Uses Supabase Python SDK directly (OpenDAL Python lacks support)

Source structure (book-source/):
    docs/
        {NN-Part}/
            README.md
            {NN-Chapter}/
                README.md
                {NN-lesson}.md
    static/
        img/
        slides/

Target structure (storage backend):
    books/{book-id}/
        content/                    # Maps to docs/
            {NN-Part}/
                README.md
                {NN-Chapter}/
                    README.md
                    {NN-lesson}.md
        static/                     # Maps to static/
            images/                 # Renamed from img/
            slides/

Environment variables (from .env):
    PANAVERSITY_STORAGE_BACKEND    - fs, s3, or supabase

    For fs:
        PANAVERSITY_STORAGE_ROOT   - Root directory path

    For s3:
        PANAVERSITY_S3_BUCKET, PANAVERSITY_S3_ENDPOINT, etc.

    For supabase:
        PANAVERSITY_SUPABASE_URL, PANAVERSITY_SUPABASE_SERVICE_ROLE_KEY, etc.

Usage:
    # Dry run (preview changes)
    uv run python scripts/migrate_book_source.py --dry-run

    # Full migration with URL rewriting (recommended for cloud backends)
    uv run python scripts/migrate_book_source.py --rewrite-urls

    # Migrate content only
    uv run python scripts/migrate_book_source.py --content-only

    # Migrate assets only
    uv run python scripts/migrate_book_source.py --assets-only

    # With verbose output
    uv run python scripts/migrate_book_source.py --verbose

    # Resume from a specific path (for large migrations)
    uv run python scripts/migrate_book_source.py --resume-from "chapter-14"
"""

import argparse
import asyncio
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from dotenv import load_dotenv

# Load .env file
load_dotenv()


@dataclass
class MigrationStats:
    """Track migration statistics."""
    content_uploaded: int = 0
    content_skipped: int = 0
    content_failed: int = 0
    assets_uploaded: int = 0
    assets_skipped: int = 0
    assets_failed: int = 0
    urls_rewritten: int = 0
    bytes_uploaded: int = 0
    errors: list = field(default_factory=list)

    def summary(self) -> str:
        mb = self.bytes_uploaded / (1024 * 1024)
        return f"""
Migration Summary
=================
Content:
  Uploaded: {self.content_uploaded}
  Skipped:  {self.content_skipped}
  Failed:   {self.content_failed}

Assets:
  Uploaded: {self.assets_uploaded}
  Skipped:  {self.assets_skipped}
  Failed:   {self.assets_failed}

URLs Rewritten: {self.urls_rewritten}
Total Size: {mb:.2f} MB
Errors: {len(self.errors)}
"""


@dataclass
class MigrationConfig:
    """Migration configuration."""
    source_dir: Path
    book_id: str
    dry_run: bool = False
    content_only: bool = False
    assets_only: bool = False
    rewrite_urls: bool = False
    verbose: bool = False
    resume_from: str | None = None

    @property
    def docs_path(self) -> Path:
        return self.source_dir / "docs"

    @property
    def static_path(self) -> Path:
        return self.source_dir / "static"


class Migrator:
    """Migrate book-source to storage backend.

    Uses OpenDAL for fs/s3 backends and Supabase SDK for supabase backend.
    """

    def __init__(self, config: MigrationConfig):
        self.config = config
        self.stats = MigrationStats()
        self.operator = None  # OpenDAL operator (for fs/s3)
        self.supabase_client = None  # Supabase client (for supabase)
        self.storage_config = None
        self.resumed = not bool(config.resume_from)

        # Directory mapping (img → images for ADR-0018 compliance)
        self.dir_mapping = {
            "img": "images"
        }

    async def connect(self):
        """Initialize storage client based on backend type."""
        try:
            from panaversity_fs.config import get_config
            self.storage_config = get_config()
        except Exception as e:
            if self.config.dry_run:
                print(f"[DRY RUN] Config warning: {e}")
                print("  Continuing with limited dry run (no CDN URL preview)")
                return
            else:
                print(f"ERROR: Configuration failed: {e}")
                sys.exit(1)

        if self.config.dry_run:
            print(f"[DRY RUN] Would connect to storage backend: {self.storage_config.storage_backend}")
            return

        backend = self.storage_config.storage_backend

        if backend == "supabase":
            # Use Supabase SDK directly (OpenDAL Python lacks supabase support)
            try:
                from supabase import create_client, Client
                self.supabase_client = create_client(
                    self.storage_config.supabase_url,
                    self.storage_config.supabase_service_role_key
                )
                print(f"✓ Connected to storage: supabase (via SDK)")
            except ImportError:
                print("ERROR: supabase package not installed. Run: uv add supabase")
                sys.exit(1)
            except Exception as e:
                print(f"ERROR: Failed to connect to Supabase: {e}")
                sys.exit(1)
        else:
            # Use OpenDAL for fs and s3 backends
            try:
                from panaversity_fs.storage import get_operator
                self.operator = get_operator()
                print(f"✓ Connected to storage: {backend} (via OpenDAL)")
            except Exception as e:
                print(f"ERROR: Failed to connect to storage: {e}")
                sys.exit(1)

    def should_skip(self, path: Path) -> bool:
        """Check if file should be skipped."""
        if any(part.startswith('.') for part in path.parts):
            return True
        if 'node_modules' in path.parts:
            return True
        return False

    def check_resume(self, path: Path) -> bool:
        """Check if we should process this file (resume logic)."""
        if self.resumed:
            return True
        if self.config.resume_from and self.config.resume_from in str(path):
            self.resumed = True
            print(f"Resuming from: {path}")
            return True
        return False

    def get_storage_path(self, local_path: Path, content_type: str) -> str:
        """Convert local path to storage path.

        Args:
            local_path: Local file path
            content_type: 'content' or 'static'

        Returns:
            Storage path like 'books/ai-native-dev/content/...'
        """
        if content_type == "content":
            rel_path = local_path.relative_to(self.config.docs_path)
            return f"books/{self.config.book_id}/content/{rel_path}"
        elif content_type == "static":
            rel_path = local_path.relative_to(self.config.static_path)
            # Apply directory mapping (img → images)
            parts = list(rel_path.parts)
            if parts and parts[0] in self.dir_mapping:
                parts[0] = self.dir_mapping[parts[0]]
            mapped_path = Path(*parts)
            return f"books/{self.config.book_id}/static/{mapped_path}"
        else:
            raise ValueError(f"Unknown content type: {content_type}")

    def get_cdn_base_url(self) -> str:
        """Get CDN base URL for the configured backend."""
        if not self.storage_config:
            return ""

        if self.storage_config.storage_backend == "supabase":
            return f"{self.storage_config.supabase_url}/storage/v1/object/public/{self.storage_config.supabase_bucket}"
        elif self.storage_config.storage_backend == "s3":
            # Cloudflare R2 / S3 public URL pattern
            if self.storage_config.cdn_base_url:
                return self.storage_config.cdn_base_url
            return f"https://{self.storage_config.s3_bucket}.r2.dev"
        else:
            # Filesystem - use configured CDN or empty
            return self.storage_config.cdn_base_url or ""

    def rewrite_asset_urls(self, content: str) -> tuple[str, int]:
        """Rewrite local asset URLs to CDN URLs.

        Handles TWO patterns:

        1. Markdown image syntax:
           - ![alt](/img/...) → ![alt]({cdn}/books/{book_id}/static/images/...)
           - ![alt](/slides/...) → ![alt]({cdn}/books/{book_id}/static/slides/...)

        2. YAML frontmatter slides:
           - source: "slides/chapter-05-slides.pdf"
           → source: "{cdn}/books/{book_id}/static/slides/chapter-05-slides.pdf"

        Skips external URLs (http://, https://).

        Returns:
            Tuple of (modified content, number of URLs rewritten)
        """
        count = 0
        cdn_base = self.get_cdn_base_url()

        if not cdn_base:
            return content, 0

        # Pattern 1: Markdown images ![alt](url)
        def replace_markdown_image(match):
            nonlocal count
            alt_text = match.group(1)
            url = match.group(2)

            if url.startswith('http://') or url.startswith('https://'):
                return match.group(0)

            new_url = None

            if url.startswith('/img/'):
                rel_path = url[5:]
                new_url = f"{cdn_base}/books/{self.config.book_id}/static/images/{rel_path}"
            elif url.startswith('/slides/'):
                rel_path = url[8:]
                new_url = f"{cdn_base}/books/{self.config.book_id}/static/slides/{rel_path}"
            elif url.startswith('./images/') or url.startswith('./img/'):
                rel_path = url.split('/', 2)[-1] if '/' in url else url
                new_url = f"{cdn_base}/books/{self.config.book_id}/static/images/{rel_path}"
            elif '/' not in url and url.endswith(('.png', '.jpg', '.jpeg', '.gif', '.svg', '.webp')):
                new_url = f"{cdn_base}/books/{self.config.book_id}/static/images/{url}"

            if new_url:
                count += 1
                return f"![{alt_text}]({new_url})"

            return match.group(0)

        # Pattern 2: YAML frontmatter source: "slides/..."
        def replace_frontmatter_slides(match):
            nonlocal count
            prefix = match.group(1)  # 'source: "' or "source: '"
            path = match.group(2)    # slides/chapter-05-slides.pdf
            suffix = match.group(3)  # closing quote

            if path.startswith('http://') or path.startswith('https://'):
                return match.group(0)

            if path.startswith('slides/'):
                new_url = f"{cdn_base}/books/{self.config.book_id}/static/{path}"
                count += 1
                return f'{prefix}{new_url}{suffix}'

            return match.group(0)

        # Apply markdown image rewriting
        md_pattern = r'!\[([^\]]*)\]\(([^)]+)\)'
        new_content = re.sub(md_pattern, replace_markdown_image, content)

        # Apply frontmatter slides rewriting
        # Matches: source: "slides/..." or source: 'slides/...'
        fm_pattern = r'(source:\s*["\'])([^"\']+)(["\'])'
        new_content = re.sub(fm_pattern, replace_frontmatter_slides, new_content)

        return new_content, count

    def _get_content_type(self, file_path: Path) -> str:
        """Get MIME type for file based on extension."""
        suffix = file_path.suffix.lower()
        mime_types = {
            '.md': 'text/markdown',
            '.mdx': 'text/markdown',
            '.txt': 'text/plain',
            '.json': 'application/json',
            '.png': 'image/png',
            '.jpg': 'image/jpeg',
            '.jpeg': 'image/jpeg',
            '.gif': 'image/gif',
            '.svg': 'image/svg+xml',
            '.webp': 'image/webp',
            '.ico': 'image/x-icon',
            '.pdf': 'application/pdf',
            '.pptx': 'application/vnd.openxmlformats-officedocument.presentationml.presentation',
            '.ppt': 'application/vnd.ms-powerpoint',
            '.mp4': 'video/mp4',
            '.webm': 'video/webm',
            '.mov': 'video/quicktime',
            '.mp3': 'audio/mpeg',
            '.wav': 'audio/wav',
            '.ogg': 'audio/ogg',
        }
        return mime_types.get(suffix, 'application/octet-stream')

    async def upload_file(self, local_path: Path, storage_path: str, is_text: bool = False) -> bool:
        """Upload a single file to storage.

        Args:
            local_path: Local file path
            storage_path: Target path in storage
            is_text: Whether file is text (for URL rewriting)

        Returns:
            True if successful, False otherwise
        """
        try:
            if is_text:
                content = local_path.read_text(encoding='utf-8')

                if self.config.rewrite_urls:
                    content, url_count = self.rewrite_asset_urls(content)
                    self.stats.urls_rewritten += url_count
                    if url_count > 0 and self.config.verbose:
                        print(f"    Rewrote {url_count} URLs in {local_path.name}")

                file_bytes = content.encode('utf-8')
            else:
                file_bytes = local_path.read_bytes()

            size = len(file_bytes)
            size_str = f"{size / 1024:.1f} KB" if size < 1024 * 1024 else f"{size / (1024*1024):.2f} MB"

            if self.config.dry_run:
                print(f"  [DRY RUN] {storage_path} ({size_str})")
                self.stats.bytes_uploaded += size
                return True

            # Upload based on backend type
            if self.supabase_client:
                # Supabase SDK upload
                content_type = self._get_content_type(local_path)
                bucket = self.storage_config.supabase_bucket

                # Supabase upsert mode to overwrite existing files
                self.supabase_client.storage.from_(bucket).upload(
                    path=storage_path,
                    file=file_bytes,
                    file_options={"content-type": content_type, "upsert": "true"}
                )
            else:
                # OpenDAL upload (for fs/s3)
                await self.operator.write(storage_path, file_bytes)

            self.stats.bytes_uploaded += size

            if self.config.verbose:
                print(f"  ✓ {storage_path} ({size_str})")

            return True

        except Exception as e:
            self.stats.errors.append(f"{storage_path}: {e}")
            if self.config.verbose:
                print(f"  ✗ {storage_path}: {e}")
            return False

    async def migrate_content(self):
        """Migrate all markdown content from docs/."""
        print("\n=== Migrating Content ===")
        if self.config.rewrite_urls:
            print("  URL rewriting: ENABLED")

        if not self.config.docs_path.exists():
            print(f"  ⚠ Source docs directory not found: {self.config.docs_path}")
            return

        md_files = list(self.config.docs_path.rglob("*.md"))
        mdx_files = list(self.config.docs_path.rglob("*.mdx"))
        all_files = sorted(md_files + mdx_files)

        total = len(all_files)
        print(f"  Found {total} markdown files")

        for i, md_file in enumerate(all_files):
            if self.should_skip(md_file):
                continue

            if not self.check_resume(md_file):
                self.stats.content_skipped += 1
                continue

            storage_path = self.get_storage_path(md_file, "content")

            if not self.config.verbose:
                print(f"\r  Uploading: {i+1}/{total}", end="", flush=True)

            if await self.upload_file(md_file, storage_path, is_text=True):
                self.stats.content_uploaded += 1
            else:
                self.stats.content_failed += 1

        if not self.config.verbose:
            print()

        print(f"  ✓ Uploaded: {self.stats.content_uploaded}")
        if self.stats.content_failed:
            print(f"  ✗ Failed: {self.stats.content_failed}")
        if self.config.rewrite_urls and self.stats.urls_rewritten > 0:
            print(f"  ↻ URLs rewritten: {self.stats.urls_rewritten}")

    async def migrate_assets(self):
        """Migrate all static assets from static/."""
        print("\n=== Migrating Assets ===")

        if not self.config.static_path.exists():
            print(f"  ⚠ Source static directory not found: {self.config.static_path}")
            return

        asset_extensions = {
            '.png', '.jpg', '.jpeg', '.gif', '.svg', '.webp', '.ico',
            '.pdf', '.pptx', '.ppt',
            '.mp4', '.webm', '.mov',
            '.mp3', '.wav', '.ogg'
        }

        all_files = []
        for ext in asset_extensions:
            all_files.extend(self.config.static_path.rglob(f"*{ext}"))
        all_files = sorted(all_files)

        total = len(all_files)
        print(f"  Found {total} asset files")

        for i, asset_file in enumerate(all_files):
            if self.should_skip(asset_file):
                continue

            if not self.check_resume(asset_file):
                self.stats.assets_skipped += 1
                continue

            storage_path = self.get_storage_path(asset_file, "static")

            if not self.config.verbose:
                print(f"\r  Uploading: {i+1}/{total}", end="", flush=True)

            if await self.upload_file(asset_file, storage_path, is_text=False):
                self.stats.assets_uploaded += 1
            else:
                self.stats.assets_failed += 1

        if not self.config.verbose:
            print()

        print(f"  ✓ Uploaded: {self.stats.assets_uploaded}")
        if self.stats.assets_failed:
            print(f"  ✗ Failed: {self.stats.assets_failed}")

    async def run(self):
        """Run the migration."""
        await self.connect()

        backend = self.storage_config.storage_backend if self.storage_config else "unknown"
        sdk_info = "Supabase SDK" if self.supabase_client else "OpenDAL"

        print(f"""
{'='*60}
PanaversityFS Migration (ADR-0018)
{'='*60}
Source:       {self.config.source_dir}
Backend:      {backend} (via {sdk_info})
Book ID:      {self.config.book_id}
Mode:         {'DRY RUN' if self.config.dry_run else 'LIVE'}
Content:      {'Yes' if not self.config.assets_only else 'No'}
Assets:       {'Yes' if not self.config.content_only else 'No'}
Rewrite URLs: {'Yes' if self.config.rewrite_urls else 'No'}
Resume:       {self.config.resume_from or 'From beginning'}
{'='*60}
""")

        if not self.config.assets_only:
            await self.migrate_content()

        if not self.config.content_only:
            await self.migrate_assets()

        print(self.stats.summary())

        if self.stats.errors:
            print("\nErrors (first 10):")
            for error in self.stats.errors[:10]:
                print(f"  - {error}")
            if len(self.stats.errors) > 10:
                print(f"  ... and {len(self.stats.errors) - 10} more")

        # Print CDN URL info
        cdn_base = self.get_cdn_base_url()
        if cdn_base and not self.config.dry_run:
            print(f"""
Next Steps
==========
1. Access content via:
   {cdn_base}/books/{self.config.book_id}/content/...

2. Access assets via:
   {cdn_base}/books/{self.config.book_id}/static/images/...
   {cdn_base}/books/{self.config.book_id}/static/slides/...
""")

        return self.stats


def main():
    parser = argparse.ArgumentParser(
        description="Migrate book-source to storage backend (via OpenDAL)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        "--source",
        type=Path,
        default=Path(__file__).parent.parent.parent / "book-source",
        help="Source book-source directory (default: ../book-source)"
    )

    parser.add_argument(
        "--book-id",
        type=str,
        default="ai-native-dev",
        help="Book identifier (default: ai-native-dev)"
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Preview changes without uploading"
    )

    parser.add_argument(
        "--content-only",
        action="store_true",
        help="Only upload markdown content (skip assets)"
    )

    parser.add_argument(
        "--assets-only",
        action="store_true",
        help="Only upload assets (skip content)"
    )

    parser.add_argument(
        "--rewrite-urls",
        action="store_true",
        help="Rewrite local image URLs to CDN URLs"
    )

    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Show detailed output"
    )

    parser.add_argument(
        "--resume-from",
        type=str,
        help="Resume from files containing this string"
    )

    args = parser.parse_args()

    if not args.source.exists():
        print(f"ERROR: Source directory not found: {args.source}")
        sys.exit(1)

    config = MigrationConfig(
        source_dir=args.source,
        book_id=args.book_id,
        dry_run=args.dry_run,
        content_only=args.content_only,
        assets_only=args.assets_only,
        rewrite_urls=args.rewrite_urls,
        verbose=args.verbose,
        resume_from=args.resume_from,
    )

    migrator = Migrator(config)
    stats = asyncio.run(migrator.run())

    if stats.content_failed or stats.assets_failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
