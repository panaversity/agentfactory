#!/usr/bin/env python3
"""Upload a single image and get the CDN URL back.

Simple script for authors to upload images during lesson writing.

Usage:
    # Basic upload (auto-generates filename from source)
    uv run python scripts/upload_image.py --image ./diagram.png --part 02 --chapter 05

    # With custom name
    uv run python scripts/upload_image.py --image ./diagram.png --part 02 --chapter 05 --name skill-architecture

    # Dry run (preview URL without uploading)
    uv run python scripts/upload_image.py --image ./diagram.png --part 02 --chapter 05 --dry-run

Output:
    ✓ Uploaded: https://pub-xxx.r2.dev/books/ai-native-dev/static/images/part-2/chapter-05/skill-architecture.png

    Ready to paste in markdown:
    ![skill-architecture](https://pub-xxx.r2.dev/books/ai-native-dev/static/images/part-2/chapter-05/skill-architecture.png)
"""

import argparse
import asyncio
import hashlib
import re
import sys
from datetime import datetime, timezone
from pathlib import Path

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from dotenv import load_dotenv
load_dotenv()


def sanitize_filename(name: str) -> str:
    """Sanitize filename for URL safety."""
    # Remove extension if present
    name = Path(name).stem
    # Convert to lowercase, replace spaces with hyphens
    name = name.lower().replace(' ', '-').replace('_', '-')
    # Remove non-alphanumeric except hyphens
    name = re.sub(r'[^a-z0-9\-]', '', name)
    # Collapse multiple hyphens
    name = re.sub(r'-+', '-', name)
    return name.strip('-')


async def upload_image(
    image_path: Path,
    part: str,
    chapter: str,
    name: str | None,
    book_id: str,
    dry_run: bool
) -> str | None:
    """Upload image to storage and return CDN URL."""

    # Validate image exists
    if not image_path.exists():
        print(f"ERROR: Image not found: {image_path}")
        return None

    # Get extension
    ext = image_path.suffix.lower()
    valid_extensions = {'.png', '.jpg', '.jpeg', '.gif', '.svg', '.webp'}
    if ext not in valid_extensions:
        print(f"ERROR: Invalid image type: {ext}")
        print(f"  Supported: {', '.join(valid_extensions)}")
        return None

    # Build filename
    if name:
        safe_name = sanitize_filename(name)
    else:
        safe_name = sanitize_filename(image_path.stem)

    filename = f"{safe_name}{ext}"

    # Build storage path: books/{book_id}/static/images/part-{N}/chapter-{NN}/{filename}
    # Normalize part/chapter numbers
    part_num = part.lstrip('0') or '0'
    chapter_num = chapter.zfill(2)

    storage_path = f"books/{book_id}/static/images/part-{part_num}/chapter-{chapter_num}/{filename}"

    # Get config
    try:
        from panaversity_fs.config import get_config
        config = get_config()
    except Exception as e:
        print(f"ERROR: Configuration failed: {e}")
        print("  Make sure .env is configured with storage settings")
        return None

    # Build CDN URL
    cdn_base = config.cdn_base_url
    if not cdn_base:
        print("ERROR: CDN_BASE_URL not configured in .env")
        return None

    cdn_url = f"{cdn_base}/{storage_path}"

    # Read file
    file_bytes = image_path.read_bytes()
    file_size = len(file_bytes)
    size_str = f"{file_size / 1024:.1f} KB" if file_size < 1024 * 1024 else f"{file_size / (1024*1024):.2f} MB"

    # Check size limit (10MB for direct upload)
    max_size = 10 * 1024 * 1024
    if file_size > max_size:
        print(f"ERROR: File too large: {size_str} (max 10MB)")
        return None

    # Compute hash
    content_hash = hashlib.sha256(file_bytes).hexdigest()

    if dry_run:
        print(f"\n[DRY RUN] Would upload:")
        print(f"  Source: {image_path}")
        print(f"  Size: {size_str}")
        print(f"  Path: {storage_path}")
        print(f"  URL: {cdn_url}")
        return cdn_url

    # Upload
    try:
        from panaversity_fs.storage import get_operator
        op = get_operator()

        print(f"Uploading {image_path.name} ({size_str})...")
        await op.write(storage_path, file_bytes)

        # Sync to FileJournal
        try:
            from panaversity_fs.database.connection import get_session
            from panaversity_fs.database.models import FileJournal

            # Journal path is relative (without books/{book_id}/)
            journal_path = f"static/images/part-{part_num}/chapter-{chapter_num}/{filename}"

            async with get_session() as session:
                entry = FileJournal(
                    book_id=book_id,
                    path=journal_path,
                    user_id="__base__",
                    sha256=content_hash,
                    last_written_at=datetime.now(timezone.utc),
                    storage_backend=config.storage_backend
                )
                await session.merge(entry)

        except Exception as e:
            print(f"  Warning: FileJournal sync failed: {e}")

        return cdn_url

    except Exception as e:
        print(f"ERROR: Upload failed: {e}")
        return None


def main():
    parser = argparse.ArgumentParser(
        description="Upload a single image and get the CDN URL",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        "--image", "-i",
        type=Path,
        required=True,
        help="Path to image file"
    )

    parser.add_argument(
        "--part", "-p",
        type=str,
        required=True,
        help="Part number (e.g., 02 or 2)"
    )

    parser.add_argument(
        "--chapter", "-c",
        type=str,
        required=True,
        help="Chapter number (e.g., 05 or 5)"
    )

    parser.add_argument(
        "--name", "-n",
        type=str,
        default=None,
        help="Custom filename (without extension). Default: use source filename"
    )

    parser.add_argument(
        "--book-id", "-b",
        type=str,
        default="ai-native-dev",
        help="Book identifier (default: ai-native-dev)"
    )

    parser.add_argument(
        "--dry-run", "-d",
        action="store_true",
        help="Preview URL without uploading"
    )

    args = parser.parse_args()

    # Run upload
    cdn_url = asyncio.run(upload_image(
        image_path=args.image,
        part=args.part,
        chapter=args.chapter,
        name=args.name,
        book_id=args.book_id,
        dry_run=args.dry_run
    ))

    if cdn_url:
        print(f"\n✓ {'Would upload to' if args.dry_run else 'Uploaded'}:")
        print(f"  {cdn_url}")
        print(f"\nReady to paste in markdown:")
        # Extract name for alt text
        filename = cdn_url.split('/')[-1]
        alt_text = Path(filename).stem
        print(f"  ![{alt_text}]({cdn_url})")
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
