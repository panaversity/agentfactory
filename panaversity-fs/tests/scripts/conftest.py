"""Test fixtures for script tests."""

import tempfile
from pathlib import Path
from typing import Generator

import pytest


@pytest.fixture
def temp_dir() -> Generator[Path, None, None]:
    """Create a temporary directory for tests."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield Path(tmpdir)


@pytest.fixture
def sample_book_structure(temp_dir: Path) -> Path:
    """Create a sample book source structure for testing.

    Creates:
        Part-01/
            Chapter-01/
                01-introduction.md
                02-getting-started.md
                02-getting-started.summary.md
                img/
                    diagram.png
            Chapter-02/
                01-basics.md
                README.md  (should be skipped)
        Part-02/
            Chapter-01/
                01-advanced.md
    """
    # Part-01/Chapter-01
    ch1 = temp_dir / "Part-01" / "Chapter-01"
    ch1.mkdir(parents=True)

    (ch1 / "01-introduction.md").write_text("# Introduction\n\nWelcome to the course.")
    (ch1 / "02-getting-started.md").write_text("# Getting Started\n\nLet's begin.")
    (ch1 / "02-getting-started.summary.md").write_text("# Summary\n\nKey points from getting started.")

    # Assets
    img_dir = ch1 / "img"
    img_dir.mkdir()
    # Create a tiny PNG (1x1 pixel transparent)
    png_data = bytes([
        0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A,  # PNG signature
        0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,  # IHDR chunk
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
        0x08, 0x06, 0x00, 0x00, 0x00, 0x1F, 0x15, 0xC4,
        0x89, 0x00, 0x00, 0x00, 0x0A, 0x49, 0x44, 0x41,  # IDAT chunk
        0x54, 0x78, 0x9C, 0x63, 0x00, 0x01, 0x00, 0x00,
        0x05, 0x00, 0x01, 0x0D, 0x0A, 0x2D, 0xB4, 0x00,
        0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44, 0xAE,  # IEND chunk
        0x42, 0x60, 0x82
    ])
    (img_dir / "diagram.png").write_bytes(png_data)

    # Part-01/Chapter-02
    ch2 = temp_dir / "Part-01" / "Chapter-02"
    ch2.mkdir(parents=True)
    (ch2 / "01-basics.md").write_text("# Basics\n\nFoundational concepts.")
    (ch2 / "README.md").write_text("# Chapter 2 README\n\nThis should be skipped.")

    # Part-02/Chapter-01
    p2ch1 = temp_dir / "Part-02" / "Chapter-01"
    p2ch1.mkdir(parents=True)
    (p2ch1 / "01-advanced.md").write_text("# Advanced Topics\n\nDeep dive.")

    return temp_dir


@pytest.fixture
def sample_paths() -> list[dict]:
    """Sample source paths for testing path mapping."""
    return [
        # Valid paths
        {
            "source": "Part-01/Chapter-01/01-introduction.md",
            "expected": "content/01-Part/01-Chapter/01-introduction.md",
            "valid": True,
            "content_type": "markdown"
        },
        {
            "source": "Part-01/Chapter-02/02-getting-started.summary.md",
            "expected": "content/01-Part/02-Chapter/02-getting-started.summary.md",
            "valid": True,
            "content_type": "summary"
        },
        {
            "source": "Part-02/Chapter-01/01-advanced.md",
            "expected": "content/02-Part/01-Chapter/01-advanced.md",
            "valid": True,
            "content_type": "markdown"
        },
        {
            "source": "Part-01/Chapter-01/img/diagram.png",
            "expected": "static/img/diagram.png",
            "valid": True,
            "content_type": "asset"
        },
        # Invalid paths
        {
            "source": "Part-01/Chapter-01/README.md",
            "expected": None,
            "valid": False,
            "content_type": "readme"
        },
        {
            "source": "invalid-path.md",
            "expected": None,
            "valid": False,
            "content_type": "markdown"
        },
        {
            "source": "Part-01/01-orphan.md",
            "expected": None,
            "valid": False,
            "content_type": "markdown"
        },
    ]


@pytest.fixture
def manifest_data() -> dict:
    """Sample manifest data for testing."""
    return {
        "book_id": "test-book",
        "manifest_hash": "abc123def456",
        "timestamp": "2025-01-01T00:00:00Z",
        "file_count": 10
    }
