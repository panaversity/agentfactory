"""
Lesson file parser for RoboLearn docs.

Parses frontmatter and extracts content from markdown files.
Computes file hash for change detection.
"""

import frontmatter
from pathlib import Path
from typing import Literal

from rich.console import Console

from models.document import (
    LessonFile,
    ModuleName,
    ProficiencyLevel,
    TeachingLayer,
    HardwareTier,
    compute_file_hash,
)
from rag.ingestion.crawler import DiscoveredFile

console = Console()


class LessonParser:
    """
    Parses markdown lesson files, extracting frontmatter and content.
    Computes file hash for incremental update detection.
    """

    # Default values if frontmatter is missing
    DEFAULTS = {
        "proficiency_level": "A2",
        "layer": "L1",
        "hardware_tier": 1,
    }

    def parse(self, discovered: DiscoveredFile) -> LessonFile:
        """
        Parse a discovered file into a LessonFile model.

        Args:
            discovered: DiscoveredFile from crawler

        Returns:
            LessonFile with parsed frontmatter, content, and file hash
        """
        file_path = discovered.absolute_path

        # Compute file hash for change detection
        file_hash = compute_file_hash(str(file_path))

        # Read and parse frontmatter
        post = frontmatter.load(file_path)

        # Extract frontmatter values with defaults
        title = post.get("title", discovered.filename)
        description = post.get("description", None)
        doc_id = post.get("id", None)  # Docusaurus doc ID for URL generation
        proficiency = self._normalize_proficiency(post.get("proficiency_level", self.DEFAULTS["proficiency_level"]))
        layer = self._normalize_layer(post.get("layer", self.DEFAULTS["layer"]))
        hardware_tier = self._normalize_tier(post.get("hardware_tier", self.DEFAULTS["hardware_tier"]))

        # Use discovered file info for module/chapter/lesson
        # (more reliable than frontmatter which may vary)
        lesson_num = discovered.lesson if discovered.lesson is not None else 0

        return LessonFile(
            file_path=str(file_path),
            relative_path=discovered.relative_path,
            file_hash=file_hash,
            title=title,
            description=description,  # Extract description from frontmatter
            doc_id=doc_id,  # Docusaurus doc ID
            module=discovered.module,  # type: ignore (validated by crawler)
            chapter=discovered.chapter,
            lesson=lesson_num,
            hardware_tier=hardware_tier,  # type: ignore
            proficiency_level=proficiency,  # type: ignore
            layer=layer,  # type: ignore
            raw_content=post.content,
            sections=[],  # Will be populated by chunker
        )

    def _normalize_proficiency(self, value: str | None) -> ProficiencyLevel:
        """Normalize proficiency level to valid enum value."""
        if value is None:
            return "A2"

        value = str(value).upper().strip()

        # Handle quoted values
        value = value.strip('"').strip("'")

        # Handle range values like "A2-B1" - take the lower bound
        if "-" in value:
            value = value.split("-")[0]

        valid = {"A1", "A2", "B1", "B2", "C1", "C2"}
        if value in valid:
            return value  # type: ignore

        # Try to extract from common formats
        if "beginner" in value.lower():
            return "A2"
        if "intermediate" in value.lower():
            return "B1"
        if "advanced" in value.lower():
            return "C1"

        console.print(f"[yellow]Unknown proficiency '{value}', defaulting to A2[/yellow]")
        return "A2"

    def _normalize_layer(self, value: str | None) -> TeachingLayer:
        """Normalize teaching layer to valid enum value."""
        if value is None:
            return "L1"

        value = str(value).upper().strip()
        value = value.strip('"').strip("'")

        valid = {"L1", "L2", "L3", "L4"}
        if value in valid:
            return value  # type: ignore

        # Try numeric
        if value in {"1", "2", "3", "4"}:
            return f"L{value}"  # type: ignore

        console.print(f"[yellow]Unknown layer '{value}', defaulting to L1[/yellow]")
        return "L1"

    def _normalize_tier(self, value: int | str | None) -> HardwareTier:
        """Normalize hardware tier to valid integer."""
        if value is None:
            return 1

        try:
            tier = int(value)
            if 1 <= tier <= 4:
                return tier  # type: ignore
        except (ValueError, TypeError):
            pass

        console.print(f"[yellow]Invalid hardware tier '{value}', defaulting to 1[/yellow]")
        return 1
