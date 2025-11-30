"""
Document crawler for RoboLearn docs.

Discovers all lesson MD files in the docs directory structure.
Extracts module and chapter info from path conventions.
"""

import os
import re
from pathlib import Path
from dataclasses import dataclass
from typing import Iterator

from rich.console import Console

console = Console()


@dataclass
class DiscoveredFile:
    """A discovered markdown file with extracted path info."""
    absolute_path: Path
    relative_path: str
    module: str          # ros2, gazebo, isaac, vla
    module_number: int   # 1, 2, 3, 4
    chapter: int
    lesson: int | None   # None for README files
    filename: str
    is_readme: bool


class DocsCrawler:
    """
    Crawls docs directory to discover all lesson markdown files.

    Directory structure convention:
    docs/
    ├── module-1-ros2/
    │   ├── README.md                    # Module overview
    │   ├── chapter-1-physical-ai/
    │   │   ├── README.md                # Chapter overview
    │   │   ├── 01-digital-to-physical.md
    │   │   ├── 02-why-humanoids.md
    │   │   └── ...
    """

    # Module name mapping
    MODULE_MAP = {
        "module-1-ros2": ("ros2", 1),
        "module-2-simulation": ("gazebo", 2),  # Gazebo is primary simulator
        "module-3-isaac": ("isaac", 3),
        "module-4-vla": ("vla", 4),
    }

    # Pattern to extract chapter number: chapter-{N}-{name}
    CHAPTER_PATTERN = re.compile(r"chapter-(\d+)-")

    # Pattern to extract lesson number: {NN}-{name}.md
    LESSON_PATTERN = re.compile(r"^(\d+)-.*\.md$")

    def __init__(self, docs_path: str | Path):
        """
        Initialize crawler with docs directory path.

        Args:
            docs_path: Path to docs directory (e.g., robolearn-interface/docs)
        """
        self.docs_path = Path(docs_path).resolve()
        if not self.docs_path.exists():
            raise FileNotFoundError(f"Docs path does not exist: {self.docs_path}")
        if not self.docs_path.is_dir():
            raise NotADirectoryError(f"Docs path is not a directory: {self.docs_path}")

    def discover(self, include_readmes: bool = True) -> Iterator[DiscoveredFile]:
        """
        Discover all markdown files in docs directory.

        Args:
            include_readmes: Whether to include README.md files (module/chapter overviews)

        Yields:
            DiscoveredFile for each discovered markdown file
        """
        console.print(f"[blue]Crawling docs at: {self.docs_path}[/blue]")

        for module_dir in sorted(self.docs_path.iterdir()):
            if not module_dir.is_dir():
                continue

            # Skip non-module directories
            if module_dir.name not in self.MODULE_MAP:
                # Check for standalone files like preface-agent-native.md
                if module_dir.suffix == ".md":
                    console.print(f"[yellow]Skipping standalone file: {module_dir.name}[/yellow]")
                continue

            module_name, module_number = self.MODULE_MAP[module_dir.name]
            console.print(f"[green]Found module: {module_dir.name} -> {module_name}[/green]")

            # Module README
            module_readme = module_dir / "README.md"
            if include_readmes and module_readme.exists():
                yield DiscoveredFile(
                    absolute_path=module_readme,
                    relative_path=str(module_readme.relative_to(self.docs_path)),
                    module=module_name,
                    module_number=module_number,
                    chapter=0,  # Module-level
                    lesson=None,
                    filename="README.md",
                    is_readme=True,
                )

            # Iterate chapters
            for chapter_dir in sorted(module_dir.iterdir()):
                if not chapter_dir.is_dir():
                    continue

                chapter_match = self.CHAPTER_PATTERN.match(chapter_dir.name)
                if not chapter_match:
                    console.print(f"[yellow]Skipping non-chapter dir: {chapter_dir.name}[/yellow]")
                    continue

                chapter_num = int(chapter_match.group(1))

                # Chapter README
                chapter_readme = chapter_dir / "README.md"
                if include_readmes and chapter_readme.exists():
                    yield DiscoveredFile(
                        absolute_path=chapter_readme,
                        relative_path=str(chapter_readme.relative_to(self.docs_path)),
                        module=module_name,
                        module_number=module_number,
                        chapter=chapter_num,
                        lesson=None,
                        filename="README.md",
                        is_readme=True,
                    )

                # Iterate lesson files
                for lesson_file in sorted(chapter_dir.iterdir()):
                    if not lesson_file.is_file():
                        continue
                    if lesson_file.suffix != ".md":
                        continue
                    if lesson_file.name == "README.md":
                        continue  # Already handled

                    lesson_match = self.LESSON_PATTERN.match(lesson_file.name)
                    lesson_num = int(lesson_match.group(1)) if lesson_match else 0

                    yield DiscoveredFile(
                        absolute_path=lesson_file,
                        relative_path=str(lesson_file.relative_to(self.docs_path)),
                        module=module_name,
                        module_number=module_number,
                        chapter=chapter_num,
                        lesson=lesson_num,
                        filename=lesson_file.name,
                        is_readme=False,
                    )

    def count(self, include_readmes: bool = True) -> dict:
        """
        Count discovered files by type.

        Returns:
            Dict with counts: {modules, chapters, lessons, readmes, total}
        """
        modules = set()
        chapters = set()
        lessons = 0
        readmes = 0

        for file in self.discover(include_readmes=include_readmes):
            modules.add(file.module)
            if file.chapter > 0:
                chapters.add((file.module, file.chapter))
            if file.is_readme:
                readmes += 1
            else:
                lessons += 1

        return {
            "modules": len(modules),
            "chapters": len(chapters),
            "lessons": lessons,
            "readmes": readmes,
            "total": lessons + readmes,
        }
