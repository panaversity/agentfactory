# Cross-Platform Path Handling with pathlib Summary

### Core Concept

Path objects replace string-based path construction, automatically handling OS-specific separators (backslash on Windows, forward slash on Unix). The `/` operator joins path segments intuitively and works identically across all operating systems.

### Key Mental Models

**Paths as Objects, Not Strings**: A Path object isn't just text—it's an entity with methods (`.exists()`, `.is_file()`, `.mkdir()`). Treat paths as first-class Python objects with capabilities, not just formatted strings.

**Defensive File Operations**: Always check before operating. `if path.exists():` prevents crashes. `path.mkdir(parents=True, exist_ok=True)` safely creates nested structures even if partially present.

### Critical Patterns

- `from pathlib import Path` → import once
- `Path("data") / "notes" / "file.txt"` → intuitive path joining with `/` operator
- `path.name`, `path.stem`, `path.suffix` → extract components safely
- `path.exists()`, `path.is_file()`, `path.is_dir()` → defensive checks
- `path.mkdir(parents=True, exist_ok=True)` → safe recursive directory creation
- `path.glob("*.txt")`, `path.iterdir()` → find files by pattern or list contents
- `path.resolve()` and `.is_relative_to()` → prevent path traversal attacks

### AI Collaboration Keys

Ask AI about cross-platform compatibility: "Show me code that works on Windows, Mac, and Linux without modification." AI handles syntax; you focus on making code portable and secure.

### Common Mistakes

- Hardcoding backslashes or forward slashes (breaks on other OSs)
- Assuming files exist without checking (FileNotFoundError crashes)
- Using `mkdir()` without `parents=True` (fails if parent missing)
- Forgetting `glob()` returns Path objects, not strings (confusion with string methods)

### Connections

Builds on Lesson 2 (file I/O needs safe paths). Enables Lesson 4 (structured formats require path navigation). Critical for Lesson 5 capstone (organizing notes by category across all platforms).
