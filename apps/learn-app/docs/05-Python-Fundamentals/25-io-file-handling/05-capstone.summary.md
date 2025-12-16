# Capstone - Note-Taking App Summary

### Core Concept

Integrate all Chapter 26 concepts (console I/O, file safety, pathlib organization, JSON persistence) into a production-quality CLI application with CRUD operations, menu loops, and comprehensive error handling.

### Key Mental Models

**Separation of Concerns**: Menu loop handles UI/input validation; CRUD functions handle persistence; data structure defines what gets stored. Each layer has one responsibility, making code testable and maintainable.

**File-Per-Note Architecture**: One JSON file per note with unique ID allows independent creation/deletion/search. Directory hierarchy (categories) organizes files without database complexity.

### Critical Patterns

- Menu loop: `while True` → display options → accept input with validation → route to function → continue loop
- CRUD functions: `get_all_notes()`, `save_note(note)`, `search_notes(keyword)`, `delete_note(id)`
- Path safety: `BASE_DIR.resolve()` + `.is_relative_to()` prevents directory traversal
- Error recovery: Try/except around file operations, JSON parsing, user input
- Data structure: Dict with id, title, body, category, created, modified fields

### AI Collaboration Keys

Ask AI about architecture before coding: "Should I store all notes in one file or one per note? What are tradeoffs?" Understand design trade-offs, then iterate with AI on implementation.

### Common Mistakes

- Loading all notes repeatedly (performance issue—could cache in memory)
- Missing error handling when JSON corrupted (crash instead of graceful warning)
- Not validating user input before saving (empty notes, invalid categories)
- Forgetting to create directories before writing files
- Path traversal vulnerabilities (not using `.resolve()` validation)

### Connections

Synthesis of Lessons 1-4: Input validation (1), file safety (2), cross-platform paths (3), JSON persistence (4). Demonstrates Layer 4 thinking—orchestrating components into complete application. Foundation for future features: timestamps (datetime), export-to-CSV, undo/redo history.
