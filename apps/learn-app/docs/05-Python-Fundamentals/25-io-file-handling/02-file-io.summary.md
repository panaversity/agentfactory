# File I/O Fundamentals with Context Managers Summary

### Core Concept

Context managers (`with` statement) guarantee file closure even when errors occur, preventing resource leaks and data corruption. File modes determine permissions: 'r' reads, 'w' creates/overwrites, 'a' appends.

### Key Mental Models

**Context Manager as Safety Net**: Without `with`, a crashed program leaves files unclosed—resources leak, data corrupts. With `with`, Python handles cleanup automatically, like a checkpoint that always executes.

**File Modes as Intent**: Choose your mode before opening. 'w' is destructive (deletes existing content), 'a' preserves it. This choice determines your file's final state.

### Critical Patterns

- `with open(filename, 'r', encoding='utf-8') as f:` → safe reading
- `with open(filename, 'w', encoding='utf-8') as f:` → destructive write (existing content deleted)
- `with open(filename, 'a', encoding='utf-8') as f:` → append without losing data
- `f.readlines()` or iterate `for line in f:` → memory-efficient line processing
- `f.write("\n")` → explicit newlines (unlike `print()` which adds them automatically)

### AI Collaboration Keys

Specify your intent to AI: "I want to add lines to an existing file without losing data—which mode?" Let AI handle syntax; you focus on intent and validation.

### Common Mistakes

- Opening file without `with` (resource leak if code crashes)
- Using 'w' mode when 'a' is intended (data loss)
- Forgetting `.strip()` removes `\n` from `readline()` output (unexpected blank lines)
- Missing exception handling for FileNotFoundError, PermissionError, IOError

### Connections

Builds on Lesson 1 (console interaction). Foundation for Lesson 3 (pathlib needs file operations). Enables Lesson 4 (reading/writing structured formats requires safe I/O). Essential for Lesson 5 capstone (note persistence).
