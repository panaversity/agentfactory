### Core Concept
File paths are like directions—**absolute paths** are complete addresses that work from anywhere (start with `/` or `~`), while **relative paths** are directions from your current location (just folder names). Understanding this distinction lets you supervise AI navigation safely and verify it's operating in the right place.

### Key Mental Models
- **Addresses vs. Directions**: Absolute paths = complete address from a fixed starting point; Relative paths = directions from where you currently stand
- **Location Awareness**: Always know where you are (`pwd`) before using relative paths—they only work if the target folder exists in your current location
- **Safety Through Verification**: Verify location before any file operation to prevent mistakes (wrong folder deletions, misplaced files)

### Critical Patterns
- `cd /path/to/folder` — navigate using absolute path (complete address, works from anywhere)
- `cd folder-name` — navigate using relative path (must be in correct location first)
- `cd ..` — go up one level to parent folder (two dots = parent)
- `cd ~` — go to home directory
- `pwd` — verify current location before and after navigation
- Ask AI to show `pwd` after each step to verify navigation before file operations

### Common Mistakes
- Using relative paths without checking current location—the folder may not exist where you are
- Performing file operations (delete, move, copy) without verifying location first—could affect wrong folder
- Forgetting that relative paths are context-dependent—`cd Documents` only works if Documents exists in your current folder

### Connections
- **Builds on**: Terminal basics and file system tree structure (Lesson 2)
- **Leads to**: File operations—listing, viewing, and creating files with AI collaboration
