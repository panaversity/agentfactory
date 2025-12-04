### Core Concept
File operations (copy, move, delete) have different risk levels that require different safety approaches. Your role in AI-native development is not to memorize command flags, but to understand what operation you need, supervise AI's execution, and verify results.

### Key Mental Models
- **Risk Hierarchy**: Copy (low risk—original untouched) → Move (medium risk—location changes) → Delete (high risk—difficult to undo)
- **Verify-Before-and-After**: Always check what exists before an operation (`ls`), understand what will change, then verify the result after
- **Collaborative Safety Culture**: You specify safety requirements, AI internalizes the pattern—together you establish safe workflows

### Critical Patterns
- Copy files: `cp source destination` (original remains)
- Copy folders: `cp -r folder/ backup/` (recursive flag for directories with contents)
- Safety workflow: (1) list affected files, (2) understand the command, (3) ask if reversible, (4) backup if risky, (5) verify afterward
- Verification pattern: use `ls -la` to confirm both original and copy exist after copying

### Common Mistakes
- Performing file operations without first listing what files will be affected
- Forgetting the `-r` flag when copying folders (copies nothing or fails)
- Not verifying results after operations—assuming success without checking
- Memorizing flags instead of understanding intent (AI handles syntax; you handle verification)

### Connections
- **Builds on**: Lesson 2's 5-step safety pattern, Lesson 3's navigation and listing commands (`ls`, `cd`, `pwd`)
- **Leads to**: Move operations, delete operations with proper safeguards, and more complex file management workflows
