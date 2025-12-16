### Core Concept
Git provides three safe undo commands that form your safety net for fearless experimentation—you can always inspect changes and recover from mistakes without losing work.

### Key Mental Models
- **View → Decide → Undo**: Always inspect changes with `git diff` before deciding whether to keep or discard them; informed undo is safe undo
- **Staged vs Unstaged Recovery**: Different commands for different states—`git restore` undoes working directory changes, `git reset HEAD` undoes staging
- **Non-Destructive vs Destructive**: Most undo commands preserve your work (restore, reset HEAD); only `--hard` variants permanently delete—know the difference before executing

### Critical Patterns
- `git diff <file>`: View exactly what changed (+ lines added, - lines removed) before committing
- `git restore <file>`: Discard all unstaged changes, return file to last committed state
- `git reset HEAD <file>`: Unstage a file without deleting it—file stays in working directory
- Safety workflow: Commit working code → Let AI make changes → `git diff` to inspect → `git restore` if broken

### Common Mistakes
- Confusing `restore` and `reset`: restore discards changes, reset only changes staging status
- Using `git reset --hard` when `git restore` would suffice—`--hard` is destructive and permanent
- Skipping `git diff` inspection—you can't make informed undo decisions without seeing what changed first
- Assuming unstaging deletes files—`git reset HEAD` only removes from staging area, file contents unchanged

### Connections
- **Builds on**: Commits as save points (Lesson 1)
- **Leads to**: Branching for parallel experimentation, collaborative workflows
