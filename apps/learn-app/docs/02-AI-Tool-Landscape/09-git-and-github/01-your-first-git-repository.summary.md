### Core Concept
Git creates **save points** for your project—snapshots you can return to if AI-generated code breaks something—enabling fearless experimentation.

### Key Mental Models
- **Video Game Saves**: Commits are like saving before a boss fight; if you die (break code), reload and try again
- **Three Zones**: Working directory (your files) → Staging area (what you're about to save) → Repository (permanent history)
- **Tracking = Protection**: Git only protects files you explicitly tell it to track; untracked files aren't in your save points
- **Commit = Snapshot**: Each commit captures your entire project at that moment with a unique ID and message

### Critical Patterns
- `git init` — creates the `.git` folder that stores all history (don't delete this!)
- `git status` — shows what Git sees: tracked, untracked, staged, modified
- `git add <file>` — stages a file (puts it in the "to be saved" checklist)
- `git commit -m "message"` — creates the actual save point with a description
- `git log` — shows your save point history (commit IDs, authors, dates, messages)

### Common Mistakes
- Forgetting to stage files before committing (commit saves nothing if staging area is empty)
- Deleting the `.git` folder (destroys entire project history)
- Committing everything instead of selecting what belongs together (staging exists for a reason)
- Writing vague commit messages like "update" instead of describing what changed

### Connections
- **Builds on**: Terminal basics (`mkdir`, `cd`, `ls -la`), file creation
- **Leads to**: Making changes and viewing diffs, undoing mistakes, working with remote repositories
