### Core Concept
Transform the Git patterns you've been using repeatedly into explicit documentation in `git-workflow.md`—a personal reference you can apply to new projects without re-learning.

### Key Mental Models
- **Tacit → Explicit**: What you do instinctively becomes written documentation; if you can't explain it, you don't truly own it
- **Three Fundamental Patterns**: Commit-Before-Experiment (safety checkpoint), Branch-Test-Merge (isolated testing), Push-for-Backup (disaster recovery)
- **Questions + Principles Structure**: Each pattern has decision questions ("Have I tested before merging?") and guiding principles ("Never modify main when uncertain")

### Critical Patterns
- **Pattern 1 (Commit-Before-Experiment)**: `git status` → `git diff` → `git add` → `git commit -m "Before [task]: [context]"` before any risky change
- **Pattern 2 (Branch-Test-Merge)**: Create branch → let AI implement → test locally → merge if good, `git branch -D` if bad
- **Pattern 3 (Push-for-Backup)**: `git push` at least daily + verify on GitHub that backup actually succeeded
- **Documentation structure**: For each pattern, document When to use / The Workflow / Questions to ask / Principles to follow

### Common Mistakes
- Committing directly to main when uncertain about the outcome (should branch first)
- Assuming push worked without checking GitHub to confirm backup exists
- Keeping patterns as tacit knowledge instead of writing them down—you can't reference what's not documented

### Connections
- **Builds on**: Git operations from Lessons 1-5 (commit, branch, merge, push, recover)
- **Leads to**: Using `git-workflow.md` as your safety net throughout Part 2 and future projects
