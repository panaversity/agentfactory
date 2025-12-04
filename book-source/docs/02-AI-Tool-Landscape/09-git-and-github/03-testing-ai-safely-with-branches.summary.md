### Core Concept
Branches are parallel timelines that let you test multiple AI-generated solutions simultaneously without risking your main code. Create a branch, experiment safely, merge winners, delete failures—main stays untouched throughout.

### Key Mental Models
- **Parallel Universes**: Each branch is an independent version of your code; changes on one don't affect others until you explicitly merge
- **Safe Experimentation**: Branches enable fearless testing—if AI suggestions break things, delete the branch, not your work
- **Isolation → Comparison → Integration**: Create branches for options, test each independently, merge only what proves valuable

### Critical Patterns
- Create branch: `git branch feature-name` (creates pointer) → `git switch feature-name` (moves to it)
- Shortcut: `git checkout -b feature-name` (creates and switches in one command)
- Merge winner: switch to main first (`git switch main`) → then `git merge winning-branch`
- Clean up: `git branch -d branch-name` (safe delete, blocks if unmerged) vs `-D` (force delete)
- Decision framework: Branch when testing multiple approaches OR risking breakage; commit directly for small proven changes

### Common Mistakes
- Creating branches but forgetting to switch to them (changes accidentally go to main)
- Not verifying which branch you're on before making changes (use `git status` or `git branch`)
- Force-deleting branches (`-D`) without checking if they contain unmerged work
- Over-branching for trivial changes where commits would suffice

### Connections
- **Builds on**: Lesson 1-2's commit fundamentals (branches are pointers to commits, commits are snapshots)
- **Leads to**: Pull requests (branches shared for review), merge conflict resolution, team collaboration workflows
