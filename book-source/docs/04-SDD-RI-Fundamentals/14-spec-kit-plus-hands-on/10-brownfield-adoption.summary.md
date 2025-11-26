## Lesson Title

Brownfield adoption teaches the safe, strategic process of integrating Spec-Kit Plus into existing projects while preserving team knowledge. Unlike greenfield projects where you control the entire structure from the start, brownfield work requires protecting accumulated institutional knowledge (custom CLAUDE.md, slash commands, team conventions) while avoiding data loss from framework-driven overwrites. The lesson establishes a three-layer safety workflow (git branches + manual backups + git commit history) before running `specifyplus init --here`, then shows students how to classify and merge their existing knowledge into the appropriate Spec-Kit Plus locations (constitution.md for standards, CLAUDE.md for collaboration patterns).

### Mental Models

- **Greenfield vs Brownfield**: Greenfield means starting from scratch with no existing code, conventions, or institutional knowledge. Brownfield means inheriting an existing codebase with months/years of accumulated decisions, team knowledge, and working code that must be protected during framework adoption.

- **File Preservation Profile**: Not all files are equally at risk during init. Understanding exactly what gets overwritten (CLAUDE.md, .specify/), what gets created (constitution.md, templates), and what stays completely safe (source code, git history, custom slash commands) lets students make informed decisions about backup strategy.

- **Content Classification**: Team knowledge stored in a single CLAUDE.md file must be sorted by destination after adoption—coding standards and architecture principles belong in constitution.md (project-specific rules), while AI collaboration patterns and behavioral instructions belong appended to CLAUDE.md (instructions for AI working with the team).

- **Redundant Safety Strategy**: Multiple recovery paths (git branch isolation, manual backup files, git commit history, diffs) provide overlapping protection. If one recovery method fails, others can restore content—eliminating single points of failure.

### Key Patterns

- **Three-Layer Safety Workflow**: Create experiment branch (isolates main branch from risk) + create backup files (manual recovery) + commit current state (git recovery history) BEFORE running init. This produces three independent ways to recover if needed.

- **Diff-Based Verification**: After running init, inspect actual changes via `git diff --name-only` to confirm only expected files changed. This prevents surprising overwrites and builds confidence in the safety of the process.

- **Content Routing Decision Matrix**: Establish clear classification rules before merging—coding standards/architecture/design patterns go to constitution.md (project constraints), collaboration behaviors/AI interaction patterns go to CLAUDE.md (behavioral instructions). This prevents mixing concerns and maintains clean separation.

- **Incremental Adoption Strategy**: Teams don't have to adopt Spec-Kit Plus all at once. Testing safely on a branch, merging selectively, and keeping both systems coexisting temporarily lets teams transition gradually without disrupting existing workflows.

### Common Mistakes

- **No Backup Before Init**: Running `specifyplus init --here` without backing up CLAUDE.md first. If the init overwrites your team knowledge and you don't have backups or git history, the content is permanently lost—no recovery mechanism exists.

- **Single Point of Failure**: Relying on only one recovery method (like git commit) and ignoring the others. If you used git and the git history gets corrupted or accidentally reset, you have no fallback. Multiple overlapping recovery methods prevent total loss.

- **Wrong Content Destination**: Putting coding standards into CLAUDE.md instead of constitution.md, or putting behavioral AI instructions into constitution.md instead of CLAUDE.md. This confuses concerns—constitution defines project rules, CLAUDE.md defines how AI should behave within those rules.

- **Testing on Main Branch**: Running init directly on main branch instead of an experiment branch. If something unexpected happens, your production branch is affected immediately instead of isolated in a test branch.

- **Losing Context During Merge**: After init creates the new files, failing to understand what your original content was for. If you don't read and categorize your CLAUDE.md.backup content before attempting the merge, you might accidentally discard important team knowledge while integrating the new structure.

### Progression Context

- **Builds on**: Chapter 14 foundation lessons teaching greenfield workflow, CLAUDE.md structure, constitution.md purpose, slash commands, and specifyplus initialization. Students understand what Spec-Kit Plus does and how it organizes knowledge before learning how to integrate it into existing systems.

- **Leads to**: Capstone (Lesson 15, Chapter 14) where students apply full Spec-Kit Plus workflow to real-scale projects combining all previous lessons, including safe brownfield adoption patterns for teams inheriting existing systems.
