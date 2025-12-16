### Core Concept
Pull requests (PRs) are GitHub's code review mechanism where you propose merging changes and document them BEFORE they affect main—in AI-native development, this includes transparently documenting which AI generated what code and what you modified.

### Key Mental Models
- **PR as Safety Checkpoint**: PRs give you a moment to evaluate AI-generated code before it ships; commits save changes, PRs formalize the review decision
- **Three-Part PR Description**: Summary (what changed), Testing (how verified), AI Attribution (what AI generated vs. what you modified)
- **Diff Review as Quality Gate**: Red lines = removed, green lines = added; never merge code you don't understand

### Critical Patterns
- PR workflow: push feature branch → create PR on GitHub → review diff → add description → merge
- PR description template: Summary, Changes, AI Assistance (tool used, what generated, what modified), Testing Done
- Diff review checklist: Does it match intent? Any bugs? Code I don't understand? Security issues?
- AI transparency format: Name the tool, list what it generated, list what you changed, note bugs you found and fixed

### Common Mistakes
- Merging without reviewing the diff—always click "Files Changed" and verify changes match your intent
- Hiding AI assistance—professional practice is transparent attribution, not overclaiming credit
- Merging AI code you don't understand—test it manually or ask AI to explain before approving
- Skipping the "What I Modified" section—reviewers need to know where human judgment was applied

### Connections
- **Builds on**: Feature branches (Lesson 3), pushing to GitHub (Lesson 4), commit messages (Lesson 1)
- **Leads to**: Team collaboration workflows, CI/CD pipelines that run on PR creation, professional portfolio demonstration
