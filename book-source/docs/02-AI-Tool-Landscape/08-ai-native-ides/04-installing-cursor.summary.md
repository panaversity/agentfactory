### Core Concept
Cursor represents a different approach to AI-native IDEs: taking VS Code's proven architecture and rebuilding it with AI-first design. This gives you VS Code familiarity plus native AI features like Agent mode—autonomous multi-file code generation with diff-based review.

### Key Mental Models
- **Fork vs. Build-from-Scratch**: Cursor forks VS Code (inherits ecosystem), Zed builds from Rust (inherits speed)—different tradeoffs, both valid
- **.cursorrules as AI Context**: A project-root file that tells AI your coding standards, tech stack, and conventions—your prompt becomes internally enriched
- **BYOK vs. Subscription**: Bring-your-own-key (~$0.03/1M tokens) vs. Cursor Pro ($20/month)—choose based on usage volume and convenience preference

### Critical Patterns
- Migrate VS Code settings via import wizard OR manual copy of settings.json to Cursor's User folder
- Cmd+K (Ctrl+K) triggers inline generation; Tab accepts, Escape rejects
- Create `.cursorrules` in project root to enforce consistent AI output (type hints, docstrings, naming conventions)
- Configure "Files Exclude" for node_modules/.venv/__pycache__ to speed up indexing on large projects
- Keep Agent Auto-Mode OFF while learning—run with confirmation until you trust the workflow

### Common Mistakes
- Forgetting to authenticate—inline generation fails with "No API Key Configured" until you subscribe or add BYOK
- Naming the rules file wrong—must be exactly `.cursorrules` (no extension, starts with dot, in project root)
- Expecting instant extension compatibility—some VS Code extensions need reinstallation or have Cursor-specific quirks
- Leaving auto-indexing on massive monorepos—configure Index Ignore to skip large generated directories

### Connections
- **Builds on**: Zed installation and verification (Lesson 2), AI provider authentication concepts (Chapter 5-6)
- **Leads to**: Cursor Chat/Agent mode deep dive (Lesson 5), IDE comparison and workflow selection (Lesson 7)
