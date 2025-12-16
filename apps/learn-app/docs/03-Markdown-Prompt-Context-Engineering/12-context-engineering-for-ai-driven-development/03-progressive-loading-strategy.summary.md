### Core Concept
Three-tier progressive loading (Foundation always-loaded, Current task-specific, On-Demand referenced) allocates context intentionally to prevent degradation while leaving capacity for mid-session needs.

### Key Mental Models
- **Deliberate preloading**: Conscious decisions about what to load upfront rather than loading everything and hoping
- **Foundation as project DNA**: Files that apply to every task (CLAUDE.md, architecture.md, decisions.md) establish consistent patterns across sessions
- **On-Demand reserve**: 30% protected capacity prevents preloading waste by allowing files to be fetched mid-session when AI requests them
- **Tier allocation reflects priorities**: Large, frequently-changing files may belong in Current rather than Foundation (staleness risk)

### Critical Patterns
- **File importance matrix**: Classify by frequency (always vs rarely) and impact (high vs low) to determine tier placement
- **Foundation constraints**: Should be 10-15% of context (~20K tokens), stable across sessions, frequently referenced in all tasks
- **Current flexibility**: Task-specific allocation (20-30%) changes per session based on today's work
- **Iteration improves allocation**: Initial suggestions reveal constraints; project-specific knowledge refines tier placement

### AI Collaboration Keys
- AI suggests general three-tier framework; you refine based on project constraints (files that change frequently, cross-references that matter most)
- AI helps estimate file sizes in tokens when precise counts unavailable
- AI validates whether Foundation includes critical files needed by all tasks or files that should move to Current

### Common Mistakes
- Putting all large files in Foundation (bloats Foundation beyond 15% target)
- Including frequently-changing files in Foundation (loads stale versions)
- Not reserving 30% for On-Demand capacity (limits ability to fetch references mid-session)
- Treating all files as equally important (failing to identify truly critical Foundation files)

### Connections
- **Builds on**: Lessons 1-2 (understanding context limitations and degradation)
- **Leads to**: Lesson 4 (compressing when even good loading strategies hit limits), Lesson 5 (isolating unrelated tasks to prevent pollution)
