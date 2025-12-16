### Core Concept
AI collaboration is iterative—you build solutions through multi-turn conversations where each message refines constraints, reviews diffs line-by-line, and accumulates context. This transforms AI from a code generator into a collaborative development partner.

### Key Mental Models
- **Iterative Refinement**: First draft → add constraints → security review → integration. Each message improves the solution rather than replacing it.
- **Context as Working Memory**: Context = files added (@filename) + conversation history + editor content. You actively shape what AI "knows" about your project.
- **Diff = Code Review**: Don't accept/reject wholesale—examine changes line-by-line, modify what's 80% right, catch bugs before they land.

### Critical Patterns
- **Multi-turn flow**: Specification → constraints → security review → implementation (4+ messages typical)
- **Intentional context**: Add only relevant files, remove when done; ask AI to summarize understanding
- **Diff review checklist**: Bugs? Error handling? Type hints? Imports correct? Style consistent?
- **Claude Rules** (`.cursor/rules.md`): Write standards once, AI follows automatically
- **Thread orchestration**: Separate threads for models/routes/tests; reference decisions across threads
- **Context priming**: "I'm building X with these patterns: [list]" before requesting features
- **Specification prompts**: Include requirements + constraints + non-goals, not just "build X"

### Common Mistakes
- Auto-accepting diffs without review—bugs like double-encoding pass through unnoticed
- Adding entire project to context instead of specific files—wastes tokens, reduces quality
- Implementation prompts ("Create login") instead of specification prompts (requirements + constraints)
- One-shot generation instead of iterative refinement—misses early issue detection
- Letting conversations grow too long—start new thread with summary when context bloats

### Connections
- **Builds on**: Cursor installation and interface basics (Lessons 3-4)
- **Leads to**: AI-native debugging techniques—finding and fixing bugs with AI assistance
