### Core Concept
AI tools have fundamentally different tradeoffs (Claude Code: deep reasoning + limited context vs. Gemini CLI: massive context + broad analysis). Selecting the right tool depends on evaluating task scope, reasoning depth, context budget, and file selection control—not picking one tool for all problems.

### Key Mental Models
- **Context window as budget constraint**: 200K (Claude) vs. 2M (Gemini) tokens create fundamentally different task categories
- **Reasoning depth vs. breadth tradeoff**: Claude excels at step-by-step architectural decisions; Gemini at cross-file pattern analysis
- **Task scope dimensions**: Lines of code, files affected, reasoning need, and file control each independently influence tool choice
- **Two-phase workflows**: Use Gemini for exploration/understanding, then Claude for implementation/reasoning (complements strengths)
- **Progressive loading as context optimization**: Claude Code with selective file loading can handle 100K-line codebases without Gemini

### Critical Patterns
- **Decision tree application**: Start with codebase size → branch on complexity/control needs → arrive at tool choice
- **Scenario-based reasoning**: Small (1-10K) → Claude, Medium (10-100K) → Claude with progressive loading, Large (100K+) → Gemini for exploration OR Claude with memory files
- **File selection control principle**: High control needed (you know which files matter) → Claude; Low control (AI finds patterns) → Gemini
- **Hybrid tool strategy for enterprise**: Gemini to understand legacy system architecture, Claude to refactor specific modules
- **Session continuity consideration**: Claude Code with memory files (Lesson 6) enables multi-day development on large codebases without Gemini

### AI Collaboration Keys
- Claude excels when you provide focused context (you choose files) + deep reasoning questions
- Gemini excels when you have no mental model yet (load entire codebase, let AI find patterns)
- Memory files bridge gap: transforms "large codebase" problem into "essential context only" problem for Claude
- Tool selection is itself collaborative: describe task characteristics, AI recommends tool with reasoning

### Common Mistakes
- Loading entire codebase into Claude (token waste) instead of using progressive loading
- Using Gemini for every task (good for exploration, bad for detailed reasoning/implementation)
- Ignoring session continuity (could use Claude with memory files instead of default to Gemini)
- Treating tool selection as permanent (may switch tools mid-project as understanding deepens)
- Assuming 200K context limit is hard (extended to 1M for tier 4+ users; changes the decision calculus)

### Connections
- **Builds on**: Lesson 3 (progressive loading is Claude-specific optimization), Lesson 6 (memory files enable Claude to handle larger projects)
- **Leads to**: Lesson 8 (tool selection decision is part of remediation strategy when context problems appear)
- **Related skill**: Skills/tool-selection-framework (reusable decision framework for any AI-assisted development)
- **Integration**: Works with memory files to create hybrid strategy: Gemini for architecture understanding, Claude with persisted context for implementation
