### Core Concept
An AI-native IDE is a code editor designed from the ground up with AI collaboration as a core architectural principle—not retrofitted with plugins. This foundational difference enables deeper integration, full project context, and autonomous agent capabilities.

### Key Mental Models
- **Foundation vs Retrofit**: Like building a modern house with planned electrical wiring versus adding outlets to an 1800s house—native architecture doesn't work around constraints
- **Architecture → Capability**: Native design enables context-awareness (full project), multi-model flexibility (right model per task), and agent autonomy (multi-file changes with preview)
- **Complexity Threshold**: Architecture differences matter most for complex multi-file projects and agent-driven workflows; matter less for simple single-file edits

### Critical Patterns
- Three characteristics define AI-native: context-aware AI (understands full project, not just current file), multi-model support (assign different models to different tasks), agent capabilities (autonomous changes with approval workflow)
- 2025 landscape has three approaches: Zed (speed-focused, Rust-based), Cursor (VS Code evolution with AI-first rebuild), Antigravity (agent control plane, launched Nov 2025)
- Evaluate trade-offs: AI-native offers tighter integration and faster response; plugin-based offers larger ecosystems and familiar interfaces

### Common Mistakes
- Equating "has AI features" with "AI-native"—plugins add surface-level AI; native architecture provides deep integration
- Ignoring ecosystem trade-offs—plugin-based editors have mature extension libraries that AI-native tools are still building
- Underestimating architecture impact on complex projects—context quality and agent capabilities depend on architectural access

### Connections
- **Builds on**: Claude Code (Chapter 5), Gemini CLI (Chapter 6)—you've used AI tools, now understand what makes editors AI-native
- **Leads to**: Hands-on installation and usage of Zed, Cursor, and Antigravity (Lessons 2-7)
