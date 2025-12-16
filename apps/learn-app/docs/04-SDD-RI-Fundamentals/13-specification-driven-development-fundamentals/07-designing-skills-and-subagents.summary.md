### Core Concept
Persona + Questions + Principles (P+Q+P) is a pattern-activation framework that shifts AI from generic pattern-matching to context-specific reasoning, enabling reusable intelligence that actually adapts to your constraints.

### Key Mental Models
- **Prediction mode vs reasoning mode**: Generic instructions (e.g., "make it secure") trigger pattern-matching; P+Q+P triggers context analysis
- **Persona as cognitive stance**: Specific thinking pattern (e.g., "defensive programmer focused on attack surfaces") activates domain expertise better than generic "expert"
- **Questions force context analysis**: Context-specific questions (5+ for Subagents, 2-4 for Skills) replace vague aspirations with measurable reasoning
- **Principles provide decision frameworks**: When tradeoffs appear, principles guide consistent choices (not aspirations like "be efficient")

### Critical Patterns
- **Input Validation Skill**: Persona (defensive programming), Questions (valid types? ranges? error handling? strict/lenient? boundaries?), Principles (validate boundaries, fail fast, type hints + runtime, documentation)
- **Performance Optimization Subagent**: Persona (performance engineer analyzing scalability), Questions (data volumes? latency? N+1 queries? caching? memory? data structures?), Principles (optimize actual use case, measure, document requirements, choose intentionally)
- **Security Review Subagent**: Persona (threat modeler reviewing attack surfaces), Questions (threat actors? data sensitivity? compliance? attack vectors?), Principles (defense in depth, fail secure, least privilege)

### AI Collaboration Keys
- Specifications + P+Q+P design enable AI agents to reason rather than pattern-match
- Skills guide AI consistently (apply same error-handling approach to all APIs)
- Subagents provide expert review automatically (security audit, performance analysis, accessibility check)
- P+Q+P documentation produces more consistent outputs than repeated prompting

### Common Mistakes
- Weak Persona ("You are an expert") vs specific Persona ("Defensive programmer focused on input validation attack surfaces")
- Questions that pattern-match ("Is this secure?") vs questions that force analysis ("What could an attacker provide?")
- Principles that aspirate ("Use best practices") vs principles that guide ("Validate at boundaries, fail fast, type hints + runtime")
- Confusing decision point count (2-4 for Skills, 5+ for Subagents) leading to over-complex Skills or under-utilized Subagents

### Connections
- **Builds on**: Recognizing when patterns recur (Lesson 6 identifies intelligence opportunities)
- **Leads to**: Implementing P+Q+P in actual framework tools (Claude Code Skills, subagent runtimes, MCP servers)
