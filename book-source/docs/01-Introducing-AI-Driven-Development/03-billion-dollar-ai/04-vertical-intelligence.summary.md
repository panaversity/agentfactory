### Core Concept
The paradigm has shifted from "Don't Repeat Yourself" (code reuse) to "Reuse Intelligence, Not Code." When AI generates specialized code in seconds, maintaining reusable libraries becomes more expensive than regenerating fresh code per application. Value now lives in system prompts, skills, and integrations—not code logic.

### Key Mental Models
- **Disposable Code, Permanent Intelligence**: Code is cheap to generate but expensive to maintain across applications. Intelligence (prompts, skills, integrations) is expensive to create once but infinitely reusable.
- **Five Components Framework**: Reusable subagents have five layers—system prompt (persona + constraints), horizontal skills (generic infrastructure), vertical skills (domain expertise), horizontal MCPs (dev tools), and vertical MCPs (industry APIs).
- **Defensibility Through Vertical Integration**: Generic code and horizontal skills are easily replicated. Vertical MCP connections (industry-specific integrations like Epic for healthcare or Bloomberg for finance) create competitive moats.

### Critical Patterns
- Design system prompts with WHO (persona), WHAT (knowledge scope), and CONSTRAINTS (boundaries)
- Distinguish horizontal components (reusable across all domains) from vertical components (reusable only within a domain)
- Vertical MCPs are the defensibility layer—competitors must rebuild these integrations from scratch
- Use the "mass production vs. 3D printing" mental model: when customization is free, standardization loses its advantage

### Common Mistakes
- Applying traditional DRY thinking when code generation is nearly free—this overinvests in maintenance
- Treating all five components as equally defensible (they're not—vertical MCPs create the moat, system prompts are easiest to copy)
- Confusing "disposable" with "worthless"—the code is disposable, but the intelligence that generates it is the permanent asset

### Connections
- **Builds on**: Market opportunity concepts from earlier lessons in this chapter
- **Leads to**: Piggyback Protocol Pivot—how to actually enter vertical markets with this architecture
