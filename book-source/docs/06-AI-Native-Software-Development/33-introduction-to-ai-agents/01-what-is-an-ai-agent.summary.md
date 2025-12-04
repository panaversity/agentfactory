### Core Concept
An AI agent uses a language model **in a loop**—reason, act, observe, repeat—until the goal is achieved. The loop, not the tools, is what distinguishes agents from chatbots.

### Key Mental Models
- **Loop-Based Definition**: Agents iterate autonomously; chatbots respond once. Claude Code loops until your task is complete.
- **5-Level Taxonomy**: Classify any AI system by who controls strategy—from Level 0 (single response) to Level 4 (self-evolving).
- **Director vs Bricklayer**: Specify intent and constraints (director) rather than every step (bricklayer). Agents reason about *how*.

### Critical Patterns
- Classify systems by asking: "Who controls the reasoning loop—human or system?"
- Level 2 agents plan multi-step strategies from goals; Level 3 coordinates specialists
- Director thinking requires precise intent + clear constraints; vague goals create unreliable agents
- Claude Code operates at Level 2-3 depending on task complexity

### Common Mistakes
- Confusing tool sophistication with agent level—the level is about *autonomy*, not *capability*
- Giving bricklayer-style instructions to agents ("do X, then Y, then Z") instead of specifying intent
- Assuming all AI assistants are agents—Level 0-1 systems aren't true agents

### Connections
- **Builds on**: Specification-Driven Development (Part 4)—specs become agent instructions
- **Leads to**: Core Agent Architecture (Lesson 2)—how agents are built internally
