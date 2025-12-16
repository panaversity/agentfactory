### Core Concept
Four frameworks dominate agent development, each with a distinct philosophy: OpenAI (Handoff-Centric), Google ADK (Service-Centric), Anthropic (Capability-Centric), and Microsoft (Conversation-Centric). Match the framework's philosophy to your problem structure.

### Key Mental Models
- **Philosophy → Spec Focus**: Each framework shapes what your specification emphasizes—transfer rules, data schemas, tool definitions, or interaction patterns
- **Problem Type → Framework**: Routing/triage → OpenAI; Business process → Google; Task execution → Anthropic; Team collaboration → Microsoft
- **Concepts Transfer**: 3+1 Architecture, 5-Step Loop, patterns, and Agent Ops appear in all frameworks—implementation differs, principles persist

### Critical Patterns
- **OpenAI Agents SDK**: Handoff-centric. Spec defines triage agent, specialists, and `handoff_instructions`. Best for simple routing workflows.
- **Google ADK**: Service-centric. Spec defines Artifact schemas, `input_schema`/`output_schema`, A2A protocols. Best for stateful enterprise processes.
- **Anthropic Agents Kit**: Capability-centric. Spec defines MCP server configuration and tool access. Best for agents that *do work* (coding, research).
- **Microsoft Framework**: Conversation-centric. Spec defines roles, rules of engagement, `GroupChatSelectionMethod`, termination criteria. Best for multi-agent collaboration.

### Common Mistakes
- Choosing framework before understanding your problem's core interaction pattern
- Assuming frameworks are interchangeable—each has a distinct philosophy that shapes your spec
- Writing specs that don't match the framework's emphasis (e.g., tool definitions for OpenAI instead of handoff rules)
- Ignoring that core concepts transfer—you're not starting over when switching frameworks

### Connections
- **Builds on**: All prior lessons—frameworks implement the architecture, loop, patterns, ops, and security you've learned
- **Leads to**: Chapters 35-38—hands-on implementation with each framework using AIDD and SDD
