### Core Concept
Multi-agent systems use routing agents (triagers) that understand intent and hand off to specialist agents with domain-specific tools—this architecture separates concerns, keeps agent instructions focused, and matches production AI systems. Expose the routing chain in API responses for transparency.

### Key Mental Models
- **Specialists over generalists**: One agent handling scheduling, collaboration, AND analysis would need massive instructions with conflicting tools. Instead: specialized agents with focused instructions and only their domain tools.
- **Handoff as tool calling**: `handoff(specialist_agent)` wraps a specialist as a callable tool. When triage decides to route, it calls the handoff tool, which transfers control completely. The response flows through normally.
- **Routing visibility**: `handoff_chain` in responses shows which agents were involved—["triage"] means direct answer, ["triage", "scheduler"] means routed to scheduler. Clients understand what happened.

### Critical Patterns
- Create specialist agents with focused instructions and domain-specific tools
- Define triage agent with `handoff()` tools for each specialist
- Validate task exists BEFORE expensive agent call (same as database queries)
- Build rich context including task details—agent needs full picture to give relevant advice
- Expose `handled_by` and `handoff_chain` in response for routing transparency
- Stream responses to show routing decisions as they happen (handoff event, tool calls, tokens)
- Provide direct specialist endpoints (bypass triage) for cases where routing isn't needed

### Common Mistakes
- Not passing context through handoffs—specialist doesn't know about the task being discussed
- Overlapping specialist tools—both agents handling "meetings" confuses routing
- Missing routing visibility—response only has `response` field, client can't see which agents were involved
- Building instructions that expose implementation (role labels, metadata comments)—should be transparent to client
- Not handling not-found before agent call—expensive LLM call for nonexistent resource wastes money

### Connections
- **Builds on**: Lesson 6's streaming that now streams agent responses and routing events
- **Leads to**: Lesson 8's capstone combining all patterns into production system
