# Chapter 34: OpenAI Agents SDK — Production Mastery

Build production-grade AI agents using OpenAI's official Agents SDK. This chapter transforms you from SDK beginner to expert through hands-on implementation of the complete Customer Support FTE system.

## What You'll Build

A **Customer Support Digital FTE** that:
- Routes inquiries to specialist agents (triage, FAQ, booking, escalation)
- Persists conversations with SQLite sessions
- Validates inputs with agent-based guardrails
- Connects to external tools via MCP (Context7 for documentation)
- Implements agentic RAG with FileSearchTool
- Provides full observability via tracing

## Prerequisites

- **Chapter 33**: AI Agent Foundations (Agent/Environment/Action loop)
- **Python 3.11+** with `uv` package manager
- **OpenAI API key** (`export OPENAI_API_KEY=...`)

## Learning Path

| Lesson | Title | Layer | Key Patterns |
|--------|-------|-------|--------------|
| 1 | SDK Setup & First Agent | L1 | Agent, Runner, run_sync |
| 2 | Function Tools & Context | L1 | @function_tool, Pydantic context |
| 3 | Agents as Tools | L2 | agent.as_tool(), orchestration |
| 4 | Handoffs & Filtering | L2 | handoff(), on_handoff, filters |
| 5 | Guardrails & Validation | L2 | @input_guardrail, agent-based |
| 6 | Sessions & Memory | L2 | SQLiteSession, branching |
| 7 | Tracing & Observability | L2+3 | RunHooks, trace(), custom_span |
| 8 | MCP Integration | L3 | MCPServerStreamableHttp, Context7 |
| 9 | RAG with FileSearchTool | L3 | FileSearchTool, vector stores |
| 10 | Capstone: Customer Support FTE | L4 | Complete system integration |

## Running Examples

All code examples are executable. To run:

```bash
# Install dependencies
uv add openai-agents

# Set API key
export OPENAI_API_KEY=your_key_here

# Run any example
python examples/hello_agent.py
```

## Success Criteria

After completing this chapter, you can:
- [ ] Create agents with custom tools and context
- [ ] Build multi-agent systems with handoffs
- [ ] Implement production guardrails
- [ ] Persist conversations with SQLite
- [ ] Debug with lifecycle hooks and tracing
- [ ] Connect agents to MCP servers (Context7)
- [ ] Implement agentic RAG with FileSearchTool
- [ ] Deploy a complete Customer Support FTE
