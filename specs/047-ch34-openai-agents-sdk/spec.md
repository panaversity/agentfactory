# Feature Specification: Chapter 34 - OpenAI Agents SDK (Production Mastery)

**Feature Branch**: `047-ch34-openai-agents-sdk`
**Created**: 2025-12-26
**Status**: Draft
**Proficiency Level**: B1-B2 → C1 (Build production FTEs)
**Source Examples**: `github.com/openai/openai-agents-python/examples/`

## Vision: Building FTEs (Full-Time Equivalent Digital Employees)

This chapter teaches production mastery of the OpenAI Agents SDK—the same infrastructure powering ChatGPT's agentic features. Students don't just learn the API; they master the patterns needed to build autonomous digital workers that can:
- Maintain context across complex multi-turn workflows
- Hand off to specialists while preserving conversation state
- Self-validate with guardrails before taking action
- Persist memory across sessions
- Produce debuggable traces for continuous improvement

**Reference Repository**: All patterns are based on real examples from `openai/openai-agents-python`.

## Assumed Knowledge

**What students know BEFORE this chapter**:
- 5-Level Agent Taxonomy, 3+1 Architecture, 5-Step Loop (Chapter 33)
- Multi-agent patterns: Coordinator, Sequential, Iterative, Human-in-the-Loop
- Agent Ops: evaluation, tracing, golden datasets
- Python async/await, type hints, Pydantic models (Part 5)
- Specification-driven development (Part 4)

**What this chapter teaches from scratch**:
- SDK primitives: Agent, Runner, @function_tool, handoff()
- Context objects for state management (Pydantic models across agents)
- Handoff filters and callbacks for context control
- Agent.clone() and agent.as_tool() for dynamic composition
- Session backends: SQLite, SQLAlchemy, Redis, AdvancedSQLite with branching
- Lifecycle hooks: RunHooks, AgentHooks for monitoring
- Tracing: trace(), custom_span(), gen_trace_id(), group_id
- Agent-based guardrails (guardrail that calls another agent)
- MCP integration with MCPServerStdio

---

## Chapter Structure (8 Lessons)

### Lesson 1: SDK Setup & First Agent
**Pattern Source**: `examples/basic/hello_world.py`, `examples/model_providers/litellm_provider.py`

### Lesson 2: Function Tools & Context Objects
**Pattern Source**: `examples/basic/tools.py`, `examples/customer_service/main.py` (AirlineAgentContext)

### Lesson 3: Agents as Tools & Orchestration
**Pattern Source**: `examples/agent_patterns/agents_as_tools.py`, `examples/financial_research_agent/manager.py`

### Lesson 4: Handoffs & Message Filtering
**Pattern Source**: `examples/handoffs/message_filter.py`, `examples/agent_patterns/routing.py`

### Lesson 5: Guardrails & Agent-Based Validation
**Pattern Source**: `examples/agent_patterns/input_guardrails.py`, `examples/agent_patterns/output_guardrails.py`

### Lesson 6: Sessions & Conversation Memory
**Pattern Source**: `examples/memory/sqlite_session_example.py`, `examples/memory/advanced_sqlite_session_example.py`

### Lesson 7: Tracing, Hooks & Observability
**Pattern Source**: `examples/basic/lifecycle_example.py`, `examples/agent_patterns/deterministic.py`

### Lesson 8: Capstone - Customer Support FTE
**Pattern Source**: `examples/customer_service/main.py` (full implementation)

---

## User Scenarios & Testing

### User Story 1 - First Agent with LiteLLM (Priority: P1)

**Concrete Pattern**: Student implements `hello_world.py` pattern, then extends with `LitellmModel` for free models.

**Acceptance Scenarios**:
1. **Given** SDK installed, **When** student runs basic agent, **Then** response within 5 seconds
2. **Given** `LitellmModel(model="anthropic/claude-3-5-sonnet")`, **When** student runs same code, **Then** identical behavior with Anthropic
3. **Given** `set_tracing_disabled(True)`, **When** using non-OpenAI model, **Then** no tracing errors

---

### User Story 2 - Context Objects for State (Priority: P1)

**Concrete Pattern**: `AirlineAgentContext` from customer_service example - Pydantic model passed via `context=` parameter.

```python
class TaskManagerContext(BaseModel):
    user_id: str | None = None
    current_project: str | None = None
    tasks_added: int = 0
```

**Acceptance Scenarios**:
1. **Given** context Pydantic model, **When** tool accesses `context.context.user_id`, **Then** state persists across tool calls
2. **Given** `RunContextWrapper[TaskManagerContext]`, **When** type hints used, **Then** IDE autocomplete works
3. **Given** multiple tools modify context, **When** agent run completes, **Then** all mutations visible

---

### User Story 3 - Agents as Tools with Custom Extractors (Priority: P1)

**Concrete Pattern**: `financial_research_agent/manager.py` - `agent.as_tool()` with `custom_output_extractor`

```python
fundamentals_tool = financials_agent.as_tool(
    tool_name="fundamentals_analysis",
    tool_description="Get financial metrics write-up",
    custom_output_extractor=lambda r: str(r.final_output.summary),
)
```

**Acceptance Scenarios**:
1. **Given** `agent.as_tool(tool_name, tool_description)`, **When** orchestrator calls it, **Then** sub-agent runs and returns
2. **Given** `custom_output_extractor`, **When** sub-agent returns structured output, **Then** only summary text passed to orchestrator
3. **Given** multiple sub-agents as tools, **When** orchestrator runs, **Then** can call any in any order

---

### User Story 4 - Agent Cloning for Dynamic Composition (Priority: P2)

**Concrete Pattern**: `financial_research_agent/manager.py` - `writer_agent.clone(tools=[...])`

```python
writer_with_tools = writer_agent.clone(tools=[fundamentals_tool, risk_tool])
```

**Acceptance Scenarios**:
1. **Given** base agent, **When** `agent.clone(tools=[new_tools])`, **Then** new agent has additional tools
2. **Given** cloned agent, **When** original modified, **Then** clone unaffected
3. **Given** runtime decision, **When** clone created with conditional tools, **Then** behavior changes accordingly

---

### User Story 5 - Handoff Callbacks & Context Injection (Priority: P1)

**Concrete Pattern**: `customer_service/main.py` - `on_handoff` callback

```python
async def on_seat_booking_handoff(context: RunContextWrapper[AirlineAgentContext]) -> None:
    context.context.flight_number = f"FLT-{random.randint(100, 999)}"

handoff(agent=seat_booking_agent, on_handoff=on_seat_booking_handoff)
```

**Acceptance Scenarios**:
1. **Given** `handoff(agent, on_handoff=callback)`, **When** handoff occurs, **Then** callback runs before target agent
2. **Given** callback modifies context, **When** target agent accesses context, **Then** sees modified values
3. **Given** callback is async, **When** needs external lookup, **Then** can await database/API calls

---

### User Story 6 - Handoff Message Filtering (Priority: P2)

**Concrete Pattern**: `handoffs/message_filter.py` - `input_filter` with `handoff_filters.remove_all_tools`

```python
def custom_filter(data: HandoffInputData) -> HandoffInputData:
    filtered = handoff_filters.remove_all_tools(data)
    return HandoffInputData(
        input_history=filtered.input_history[2:],  # Remove first 2 messages
        pre_handoff_items=tuple(filtered.pre_handoff_items),
        new_items=tuple(filtered.new_items),
    )

handoff(spanish_agent, input_filter=custom_filter)
```

**Acceptance Scenarios**:
1. **Given** `input_filter` on handoff, **When** handoff occurs, **Then** filter transforms history before target sees it
2. **Given** `handoff_filters.remove_all_tools`, **When** applied, **Then** all tool calls removed from history
3. **Given** custom filter removes messages, **When** target agent runs, **Then** only sees filtered history

---

### User Story 7 - Agent-Based Guardrails (Priority: P1)

**Concrete Pattern**: `agent_patterns/input_guardrails.py` - guardrail function that calls another agent

```python
guardrail_agent = Agent(
    name="Guardrail check",
    instructions="Check if user is asking for math homework.",
    output_type=MathHomeworkOutput,
)

@input_guardrail
async def math_guardrail(context, agent, input) -> GuardrailFunctionOutput:
    result = await Runner.run(guardrail_agent, input, context=context.context)
    output = result.final_output_as(MathHomeworkOutput)
    return GuardrailFunctionOutput(
        output_info=output,
        tripwire_triggered=output.is_math_homework,
    )
```

**Acceptance Scenarios**:
1. **Given** agent-based guardrail, **When** runs in parallel with main agent, **Then** can use full LLM reasoning
2. **Given** `tripwire_triggered=True`, **When** guardrail trips, **Then** `InputGuardrailTripwireTriggered` raised
3. **Given** exception caught, **When** handling, **Then** `e.guardrail_result.output_info` contains structured output

---

### User Story 8 - SQLite Sessions with History (Priority: P2)

**Concrete Pattern**: `memory/sqlite_session_example.py`

```python
session = SQLiteSession("user_123")
result = await Runner.run(agent, "What city is the Golden Gate Bridge in?", session=session)
result = await Runner.run(agent, "What state is it in?", session=session)  # Remembers context
```

**Acceptance Scenarios**:
1. **Given** `session=SQLiteSession(id)`, **When** multiple runs with same session, **Then** history automatic
2. **Given** `session.get_items(limit=2)`, **When** called, **Then** returns last 2 conversation items
3. **Given** file-based session, **When** process restarts, **Then** conversation persists

---

### User Story 9 - Advanced Sessions with Branching (Priority: P3)

**Concrete Pattern**: `memory/advanced_sqlite_session_example.py`

```python
session = AdvancedSQLiteSession(session_id="conversation", create_tables=True)
await session.store_run_usage(result)  # Track tokens
branch_id = await session.create_branch_from_turn(2)  # Branch conversation
await session.switch_to_branch("main")  # Switch back
```

**Acceptance Scenarios**:
1. **Given** `store_run_usage(result)`, **When** called after runs, **Then** token usage tracked per turn
2. **Given** `create_branch_from_turn(2)`, **When** new messages added, **Then** branch has independent history
3. **Given** `get_conversation_by_turns()`, **When** called, **Then** returns structured turn data

---

### User Story 10 - Lifecycle Hooks (Priority: P2)

**Concrete Pattern**: `basic/lifecycle_example.py`

```python
class ExampleHooks(RunHooks):
    async def on_agent_start(self, context, agent): ...
    async def on_llm_start(self, context, agent, system_prompt, input_items): ...
    async def on_llm_end(self, context, agent, response): ...
    async def on_tool_start(self, context, agent, tool): ...
    async def on_tool_end(self, context, agent, tool, result): ...
    async def on_handoff(self, context, from_agent, to_agent): ...
    async def on_agent_end(self, context, agent, output): ...

await Runner.run(agent, input, hooks=ExampleHooks())
```

**Acceptance Scenarios**:
1. **Given** `RunHooks` with `on_tool_start`, **When** tool called, **Then** hook fires with tool context
2. **Given** `on_handoff` hook, **When** handoff occurs, **Then** logs source and target agents
3. **Given** `context.usage`, **When** accessed in hook, **Then** contains token counts

---

### User Story 11 - Tracing with Custom Spans (Priority: P2)

**Concrete Pattern**: `agent_patterns/deterministic.py`, `financial_research_agent/manager.py`

```python
trace_id = gen_trace_id()
with trace("Financial research trace", trace_id=trace_id):
    print(f"View: https://platform.openai.com/traces/trace?trace_id={trace_id}")
    with custom_span("Search the web"):
        results = await search(...)
```

**Acceptance Scenarios**:
1. **Given** `gen_trace_id()`, **When** passed to `trace()`, **Then** can link to dashboard URL
2. **Given** `group_id` parameter, **When** multiple traces share it, **Then** grouped in dashboard
3. **Given** `custom_span("name")`, **When** nested in trace, **Then** appears as sub-operation

---

### User Story 12 - MCP Integration (Priority: P3)

**Concrete Pattern**: `mcp/filesystem_example/main.py`

```python
async with MCPServerStdio(
    name="Filesystem Server",
    params={"command": "npx", "args": ["-y", "@modelcontextprotocol/server-filesystem", path]},
) as server:
    agent = Agent(name="Assistant", mcp_servers=[server])
```

**Acceptance Scenarios**:
1. **Given** `MCPServerStdio` context manager, **When** agent runs, **Then** MCP tools available
2. **Given** `mcp_servers=[server]` on agent, **When** user asks to "list files", **Then** MCP tool called
3. **Given** multiple MCP servers, **When** configured, **Then** all tools available to agent

---

### User Story 13 - Full Customer Support FTE (Priority: P1)

**Concrete Pattern**: `customer_service/main.py` - complete implementation

**Components**:
- Context: `AirlineAgentContext` with passenger_name, confirmation_number, seat_number, flight_number
- Tools: `faq_lookup_tool` (with keyword matching), `update_seat` (with context access)
- Agents: `triage_agent`, `faq_agent`, `seat_booking_agent`
- Handoffs: Bidirectional (specialists can hand back to triage)
- Hooks: `on_seat_booking_handoff` for context injection
- Tracing: `group_id=conversation_id` for linking turns
- Loop: Full conversation loop with `to_input_list()` and `result.last_agent`

**Acceptance Scenarios**:
1. **Given** FAQ question, **When** triage receives, **Then** hands off to FAQ agent
2. **Given** seat booking request, **When** triage receives, **Then** handoff callback sets flight_number
3. **Given** off-topic question to specialist, **When** detected, **Then** hands back to triage
4. **Given** entire conversation, **When** traced, **Then** all turns visible under same group_id

---

## Requirements *(mandatory)*

### Functional Requirements

#### Lesson 1: SDK Setup & First Agent
- **FR-001**: MUST show `pip install openai-agents` and `pip install "openai-agents[litellm]"`
- **FR-002**: MUST demonstrate `OPENAI_API_KEY` configuration
- **FR-003**: MUST implement basic Agent → Runner.run_sync() pattern
- **FR-004**: MUST show LitellmModel with Anthropic and OpenAI-via-LiteLLM
- **FR-005**: MUST explain `set_tracing_disabled(True)` for non-OpenAI

#### Lesson 2: Function Tools & Context Objects
- **FR-006**: MUST implement @function_tool with type hints and docstrings
- **FR-007**: MUST create Pydantic context model (like AirlineAgentContext)
- **FR-008**: MUST demonstrate `RunContextWrapper[ContextType]` in tool signature
- **FR-009**: MUST show context mutations persisting across tool calls
- **FR-010**: MUST implement TaskManager with context tracking

#### Lesson 3: Agents as Tools & Orchestration
- **FR-011**: MUST demonstrate `agent.as_tool(tool_name, tool_description)`
- **FR-012**: MUST implement `custom_output_extractor` for structured outputs
- **FR-013**: MUST show orchestrator pattern (manager calls sub-agents)
- **FR-014**: MUST demonstrate `agent.clone(tools=[...])` for dynamic composition
- **FR-015**: MUST contrast with handoff pattern (who owns conversation)

#### Lesson 4: Handoffs & Message Filtering
- **FR-016**: MUST implement basic handoffs list: `handoffs=[agent1, agent2]`
- **FR-017**: MUST demonstrate `handoff(agent, on_handoff=callback)` pattern
- **FR-018**: MUST implement `input_filter` with `HandoffInputData`
- **FR-019**: MUST show `handoff_filters.remove_all_tools` usage
- **FR-020**: MUST demonstrate bidirectional handoffs (specialist → triage)

#### Lesson 5: Guardrails & Agent-Based Validation
- **FR-021**: MUST implement `@input_guardrail` with tripwire
- **FR-022**: MUST implement `@output_guardrail` with structured output check
- **FR-023**: MUST demonstrate agent-based guardrail (guardrail calls agent)
- **FR-024**: MUST handle `InputGuardrailTripwireTriggered` exception
- **FR-025**: MUST show practical PII/topic detection example

#### Lesson 6: Sessions & Conversation Memory
- **FR-026**: MUST implement `SQLiteSession(session_id)` basic usage
- **FR-027**: MUST show file-based persistence: `SQLiteSession(id, "file.db")`
- **FR-028**: MUST demonstrate `session.get_items(limit=N)`
- **FR-029**: MUST implement `AdvancedSQLiteSession` with usage tracking
- **FR-030**: MUST show conversation branching with `create_branch_from_turn()`

#### Lesson 7: Tracing, Hooks & Observability
- **FR-031**: MUST implement `RunHooks` with all lifecycle methods
- **FR-032**: MUST demonstrate `gen_trace_id()` and dashboard URL construction
- **FR-033**: MUST implement `custom_span()` for sub-operations
- **FR-034**: MUST show `group_id` for linking conversation turns
- **FR-035**: MUST track `context.usage` for token monitoring

#### Lesson 8: Capstone - Customer Support FTE
- **FR-036**: MUST implement full customer_service pattern from repo
- **FR-037**: MUST include: context object, tools with context, handoff callbacks
- **FR-038**: MUST implement bidirectional handoffs between 3+ agents
- **FR-039**: MUST add input guardrail for abuse detection
- **FR-040**: MUST trace entire conversation with group_id
- **FR-041**: MUST implement full conversation loop with `to_input_list()`

---

### Key Production Patterns (from GitHub examples)

| Pattern | Source File | Key Learning |
|---------|-------------|--------------|
| Context Objects | `customer_service/main.py` | State across agents via Pydantic |
| Agents as Tools | `agent_patterns/agents_as_tools.py` | Sub-agents without ownership transfer |
| Custom Extractors | `financial_research_agent/manager.py` | Control what orchestrator sees |
| Agent Cloning | `financial_research_agent/manager.py` | Runtime agent composition |
| Handoff Callbacks | `customer_service/main.py` | Inject context before handoff |
| Message Filters | `handoffs/message_filter.py` | Clean history on handoff |
| Agent Guardrails | `agent_patterns/input_guardrails.py` | LLM-powered validation |
| Session Branching | `memory/advanced_sqlite_session_example.py` | Conversation editing |
| Lifecycle Hooks | `basic/lifecycle_example.py` | Monitor every event |
| Tracing Spans | `financial_research_agent/manager.py` | Sub-operation visibility |
| MCP Integration | `mcp/filesystem_example/main.py` | External tool ecosystems |
| Conversation Loop | `customer_service/main.py` | Production chat pattern |

---

## Success Criteria

- **SC-001**: Students can implement context objects that persist across tools and agents
- **SC-002**: Students can build orchestrator with 3+ sub-agents as tools
- **SC-003**: Students can implement handoff with callback that injects runtime data
- **SC-004**: Students can implement agent-based guardrail with structured output
- **SC-005**: Students can use AdvancedSQLiteSession with branching
- **SC-006**: Students can implement full RunHooks with usage tracking
- **SC-007**: Students can trace multi-agent workflow with custom spans
- **SC-008**: Capstone matches production quality of customer_service example

---

## Dependencies

- **Chapter 33**: Conceptual foundation (taxonomy, architecture, patterns)
- **Part 5**: Python async, Pydantic, type hints
- **Part 4**: SDD-RI workflow for capstone
- **Skill**: `building-with-openai-agents` provides API reference
- **GitHub Repo**: `openai/openai-agents-python/examples/` as pattern source

## Out of Scope

- Voice/realtime agents (specialized extension)
- Deployment infrastructure (Part 7)
- Custom model training
- Enterprise SSO/auth patterns
