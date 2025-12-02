# ADR-0005: ChatKit Framework Integration Architecture

> **Scope**: Integration architecture for OpenAI ChatKit framework with custom backend and AI agents. Includes framework extension pattern, persistence strategy, agent integration approach, and streaming response handling. These decisions affect how conversational AI is implemented across the platform.

- **Status:** Accepted
- **Date:** 2025-01-27
- **Feature:** 007-chatkit-server, 008-chatkit-ui-widget
- **Context:** RoboLearn platform needs conversational AI interface for educational assistance. Two architectural options emerged: (1) build custom chat system from scratch, or (2) integrate OpenAI ChatKit framework. ChatKit chosen to leverage standardized protocol, pre-built React components, and proven conversation management. System must integrate with existing RAG search agent, maintain conversation history, handle user/page context, and scale horizontally. Framework requires custom integration for agent execution, context injection, and authentication.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Defines conversational AI architecture for entire platform, affects all future chat features
     2) Alternatives: ✅ Custom implementation vs ChatKit framework, PostgreSQL vs Redis/Memory, streaming vs batch
     3) Scope: ✅ Cross-cutting decision affecting backend, frontend, database, and agent integration
-->

## Decision

Integrate **OpenAI ChatKit Framework** with the following architecture cluster:

- **Framework**: OpenAI ChatKit SDK (`chatkit` package) for protocol handling and React components
- **Backend Pattern**: Extend `ChatKitServer[RequestContext]` class, override `respond()` method for custom agent execution
- **Persistence**: PostgreSQL 14+ via async SQLAlchemy with connection pooling
- **Agent Integration**: OpenAI Agents SDK with custom tools (RAG search), context injection in system prompt
- **Response Handling**: Server-Sent Events (SSE) streaming for real-time UX
- **Context Transmission**: User info and page context via request metadata, extracted in backend and included in agent prompt

## Consequences

### Positive

- **Standardized Protocol**: ChatKit provides proven conversation management protocol, reducing custom protocol development
- **Pre-built UI Components**: React components reduce frontend development time, ensure consistent UX
- **Attachment Handling**: Built-in support for file uploads/downloads without custom implementation
- **Extensibility**: Can integrate any agent framework via `respond()` method override
- **Scalability**: PostgreSQL handles concurrent conversations, connection pooling optimizes performance
- **Real-time UX**: Streaming responses provide immediate feedback, better perceived performance
- **Framework Agnostic**: Backend pattern works with any agent framework (OpenAI Agents SDK, LangChain, custom)

### Negative

- **Dependency on ChatKit SDK**: Vendor lock-in to OpenAI's ChatKit framework, protocol constraints
- **Database Dependency**: Requires PostgreSQL setup and maintenance, connection management complexity
- **Prompt Length Growth**: History included as string in prompt, grows with conversation length (mitigated by limit=10)
- **Streaming Complexity**: More complex error handling and connection management than batch responses
- **Learning Curve**: Team must learn ChatKit protocol and extension patterns
- **Framework Updates**: ChatKit SDK updates may require code changes

## Alternatives Considered

**Alternative Stack A: Custom Chat Implementation**
- Build custom chat protocol, React components, and persistence layer from scratch
- **Why rejected**: Significant development time, no standardized UX, must handle all edge cases (attachments, threading, persistence)

**Alternative Stack B: ChatKit + Redis/Memory Storage**
- Use ChatKit framework but store conversations in Redis or in-memory
- **Why rejected**: Conversations lost on restart (Redis) or server restart (memory), no persistence for user experience

**Alternative Stack C: ChatKit + Batch Responses**
- Use ChatKit but return complete responses instead of streaming
- **Why rejected**: Poor UX (users wait for complete response), slower perceived performance

**Alternative Stack D: LangChain Chat Integration**
- Use LangChain's chat integration instead of ChatKit
- **Why rejected**: No standardized React components, more complex setup, less proven for production use

## References

- Feature Spec: `specs/007-chatkit-server/spec.md` (ADR-001, ADR-002, ADR-004)
- Implementation Plan: `specs/007-chatkit-server/plan.md` (Architecture Overview, Technology Stack)
- Related ADRs: ADR-0006 (Context Injection Pattern)
- Evaluator Evidence: `history/prompts/007-chatkit-server/0001-chatkit-server-reverse-engineering.spec.prompt.md`
