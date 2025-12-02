# ADR-0006: Context Injection Pattern for Conversational AI

> **Scope**: Pattern for injecting user profile, page context, and conversation history into conversational AI agent prompts. Includes client-side context extraction, request metadata transmission, backend extraction, and prompt construction. This pattern affects how agents understand user context and provide personalized responses.

- **Status:** Accepted
- **Date:** 2025-01-27
- **Feature:** 007-chatkit-server, 008-chatkit-ui-widget
- **Context:** Conversational AI agents need context to provide personalized, relevant responses. Three types of context are available: (1) user profile (name, role, hardware tier, preferences), (2) page context (current page URL, title, headings, description), and (3) conversation history (previous messages). Multiple approaches considered: (1) pass context as separate messages, (2) include in system prompt, (3) use agent memory/state. System prompt inclusion chosen (CarFixer pattern) for simplicity and reliability. Context must be extracted client-side (page info), transmitted via request metadata, extracted in backend, and included in agent prompt.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Defines how agents receive context, affects personalization and relevance of all responses
     2) Alternatives: ✅ Separate messages vs system prompt, client-side vs server-side extraction, different transmission methods
     3) Scope: ✅ Cross-cutting pattern affecting frontend, backend, and agent integration
-->

## Decision

Use **Context Injection via System Prompt** pattern with the following components:

- **History Pattern**: Include conversation history as formatted string in system prompt (CarFixer pattern), load last 10 messages from thread
- **User Context**: Extract user profile from authenticated session, include in request metadata, extract in backend, format as string in prompt
- **Page Context**: Extract page metadata client-side (URL, title, headings, meta tags), include in request metadata, extract in backend, format as string in prompt
- **Prompt Construction**: Combine history + user context + page context + system prompt into single prompt string
- **Transmission Method**: Request metadata (JSON in ChatKit protocol body) for user/page context, database query for history

## Consequences

### Positive

- **Simplicity**: Single prompt with all context, no complex state management
- **Proven Pattern**: Matches CarFixer implementation, works reliably
- **Agent SDK Compatibility**: Works with current OpenAI Agents SDK version
- **Client-Side Rich Context**: Can extract detailed page metadata (headings, meta tags) that server can't access
- **No Extra Requests**: Context included in same request, no additional round-trips
- **Framework Agnostic**: Pattern works with any agent framework that accepts system prompts

### Negative

- **Prompt Length Growth**: Prompt length increases with history (mitigated by limit=10 messages)
- **String Formatting**: Must format context as strings, less structured than separate parameters
- **Client Dependency**: Page context only available if client sends it (could be spoofed)
- **No Incremental Updates**: Full context rebuilt on each request (not incremental)
- **Token Usage**: Context included in every request, increases token usage

## Alternatives Considered

**Alternative Pattern A: Pass Context as Separate Messages**
- Include user/page context as separate user messages in conversation
- **Why rejected**: Pollutes conversation history, context appears as user messages, less clean

**Alternative Pattern B: Agent Memory/State**
- Use agent SDK's built-in memory/state management for context
- **Why rejected**: Not available in current SDK version, would require SDK updates

**Alternative Pattern C: Server-Side Page Context Detection**
- Detect page context server-side using URL or referrer header
- **Why rejected**: Server can't access DOM (headings, meta tags), less rich context

**Alternative Pattern D: Context in Query Parameters**
- Pass context via URL query parameters instead of request body
- **Why rejected**: Exposes context in URLs/logs, less secure, doesn't work with ChatKit protocol

**Alternative Pattern E: Incremental Context Updates**
- Only send changed context, maintain context state server-side
- **Why rejected**: More complex state management, requires context diffing, not needed for current use case

## References

- Feature Spec: `specs/007-chatkit-server/spec.md` (Requirement 5: User Context Injection, Requirement 7: Conversation History Management)
- Feature Spec: `specs/008-chatkit-ui-widget/spec.md` (Requirement 4: Page Context Extraction, Requirement 6: User Profile Context Transmission)
- Implementation Plan: `specs/007-chatkit-server/plan.md` (Pattern 2: Context Injection, Pattern 3: History in Prompt)
- Related ADRs: ADR-0005 (ChatKit Framework Integration Architecture)
- Evaluator Evidence: `history/prompts/007-chatkit-server/0001-chatkit-server-reverse-engineering.spec.prompt.md`, `history/prompts/008-chatkit-ui-widget/0001-chatkit-ui-widget-reverse-engineering.spec.prompt.md`
