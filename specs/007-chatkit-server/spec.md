# ChatKit Server Specification

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-01-XX
**Source**: `rag-agent/chatkit_server.py`, `rag-agent/app.py`, `rag-agent/lifespan.py`

## Problem Statement

The RoboLearn platform needs a conversational AI interface that:
- Integrates OpenAI's ChatKit framework for conversation management
- Provides persistent conversation history across sessions
- Integrates with existing RAG search capabilities
- Maintains user context (profile, page context, conversation history)
- Handles file attachments (images, PDFs) for educational content
- Scales horizontally with PostgreSQL-backed persistence

**Why this exists**: ChatKit provides a standardized conversation framework, but needs custom integration to work with RoboLearn's RAG agent and educational content.

## System Intent

**Target Users**: 
- Students using RoboLearn educational platform
- Backend developers integrating ChatKit with custom agents

**Core Value Proposition**: 
- Standardized conversation UI/UX via ChatKit React components
- Persistent conversation history with PostgreSQL
- Seamless integration with RAG search for educational content
- Context-aware responses using user profile and page information

**Key Capabilities**:
- Conversation thread management (create, list, retrieve)
- Message persistence (user messages, assistant responses)
- Streaming responses for real-time chat experience
- File attachment handling (images, PDFs)
- User context injection (profile, page context, conversation history)
- RAG tool integration for educational content search

## Functional Requirements

### Requirement 1: ChatKit Endpoint Handler

**What**: HTTP endpoint that processes ChatKit protocol requests

**Why**: ChatKit React components communicate via standardized protocol; backend must handle all ChatKit operations

**Inputs**: 
- HTTP POST request to `/chatkit`
- ChatKit protocol payload (JSON)
- Headers: `X-User-ID` (required), `X-Request-ID` (optional)

**Outputs**: 
- Streaming response (text/event-stream) for agent operations
- JSON response for read-only operations (threads.list, items.list)

**Side Effects**: 
- Creates/updates threads in PostgreSQL
- Creates message items in PostgreSQL
- Triggers agent execution for user messages

**Success Criteria**: 
- All ChatKit protocol operations handled correctly
- Streaming responses delivered in real-time
- Read-only operations return correct data structures

**Evidence**: `rag-agent/app.py:64-128`

### Requirement 2: Conversation Thread Management

**What**: Create, list, and retrieve conversation threads per user

**Why**: Users need multiple conversation contexts (different topics, different sessions)

**Inputs**: 
- User ID (from `X-User-ID` header)
- Thread metadata (optional title, metadata)

**Outputs**: 
- Thread list with metadata
- Individual thread with all messages

**Side Effects**: 
- Creates new thread records in PostgreSQL `chatkit.threads` table
- Updates thread metadata

**Success Criteria**: 
- Threads isolated per user (user_id in context)
- Threads persist across sessions
- Thread listing returns user's threads only

**Evidence**: Handled by base `ChatKitServer` class via `PostgresStore`

### Requirement 3: Message Persistence

**What**: Store user messages and assistant responses in PostgreSQL

**Why**: Conversation history needed for context-aware responses and user experience

**Inputs**: 
- User message content (text, attachments)
- Assistant response (streaming text)
- Thread ID

**Outputs**: 
- Persistent message items in database
- Message retrieval by thread

**Side Effects**: 
- Inserts into PostgreSQL `chatkit.items` table
- Links messages to threads via `thread_id`

**Success Criteria**: 
- All messages stored with correct thread association
- Messages retrievable in chronological order
- Message content preserved (text, attachments)

**Evidence**: `rag-agent/chatkit_server.py:185-192` (loads previous items)

### Requirement 4: Agent Response Generation

**What**: Generate AI assistant responses using OpenAI Agents SDK with RAG search

**Why**: Provide educational assistance using RoboLearn content database

**Inputs**: 
- User message (text + optional attachments)
- Conversation history (last 10 messages)
- User profile information (name, role, software background, hardware tier)
- Page context (current page URL, title, headings, description)

**Outputs**: 
- Streaming text response
- Tool calls to `search_robolearn_content` when needed
- Citations to educational content

**Side Effects**: 
- Calls OpenAI Agents SDK
- Executes RAG search tool
- Stores assistant response in database

**Success Criteria**: 
- Responses are contextually relevant
- RAG search invoked for educational questions
- Responses cite specific lessons/chapters
- Conversation history included in agent prompt

**Evidence**: `rag-agent/chatkit_server.py:224-241`

### Requirement 5: User Context Injection

**What**: Include user profile and page context in agent system prompt

**Why**: Enable personalized, context-aware responses

**Inputs**: 
- User info from request metadata: `{name, email, role, softwareBackground, hardwareTier}`
- Page context from request metadata: `{url, title, path, headings, description}`

**Outputs**: 
- Enhanced system prompt with user and page context

**Side Effects**: 
- Agent receives personalized context
- Responses adapt to user's profile and current page

**Success Criteria**: 
- User name remembered across conversation
- Responses reference current page when relevant
- Hardware tier considered for recommendations

**Evidence**: `rag-agent/chatkit_server.py:152-183`

### Requirement 6: File Attachment Handling

**What**: Process image and file attachments from ChatKit

**Why**: Students may upload screenshots, diagrams, or code files for help

**Inputs**: 
- ChatKit attachment (image or file)
- Upload URL from attachment store

**Outputs**: 
- Base64-encoded data URL for Agent SDK
- Proper format: `ResponseInputImageContentParam` or `ResponseInputFileContentParam`

**Side Effects**: 
- Downloads file from attachment store
- Converts to base64 data URL

**Success Criteria**: 
- Images processed correctly (PNG, JPG, etc.)
- Files processed correctly (PDF, code files)
- Base64 encoding correct
- Agent SDK receives proper format

**Evidence**: `rag-agent/chatkit_server.py:41-88`

### Requirement 7: Conversation History Management

**What**: Load and include previous messages in agent prompt

**Why**: Agent needs conversation context to remember user information and provide coherent responses

**Inputs**: 
- Thread ID
- Request context (for user isolation)

**Outputs**: 
- Last 10 messages (user + assistant pairs)
- Formatted history string for system prompt

**Side Effects**: 
- Queries PostgreSQL for thread items
- Formats history for prompt inclusion

**Success Criteria**: 
- History loaded in reverse chronological order
- Only messages from same thread included
- History formatted correctly for agent prompt
- User's name and preferences remembered

**Evidence**: `rag-agent/chatkit_server.py:185-214`

### Requirement 8: Error Handling

**What**: Gracefully handle errors and return user-friendly messages

**Why**: Users should see helpful error messages, not stack traces

**Inputs**: 
- Exception from agent execution
- Exception from database operations

**Outputs**: 
- Error message item in ChatKit format
- Logged error details for debugging

**Side Effects**: 
- Error logged with full context
- User receives error message

**Success Criteria**: 
- No stack traces exposed to users
- Errors logged with request context
- User receives actionable error message
- System continues operating after error

**Evidence**: `rag-agent/chatkit_server.py:247-262`

## Non-Functional Requirements

### Performance

**Observed Patterns**:
- Connection pooling for PostgreSQL (pre-warmed on startup)
- Streaming responses for real-time UX
- Async/await throughout for non-blocking I/O

**Target**: 
- First request < 2s (after connection warmup)
- Streaming response start < 500ms
- Database queries < 100ms (with connection pool)

**Evidence**: `rag-agent/lifespan.py:76-79` (connection warmup)

### Security

**Observed Patterns**:
- User ID required in header (`X-User-ID`)
- User isolation via `RequestContext`
- Metadata extraction from request (user info, page context)

**Standards**: 
- All operations require authenticated user
- User data isolated per user_id
- No user data leakage across users

**Evidence**: `rag-agent/app.py:83-85` (user ID validation)

### Reliability

**Observed Patterns**:
- Graceful degradation (ChatKit disabled if database unavailable)
- Error handling with user-friendly messages
- Non-blocking initialization (RAG can fail without blocking ChatKit)

**SLA**: 
- System starts even if RAG unavailable
- ChatKit disabled gracefully if database unavailable
- Errors don't crash entire application

**Evidence**: `rag-agent/lifespan.py:51-54` (graceful degradation)

### Scalability

**Observed Patterns**:
- PostgreSQL connection pooling
- Stateless request handling (user_id in header)
- Horizontal scaling ready (no session storage)

**Load Capacity**: 
- Scales horizontally (stateless design)
- Connection pool size configurable
- Database handles concurrent requests

**Evidence**: Stateless design, connection pooling

### Observability

**Observed Patterns**:
- Structured logging with context
- Request ID support for tracing
- Log levels: INFO, WARNING, ERROR

**Monitoring**: 
- All operations logged with user_id
- Request IDs for correlation
- Error logging with full context

**Evidence**: `logger.info()` calls throughout, `rag-agent/app.py:98` (request ID)

## System Constraints

### External Dependencies

- **PostgreSQL**: Required for ChatKit persistence (schema: `chatkit`)
- **OpenAI Agents SDK**: Required for agent execution (`agents` package)
- **ChatKit SDK**: Required for protocol handling (`chatkit` package)
- **FastAPI**: Web framework for HTTP handling
- **Qdrant**: Optional (for RAG search, but ChatKit works without it)

### Data Formats

- **ChatKit Protocol**: JSON payloads following ChatKit specification
- **PostgreSQL**: Async SQLAlchemy for database access
- **Streaming**: Server-Sent Events (SSE) for real-time responses

### Deployment Context

- **FastAPI Application**: Runs via `uvicorn`
- **Database**: PostgreSQL (Neon, Supabase, or self-hosted)
- **Environment Variables**: `DATABASE_URL` or `CHATKIT_STORE_DATABASE_URL`

### Compliance Requirements

- **User Data Isolation**: All operations scoped by user_id
- **Data Persistence**: Conversations stored per user
- **Privacy**: User data not shared across users

## Non-Goals & Out of Scope

**Explicitly excluded** (inferred from missing implementation):
- **Attachment Storage**: Uses external attachment store (S3, Supabase Storage), not implemented in server
- **User Authentication**: Assumes user_id provided by upstream auth system
- **Rate Limiting**: Not implemented (should be added in production)
- **Caching**: No Redis caching layer (CachedPostgresStore available but not used)
- **Multi-tenancy**: Single-tenant design (organization_id in RequestContext but not used)

## Known Gaps & Technical Debt

### Gap 1: Rate Limiting Missing

**Issue**: No rate limiting on `/chatkit` endpoint

**Evidence**: `rag-agent/app.py:64` (no rate limiting middleware)

**Impact**: Vulnerable to abuse, potential cost overruns

**Recommendation**: Add rate limiting middleware (e.g., `slowapi`) with per-user limits

### Gap 2: Authentication Validation Missing

**Issue**: User ID accepted from header without validation

**Evidence**: `rag-agent/app.py:83-85` (only checks presence, not validity)

**Impact**: Security risk if header can be spoofed

**Recommendation**: Validate user_id against auth system or JWT token

### Gap 3: Error Recovery Limited

**Issue**: Errors return generic message, no retry mechanism

**Evidence**: `rag-agent/chatkit_server.py:247-262` (generic error message)

**Impact**: User can't retry failed operations easily

**Recommendation**: Add retry logic for transient failures, more specific error messages

### Gap 4: No Monitoring/Alerting

**Issue**: No metrics collection or alerting

**Evidence**: Only logging, no metrics

**Impact**: Can't detect issues proactively

**Recommendation**: Add Prometheus metrics, health check endpoint, alerting

## Success Criteria

### Functional Success

- [x] ChatKit endpoint handles all protocol operations
- [x] Conversations persist across sessions
- [x] Agent generates context-aware responses
- [x] File attachments processed correctly
- [x] User context included in responses
- [x] Conversation history maintained

### Non-Functional Success

- [x] System starts even if RAG unavailable
- [x] Database connection pooling works
- [x] Streaming responses delivered in real-time
- [ ] Rate limiting implemented (Gap 1)
- [ ] Authentication validation implemented (Gap 2)
- [ ] Metrics and monitoring added (Gap 4)

## Acceptance Tests

### Test 1: Create Thread and Send Message

**Given**: User authenticated with user_id="test-user-123"

**When**: POST `/chatkit` with `threads.create` operation, then `threads.run` with message "What is ROS 2?"

**Then**: 
- Thread created in database with user_id="test-user-123"
- User message stored in database
- Agent response streamed back
- Assistant message stored in database
- Response includes RAG search results

### Test 2: Conversation History Persistence

**Given**: Existing thread with 5 messages

**When**: User sends new message "What is my name?"

**Then**: 
- Agent receives all 5 previous messages in prompt
- Agent remembers user's name from history
- Response addresses user by name

### Test 3: User Context Injection

**Given**: Request with userInfo metadata: `{name: "Alice", hardwareTier: 2}`

**When**: Agent generates response

**Then**: 
- System prompt includes "User Information: Name: Alice, Hardware Tier: 2"
- Agent adapts recommendations to Tier 2 hardware

### Test 4: Page Context Awareness

**Given**: Request with pageContext metadata: `{title: "ROS 2 Basics", url: "/ros2/basics"}`

**When**: User asks "Can you explain this?"

**Then**: 
- System prompt includes page context
- Agent references current page in response

### Test 5: File Attachment Handling

**Given**: User uploads image attachment

**When**: Message sent with attachment

**Then**: 
- Attachment downloaded from store
- Converted to base64 data URL
- Passed to agent in correct format
- Agent processes image correctly

### Test 6: Error Handling

**Given**: Database connection fails

**When**: ChatKit request received

**Then**: 
- Error logged with context
- User receives friendly error message
- System continues operating (other endpoints work)

## Architecture Decisions

### ADR-001: Use ChatKit Framework vs Custom Implementation

**Status**: Accepted

**Context**: Need standardized conversation UI/UX, persistent history, attachment handling

**Decision**: Use OpenAI ChatKit framework

**Rationale**:
1. **Standardized Protocol**: ChatKit provides proven protocol for conversation management
2. **React Components**: Pre-built UI components reduce frontend work
3. **Attachment Handling**: Built-in support for file uploads/downloads
4. **Extensibility**: Can integrate custom agents via `respond()` method

**Consequences**:
- **Positive**: Faster development, standardized UX, proven framework
- **Negative**: Dependency on ChatKit SDK, protocol constraints

**Evidence**: `rag-agent/chatkit_server.py:100` (extends `ChatKitServer`)

### ADR-002: PostgreSQL for Persistence vs Redis/Memory

**Status**: Accepted

**Context**: Need persistent conversation history across sessions

**Decision**: Use PostgreSQL for thread and message storage

**Rationale**:
1. **Persistence**: Conversations must survive server restarts
2. **Relational Data**: Threads and messages have relationships
3. **Query Capabilities**: Need to query by user, thread, date
4. **Scalability**: PostgreSQL handles concurrent writes well

**Consequences**:
- **Positive**: Persistent, queryable, scalable
- **Negative**: Database dependency, connection management needed

**Evidence**: `rag-agent/chatkit_store/postgres_store.py` (PostgreSQL implementation)

### ADR-003: Include History in Prompt vs Pass as Messages

**Status**: Accepted

**Context**: Agent needs conversation history for context-aware responses

**Decision**: Include history as string in system prompt (like CarFixer pattern)

**Rationale**:
1. **Simplicity**: Single prompt with all context
2. **Proven Pattern**: Matches CarFixer implementation
3. **Agent SDK Compatibility**: Works with current SDK version

**Consequences**:
- **Positive**: Simple, works reliably
- **Negative**: Prompt length grows with history (mitigated by limit=10)

**Evidence**: `rag-agent/chatkit_server.py:212-214` (history string in prompt)

### ADR-004: Streaming Responses vs Batch

**Status**: Accepted

**Context**: Real-time chat UX requires immediate feedback

**Decision**: Use streaming responses (Server-Sent Events)

**Rationale**:
1. **UX**: Users see responses as they're generated
2. **Perceived Performance**: Faster feel than waiting for complete response
3. **ChatKit Support**: Framework supports streaming natively

**Consequences**:
- **Positive**: Better UX, faster perceived performance
- **Negative**: More complex error handling, connection management

**Evidence**: `rag-agent/app.py:109-118` (StreamingResponse)

## Implementation Notes

### Key Files

- **`rag-agent/chatkit_server.py`**: Main ChatKit server implementation
- **`rag-agent/app.py`**: FastAPI endpoint handler
- **`rag-agent/lifespan.py`**: Application lifecycle (database initialization)
- **`rag-agent/state.py`**: Agent context definition
- **`rag-agent/prompts.py`**: System prompt for agent
- **`rag-agent/chatkit_store/`**: PostgreSQL store implementation

### Integration Points

1. **RAG Search Tool**: `rag-agent/rag/tools.py` - `search_tool` function
2. **PostgreSQL Store**: `rag-agent/chatkit_store/postgres_store.py`
3. **Agent SDK**: `agents` package - `Agent`, `Runner`, `stream_agent_response`
4. **ChatKit SDK**: `chatkit` package - `ChatKitServer`, types, protocol

### Configuration

**Environment Variables**:
- `DATABASE_URL` or `CHATKIT_STORE_DATABASE_URL`: PostgreSQL connection string
- `OPENAI_API_KEY`: Required for agent execution
- `QDRANT_URL`, `QDRANT_API_KEY`: Optional (for RAG search)

**Database Schema**: Auto-created by `PostgresStore.initialize_schema()`
- Schema: `chatkit`
- Tables: `threads`, `items`, `attachments`

