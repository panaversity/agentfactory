# ChatKit Server Implementation Plan

**Version**: 1.0 (Reverse Engineered)
**Date**: 2025-01-XX

## Architecture Overview

**Architectural Style**: Layered Architecture with Framework Integration

**Reasoning**: 
- ChatKit framework provides protocol handling and base functionality
- Custom layer extends framework for RAG agent integration
- Clear separation: Framework (ChatKit) → Custom Logic (RoboLearn) → Agent SDK

**Diagram**:
```
┌─────────────────────────────────────────┐
│         FastAPI Application             │
│  ┌───────────────────────────────────┐  │
│  │   POST /chatkit Endpoint         │  │
│  │   - Extract user_id from header  │  │
│  │   - Extract metadata from body   │  │
│  │   - Create RequestContext        │  │
│  └──────────────┬────────────────────┘  │
└─────────────────┼────────────────────────┘
                  │
┌─────────────────▼────────────────────────┐
│    RoboLearnChatKitServer                │
│  ┌───────────────────────────────────┐  │
│  │   respond() method               │  │
│  │   - Load conversation history    │  │
│  │   - Extract user/page context    │  │
│  │   - Build agent prompt           │  │
│  │   - Run agent with streaming     │  │
│  └──────────────┬────────────────────┘  │
└─────────────────┼────────────────────────┘
                  │
┌─────────────────▼────────────────────────┐
│         OpenAI Agents SDK                │
│  ┌───────────────────────────────────┐  │
│  │   Agent with search_tool          │  │
│  │   - Executes RAG search           │  │
│  │   - Generates responses           │  │
│  │   - Streams results               │  │
│  └──────────────┬────────────────────┘  │
└─────────────────┼────────────────────────┘
                  │
┌─────────────────▼────────────────────────┐
│      PostgreSQL Store                    │
│  ┌───────────────────────────────────┐  │
│  │   PostgresStore                   │  │
│  │   - Thread persistence            │  │
│  │   - Message persistence           │  │
│  │   - User isolation                │  │
│  └───────────────────────────────────┘  │
└──────────────────────────────────────────┘
```

## Layer Structure

### Layer 1: HTTP/API Layer

**Responsibility**: Handle HTTP requests, extract context, route to ChatKit server

**Components**:
- `app.py`: FastAPI application, `/chatkit` endpoint
- Request parsing: Extract user_id, metadata, payload
- Response formatting: StreamingResponse or JSON Response

**Dependencies**: → ChatKit Server Layer

**Technology**: FastAPI, Python

**Evidence**: `rag-agent/app.py:64-128`

### Layer 2: ChatKit Server Layer

**Responsibility**: Process ChatKit protocol, manage conversation state, trigger agent

**Components**:
- `chatkit_server.py`: `RoboLearnChatKitServer` class
- `respond()`: Main agent execution logic
- Thread item converter: Attachment handling
- Context extraction: User info, page context, conversation history

**Dependencies**: → Agent SDK, → PostgreSQL Store

**Technology**: ChatKit SDK, Python async/await

**Evidence**: `rag-agent/chatkit_server.py:100-270`

### Layer 3: Agent Execution Layer

**Responsibility**: Execute AI agent with RAG search tool

**Components**:
- `state.py`: `RoboLearnAgentContext` definition
- `prompts.py`: System prompt for agent
- `rag/tools.py`: `search_tool` function for RAG search
- Agent creation: `Agent` with tools and instructions

**Dependencies**: → OpenAI Agents SDK, → RAG Search

**Technology**: OpenAI Agents SDK, Python

**Evidence**: `rag-agent/chatkit_server.py:224-241`

### Layer 4: Data/Persistence Layer

**Responsibility**: Store conversations, threads, messages

**Components**:
- `chatkit_store/postgres_store.py`: PostgreSQL implementation
- `chatkit_store/context.py`: `RequestContext` for user isolation
- Schema management: Auto-create tables on startup
- Connection pooling: Pre-warmed connections

**Dependencies**: → PostgreSQL Database

**Technology**: SQLAlchemy (async), PostgreSQL

**Evidence**: `rag-agent/lifespan.py:43-87`, `rag-agent/chatkit_store/postgres_store.py`

## Design Patterns Applied

### Pattern 1: Framework Extension

**Location**: `rag-agent/chatkit_server.py:100`

**Purpose**: Extend ChatKit framework with custom agent logic

**Implementation**: 
- Inherit from `ChatKitServer[RequestContext]`
- Override `respond()` method for custom agent execution
- Base class handles read-only operations (threads.list, items.list)

**Evidence**: `rag-agent/chatkit_server.py:100-117`

### Pattern 2: Context Injection

**Location**: `rag-agent/chatkit_server.py:152-183`

**Purpose**: Inject user and page context into agent prompt

**Implementation**:
- Extract context from `RequestContext.metadata`
- Build context strings (user_context_str, page_context_str)
- Include in agent system prompt

**Evidence**: `rag-agent/chatkit_server.py:152-183`, `229`

### Pattern 3: History in Prompt

**Location**: `rag-agent/chatkit_server.py:185-214`

**Purpose**: Include conversation history in agent prompt (CarFixer pattern)

**Implementation**:
- Load last 10 messages from thread
- Format as role:content string
- Include in system prompt

**Evidence**: `rag-agent/chatkit_server.py:185-214`

### Pattern 4: Attachment Converter

**Location**: `rag-agent/chatkit_server.py:41-88`

**Purpose**: Convert ChatKit attachments to Agent SDK format

**Implementation**:
- Download file from attachment store URL
- Convert to base64 data URL
- Return appropriate format (image vs file)

**Evidence**: `rag-agent/chatkit_server.py:41-88`

### Pattern 5: Graceful Degradation

**Location**: `rag-agent/lifespan.py:51-54`

**Purpose**: System starts even if dependencies unavailable

**Implementation**:
- Check for database URL
- Log warning if unavailable
- Continue startup (ChatKit disabled but app runs)

**Evidence**: `rag-agent/lifespan.py:51-54`

## Data Flow

### Request Flow (User Message)

1. **HTTP Request**: POST `/chatkit` with ChatKit protocol payload
2. **Context Extraction**: Extract user_id from header, metadata from body
3. **RequestContext Creation**: Create context with user_id, metadata
4. **ChatKit Server**: Process payload, route to `respond()` for user messages
5. **History Loading**: Load last 10 messages from thread
6. **Context Building**: Extract user info, page context, format history
7. **Agent Creation**: Create Agent with search_tool, enhanced prompt
8. **Attachment Conversion**: Convert attachments to Agent SDK format
9. **Agent Execution**: Run agent with streaming
10. **Response Streaming**: Stream events back to client
11. **Message Persistence**: Store assistant response in database

### Read-Only Operations Flow

1. **HTTP Request**: POST `/chatkit` with read-only operation (threads.list, items.list)
2. **ChatKit Server**: Base class handles automatically
3. **PostgreSQL Query**: Query threads/items with user isolation
4. **JSON Response**: Return data structure

## Technology Stack

### Language & Runtime

- **Primary**: Python 3.11+
- **Rationale**: FastAPI ecosystem, async/await support, rich libraries

### Web Framework

- **Choice**: FastAPI 0.100+
- **Rationale**: Async support, automatic OpenAPI docs, type hints

### Database

- **Choice**: PostgreSQL 14+ (via asyncpg)
- **Rationale**: ACID compliance, JSON support, connection pooling, proven reliability

### ChatKit Framework

- **Choice**: OpenAI ChatKit SDK (`chatkit` package)
- **Rationale**: Standardized protocol, React component support, attachment handling

### Agent Framework

- **Choice**: OpenAI Agents SDK (`agents` package)
- **Rationale**: Tool integration, streaming support, context management

### ORM/Database Access

- **Choice**: SQLAlchemy 2.0+ (async)
- **Rationale**: Async support, connection pooling, schema management

### Testing

- **Choice**: pytest, pytest-asyncio
- **Rationale**: Async test support, fixtures, good ecosystem

## Module Breakdown

### Module: chatkit_server

**Purpose**: Main ChatKit server implementation

**Key Classes**: 
- `RoboLearnChatKitServer`: Extends ChatKitServer
- `RoboLearnThreadItemConverter`: Handles attachments

**Dependencies**: 
- ChatKit SDK
- Agents SDK
- RAG tools
- PostgreSQL store

**Complexity**: High

**Evidence**: `rag-agent/chatkit_server.py`

### Module: app (FastAPI)

**Purpose**: HTTP endpoint handler

**Key Functions**:
- `chatkit_endpoint()`: Main endpoint handler
- `get_request_context()`: Context extraction (unused, inline now)

**Dependencies**: 
- ChatKit server
- RequestContext

**Complexity**: Medium

**Evidence**: `rag-agent/app.py`

### Module: lifespan

**Purpose**: Application lifecycle management

**Key Functions**:
- `lifespan()`: Startup/shutdown logic
- Database initialization
- Connection warmup

**Dependencies**: 
- PostgreSQL store
- ChatKit server
- Settings

**Complexity**: Medium

**Evidence**: `rag-agent/lifespan.py`

### Module: chatkit_store

**Purpose**: PostgreSQL persistence layer

**Key Classes**:
- `PostgresStore`: Main store implementation
- `RequestContext`: User isolation context
- `StoreConfig`: Configuration

**Dependencies**: 
- SQLAlchemy
- PostgreSQL

**Complexity**: High

**Evidence**: `rag-agent/chatkit_store/`

### Module: state

**Purpose**: Agent context definition

**Key Classes**:
- `RoboLearnAgentContext`: Extends AgentContext

**Dependencies**: 
- Agents SDK
- RequestContext

**Complexity**: Low

**Evidence**: `rag-agent/state.py`

### Module: prompts

**Purpose**: System prompt for agent

**Key Constants**:
- `ROBOLEARN_SYSTEM`: Main system prompt

**Dependencies**: None

**Complexity**: Low

**Evidence**: `rag-agent/prompts.py`

## Regeneration Strategy

### Option 1: Specification-First Rebuild

1. Start with spec.md (requirements and architecture)
2. Apply extracted patterns (context injection, history in prompt)
3. Implement with improvements:
   - Add rate limiting
   - Add authentication validation
   - Add metrics/monitoring
   - Improve error handling
4. Test-driven development using acceptance criteria

**Timeline**: 2-3 weeks (single developer)

### Option 2: Incremental Enhancement

1. **Keep existing implementation** (works well)
2. **Add missing features**:
   - Rate limiting middleware
   - Authentication validation
   - Metrics collection
   - Better error messages
3. **Improve observability**:
   - Structured logging
   - Health checks
   - Performance metrics

**Timeline**: 1 week (single developer)

## Improvement Opportunities

### Technical Improvements

- [ ] **Add Rate Limiting**
  - **Addresses Gap**: Gap 1 (rate limiting missing)
  - **Effort**: Low (add middleware)
  - **Library**: `slowapi` or `fastapi-limiter`

- [ ] **Add Authentication Validation**
  - **Addresses Gap**: Gap 2 (auth validation missing)
  - **Effort**: Medium (integrate with auth system)
  - **Approach**: Validate JWT token or check auth service

- [ ] **Add Metrics Collection**
  - **Addresses Gap**: Gap 4 (no monitoring)
  - **Effort**: Medium (add Prometheus metrics)
  - **Metrics**: Request rate, latency, error rate, agent execution time

- [ ] **Improve Error Handling**
  - **Addresses Gap**: Gap 3 (error recovery limited)
  - **Effort**: Medium (add retry logic, better messages)
  - **Approach**: Retry transient failures, specific error types

### Architectural Improvements

- [ ] **Add Caching Layer**
  - **Enables**: Faster response times, reduced database load
  - **Effort**: Medium (use CachedPostgresStore with Redis)
  - **Benefit**: Cache thread metadata, recent messages

- [ ] **Add Health Check Endpoint**
  - **Enables**: Kubernetes liveness/readiness probes
  - **Effort**: Low (add `/health` endpoint)
  - **Checks**: Database connectivity, ChatKit server status

### Operational Improvements

- [ ] **Add Structured Logging**
  - **Format**: JSON logs with correlation IDs
  - **Effort**: Low (configure logging formatter)
  - **Benefit**: Better log aggregation and analysis

- [ ] **Add Distributed Tracing**
  - **Tool**: OpenTelemetry
  - **Effort**: Medium (instrument endpoints, agent calls)
  - **Benefit**: Trace requests across services

