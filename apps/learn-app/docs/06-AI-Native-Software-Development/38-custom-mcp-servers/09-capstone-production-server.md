---
sidebar_position: 9
title: "Capstone: Production MCP Server"
description: "Build a complete, production-ready MCP server integrating context, sampling, progress notifications, roots, error handling, and packaging. This is a Layer 4 spec-driven capstone producing a sellable Digital FTE component."
keywords: ["MCP production", "capstone project", "specification-first", "Digital FTE", "distributed reasoning", "server scaling", "error recovery", "packaging"]
chapter: 38
lesson: 9
duration_minutes: 45

# HIDDEN SKILLS METADATA
skills:
  - name: "Orchestrating Advanced MCP Features Into Production Systems"
    proficiency_level: "C2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can write a complete MCP server specification integrating 5+ advanced features and implement it following spec-first methodology"

  - name: "Specification-Driven MCP Server Design"
    proficiency_level: "C2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "System Design"
    measurable_at_this_level: "Student can write a spec.md describing a production server's features, success criteria, and constraints that sufficiently drives implementation without guessing"

  - name: "Evaluating Architectural Tradeoffs in MCP Servers"
    proficiency_level: "C2"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "System Architecture"
    measurable_at_this_level: "Student can decide when to use stateful vs stateless, which tools need sampling vs deterministic operations, and justify scaling choices"

  - name: "Building Composable Digital FTE Components"
    proficiency_level: "C2"
    category: "Applied"
    bloom_level: "Create"
    digcomp_area: "Product Development"
    measurable_at_this_level: "Student can package an MCP server as a Digital FTE component that other systems can integrate and monetize"

learning_objectives:
  - objective: "Design an MCP server specification that integrates context, sampling, progress, roots, and error handling into a coherent system"
    proficiency_level: "C2"
    bloom_level: "Create"
    assessment_method: "Write spec.md for a production research assistant server; evaluate for clarity, constraint definition, and feature integration"

  - objective: "Implement a production MCP server that follows specification-first principles and uses all advanced features appropriately"
    proficiency_level: "C2"
    bloom_level: "Create"
    assessment_method: "Complete working server with sampling, progress notifications, file roots, structured error handling, and packaging"

  - objective: "Evaluate architectural decisions in production servers and justify choices between competing patterns"
    proficiency_level: "C2"
    bloom_level: "Evaluate"
    assessment_method: "Write analysis explaining why certain tools use sampling vs deterministic ops, why roots are configured a certain way, and scaling implications"

  - objective: "Package and test a complete MCP server as a customer-ready Digital FTE component"
    proficiency_level: "C2"
    bloom_level: "Create"
    assessment_method: "Deliverable: installable server package with pyproject.toml, working with Claude Desktop or MCP Inspector"

cognitive_load:
  new_concepts: 12
  assessment: "12 concepts (specification composition, feature orchestration, sampling architecture, progress design, roots security model, error recovery strategy, stateful/stateless tradeoffs, streaming considerations, packaging metadata, Digital FTE monetization, customer readiness criteria, deployment validation) within C2 limit (no artificial ceiling) ✓ - Concepts integrate into coherent production system design"

differentiation:
  extension_for_advanced: "Implement streaming in sampling for large-document processing; design multi-tool coordination patterns; add cost tracking and rate limiting; evaluate horizontal scaling options with multiple server instances"
  remedial_for_struggling: "Start with simplified specification (sampling OR progress, not both); focus on getting one feature working completely before adding others; use provided reference architecture as template"
---

# Capstone: Production MCP Server

You've learned individual advanced features: Context for observability. Sampling for delegating reasoning. Progress for user feedback. Roots for secure file access. Now you'll architect a complete production system that composes all these patterns into something customers would pay for.

This capstone follows **specification-first methodology**—the same approach that builds Digital FTEs. You'll write specifications before implementation. You'll make architectural decisions through constraints, not guesswork. You'll package a production-ready component that could serve thousands of users or integrate into larger systems.

The project: A **Research Assistant Server** that helps users synthesize complex information. The server reads documents from a secure directory, uses AI sampling to generate summaries and synthesis, tracks progress for long-running operations, logs everything for audit trails, and handles failures gracefully. By the end, you'll have a deployable Digital FTE component.

## The Challenge: From Features to Systems

Imagine you're hired to build this for a consulting firm. They need:

- **Input**: Documents (PDF, Markdown) from their research folder
- **Operations**:
  - Search documents for relevant information
  - Use Claude to synthesize findings
  - Create research summaries
  - Track what the server is doing (progress, logs)
- **Constraints**:
  - Never allow users to access files outside the research folder
  - Handle document processing that takes 30+ seconds (client needs feedback)
  - Recover gracefully from API failures (Claude unavailable, file corrupted)
  - Be deployable on their infrastructure (packaging, no hardcoded paths)

Building this requires decisions. Should search be deterministic or use AI sampling? Should file listing accept user paths or only predefined roots? How should long operations notify progress? What errors should retry automatically vs fail immediately? A spec captures these decisions upfront so implementation follows, not guesses.

## Layer 4: Specification-First Design

Layer 4 (from the constitution) means starting with specs, not code. Let's walk through the specification and architectural decisions.

### The Research Assistant Specification

Here's what a production specification looks like:

```markdown
# Research Assistant Server — Specification

## Intent

Build an MCP server that helps domain experts synthesize research from document
collections using AI reasoning, with production-grade observability and error handling.

## Success Criteria

### Functional
- ✓ Search documents by keyword (deterministic, fast)
- ✓ Summarize document using Claude (sampling)
- ✓ Synthesize multiple documents into research summary (sampling + progress)
- ✓ List available research documents
- ✓ Return structured JSON results

### Non-Functional
- ✓ Long operations (>2 seconds) report progress every 10% completion
- ✓ All operations logged to client (info/warning/error levels)
- ✓ Handles API failures gracefully (retry 3x, then error message)
- ✓ All file operations confined to /research directory (roots validation)
- ✓ Supports 10+ simultaneous clients without conflicts
- ✓ Deployable with `pip install` from pyproject.toml

## Tools

### Tool 1: list_research_documents
- **Purpose**: List available documents in research directory
- **Operation**: Deterministic (no sampling)
- **Inputs**:
  - file_pattern (optional): Filter by filename glob
- **Outputs**:
  - documents: list of {path, filename, size_bytes}
- **Error handling**: If directory empty, return empty list (not error)

### Tool 2: search_documents
- **Purpose**: Full-text search across all documents
- **Operation**: Deterministic (no sampling)
- **Inputs**:
  - query: search term
  - limit: max results (default 10)
- **Outputs**:
  - matches: list of {document, line_number, matched_text}
- **Error handling**: Invalid regex → return empty results with warning log

### Tool 3: summarize_document
- **Purpose**: AI-generated summary of single document
- **Operation**: Sampling (uses Claude)
- **Inputs**:
  - document_path: filename in research directory
  - summary_length: "brief" (1 paragraph) | "standard" (3 paragraphs) | "detailed" (5+ paragraphs)
- **Outputs**:
  - document_path, summary, word_count, token_estimate
- **Progress**: 3 steps: load file (33%), prepare prompt (67%), call Claude (100%)
- **Error handling**:
  - File not found → Error with suggestion to list_research_documents first
  - Claude fails 3x → Return error; client can retry

### Tool 4: synthesize_research
- **Purpose**: AI-synthesized summary combining multiple documents
- **Operation**: Sampling + progress (most complex)
- **Inputs**:
  - document_paths: list of filenames
  - research_question: what to synthesize about
  - depth: "overview" | "detailed"
- **Outputs**:
  - synthesis, sources_used, key_insights
- **Progress**: Report per document (N documents = N progress steps)
- **Error handling**:
  - Missing file → Log warning, skip file, continue with others
  - Claude fails → Retry 3x; if all fail, return partial synthesis from documents processed so far

## Constraints & Non-Goals

### What We Build
- Server-side document reading and indexing
- Sampling-based synthesis using Claude
- Progress and logging for observability
- Error recovery and graceful degradation

### What We Don't Build
- Web UI for uploading documents (out of scope)
- Fine-tuned models (use Claude)
- Caching layer (session-based only)
- Horizontal scaling across servers (stateless ready, but not implemented)
- Authentication/authorization (assume client handles)

## Architectural Decisions

### Decision 1: Tool Determinism
- **Search & List**: Deterministic (fast, no tokens spent)
- **Summarize & Synthesize**: Sampling (need reasoning)
- **Rationale**: Minimize Claude usage for simple operations; delegate reasoning to frontier model

### Decision 2: File Access Security
- **Implementation**: Roots restricting to /research directory
- **Validation**: Each path checked against roots before file I/O
- **Rationale**: Prevent directory traversal attacks; allow deployment in untrusted environments

### Decision 3: Stateless Design
- **Session state**: Per-request caching only (no persistent cache)
- **Rationale**: Enables horizontal scaling; each server instance independent

### Decision 4: Error Recovery
- **Sampling failures**: Retry 3x with exponential backoff
- **File failures**: Log and continue (or fail if critical)
- **Rationale**: Network flakiness is normal; graceful degradation better than crashes

## Quality Criteria

- All code functions execute within 60 seconds or report progress
- No unhandled exceptions (all errors return structured JSON-RPC errors)
- All sampling operations logged pre/post with token estimates
- Packaging passes `pip install -e .` without warnings
```

This specification is detailed enough that multiple developers would build compatible systems. It captures **why** decisions matter, not just **what** features exist.

### Architectural Diagram: How Features Integrate

```
┌─────────────────────────────────────────────────────────────┐
│                    Client (Claude Desktop)                   │
├─────────────────────────────────────────────────────────────┤
│  Calls tool → Server → Receives results + progress + logs   │
└─────────────────────────────────────────────────────────────┘
                              ↑
                              │ JSON-RPC
                              │
┌─────────────────────────────────────────────────────────────┐
│              Research Assistant MCP Server                   │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  list_research_documents (Deterministic)                    │
│    ↓ uses Context for logging                              │
│    ↓ uses Roots to validate paths                          │
│    ↓ returns document list                                 │
│                                                              │
│  search_documents (Deterministic)                           │
│    ↓ uses Context for logging                              │
│    ↓ uses Roots to validate file access                    │
│    ↓ returns matches                                        │
│                                                              │
│  summarize_document (Sampling + Progress)                   │
│    ↓ uses Context for logging and progress                 │
│    ↓ uses Roots to validate file access                    │
│    ↓ calls Claude via Context.session.create_message()    │
│    ↓ returns summary                                        │
│                                                              │
│  synthesize_research (Sampling + Progress + Error Recovery) │
│    ↓ uses Context for logging and per-document progress    │
│    ↓ uses Roots for multi-file validation                  │
│    ↓ samples Claude for each document synthesis            │
│    ↓ retries on failure (up to 3x)                         │
│    ↓ returns combined synthesis                            │
│                                                              │
│  Error Handling (All Tools)                                │
│    ↓ Catch all exceptions                                  │
│    ↓ Log to Context.error()                                │
│    ↓ Return structured JSON-RPC error response             │
│    ↓ Never crash the server                                │
│                                                              │
└─────────────────────────────────────────────────────────────┘
         ↓ (filesystem)           ↓ (API)
    ┌────────────────┐     ┌──────────────────┐
    │  /research/    │     │  Claude API      │
    │  documents/    │     │  (via client)    │
    └────────────────┘     └──────────────────┘
```

## Implementation Guidance: From Spec to Code

Here's how to structure the implementation to follow the specification:

### Step 1: Project Setup

```bash
# Create project
uv init research-assistant

# Add dependencies
uv pip install fastmcp pydantic python-dotenv

# Create structure
mkdir -p src/research_assistant
touch src/research_assistant/__init__.py
touch src/research_assistant/server.py
```

### Step 2: File Access Security (Roots)

Before implementing tools, establish security boundaries:

```python
from pathlib import Path

# This comes BEFORE any tool definitions
RESEARCH_DIR = Path.home() / "research"

def validate_path(user_path: str) -> Path:
    """
    Validate that user_path is within RESEARCH_DIR.

    This prevents directory traversal attacks.
    """
    requested = (RESEARCH_DIR / user_path).resolve()

    # Check that resolved path is within research directory
    if not str(requested).startswith(str(RESEARCH_DIR.resolve())):
        raise ValueError(f"Access denied: {user_path}")

    return requested

# This will be used in EVERY tool that accesses files
```

### Step 3: Core Tool Implementation

The simplest tool (deterministic, no sampling):

```python
import fastmcp
from fastmcp import Context
from pydantic import Field
from pathlib import Path

mcp = fastmcp.FastMCP("research_assistant")

@mcp.tool(
    name="list_research_documents",
    description="List documents available in research directory"
)
async def list_research_documents(
    file_pattern: str = Field(
        default="*",
        description="Glob pattern to filter files (e.g., '*.md', '*.pdf')"
    ),
    *,
    context: Context
):
    """List documents with progress logging."""
    await context.info("Scanning research directory...")

    try:
        research_dir = validate_path(".")
        documents = []

        for file_path in sorted(research_dir.glob(file_pattern)):
            if file_path.is_file():
                size = file_path.stat().st_size
                documents.append({
                    "filename": file_path.name,
                    "path": file_path.relative_to(research_dir),
                    "size_bytes": size
                })

        await context.info(f"Found {len(documents)} documents")
        return {
            "documents": documents,
            "count": len(documents),
            "session_id": context.session.session_id
        }

    except Exception as e:
        await context.error(f"Failed to list documents: {str(e)}")
        raise
```

### Step 4: Sampling + Progress (Complex Tool)

This combines multiple features:

```python
import anthropic

@mcp.tool(
    name="summarize_document",
    description="Use Claude to generate a summary of a research document"
)
async def summarize_document(
    document_path: str = Field(description="Path to document (relative to research/)"),
    summary_length: str = Field(
        default="standard",
        description="'brief' (1 para), 'standard' (3 para), or 'detailed' (5+ para)"
    ),
    *,
    context: Context
):
    """
    Summarize a document using Claude.

    This demonstrates:
    - Path validation (Roots)
    - Progress reporting (3 steps)
    - Sampling (calling Claude via client)
    - Error handling (retry + graceful degradation)
    """
    await context.info(f"Starting summarization of {document_path}")

    try:
        # Step 1: Load file (with validation)
        await context.report_progress(1, 3)
        file_path = validate_path(document_path)

        if not file_path.exists():
            raise FileNotFoundError(f"Document not found: {document_path}")

        await context.info(f"Reading document ({file_path.stat().st_size} bytes)")
        with open(file_path, "r") as f:
            content = f.read()

        # Step 2: Prepare prompt
        await context.report_progress(2, 3)
        length_map = {
            "brief": "1 paragraph (100-150 words)",
            "standard": "3 paragraphs (300-500 words)",
            "detailed": "5+ paragraphs (800+ words)"
        }

        prompt = f"""Please summarize the following document in {length_map.get(summary_length, length_map['standard'])}.

Document: {document_path}

Content:
{content}

Provide a clear, structured summary focusing on key insights and main arguments."""

        # Step 3: Call Claude via sampling
        await context.report_progress(3, 3)
        await context.info("Requesting Claude summarization...")

        # This is sampling: server asks client to call Claude
        response = await context.session.create_message(
            model="claude-3-5-sonnet-20241022",
            max_tokens=1500,
            messages=[{
                "role": "user",
                "content": prompt
            }]
        )

        summary_text = response.content[0].text

        await context.info(f"Summarization complete: {len(summary_text)} characters")

        return {
            "document_path": document_path,
            "summary": summary_text,
            "word_count": len(summary_text.split()),
            "token_estimate": len(summary_text) // 4,  # Rough estimate
            "session_id": context.session.session_id
        }

    except FileNotFoundError as e:
        await context.error(f"File not found: {str(e)}")
        raise

    except Exception as e:
        await context.error(f"Summarization failed: {str(e)}")
        raise
```

### Step 5: Error Recovery + Multi-File Processing

The most complex tool combines everything:

```python
import asyncio
from typing import Optional

@mcp.tool(
    name="synthesize_research",
    description="Synthesize research across multiple documents using Claude"
)
async def synthesize_research(
    document_paths: list[str] = Field(
        description="List of document paths relative to research/"
    ),
    research_question: str = Field(
        description="What question should the synthesis answer?"
    ),
    depth: str = Field(
        default="overview",
        description="'overview' (brief synthesis) or 'detailed' (comprehensive)"
    ),
    *,
    context: Context
):
    """
    Synthesize research across documents.

    Demonstrates:
    - Progress reporting (per-document)
    - Multi-step error recovery (retry with backoff)
    - Sampling orchestration (multiple Claude calls)
    - Graceful degradation (skip failed documents)
    """
    await context.info(f"Starting research synthesis: {research_question}")
    await context.info(f"Processing {len(document_paths)} documents at {depth} depth")

    try:
        # Validate all paths upfront
        validated_paths = []
        for doc_path in document_paths:
            try:
                validated = validate_path(doc_path)
                if not validated.exists():
                    await context.warning(f"Document not found, skipping: {doc_path}")
                else:
                    validated_paths.append((doc_path, validated))
            except ValueError as e:
                await context.warning(f"Invalid path, skipping: {doc_path}")

        if not validated_paths:
            raise ValueError("No valid documents found to synthesize")

        # Process each document with progress
        document_summaries = []
        for i, (original_path, file_path) in enumerate(validated_paths):
            await context.report_progress(i + 1, len(validated_paths))
            await context.info(f"Processing: {original_path}")

            try:
                with open(file_path, "r") as f:
                    content = f.read()[:5000]  # First 5k chars to avoid huge prompts

                document_summaries.append({
                    "path": original_path,
                    "content": content
                })

            except Exception as e:
                await context.warning(f"Could not read {original_path}: {str(e)}")
                continue  # Skip this document, continue with others

        # Now synthesize across collected documents
        await context.info(f"Synthesizing across {len(document_summaries)} documents...")

        synthesis_prompt = f"""Based on the following research documents, answer this question:

QUESTION: {research_question}

RESEARCH MATERIALS:
"""

        for doc in document_summaries:
            synthesis_prompt += f"\n\n--- {doc['path']} ---\n{doc['content']}"

        depth_instructions = {
            "overview": "Provide a concise 2-3 paragraph overview",
            "detailed": "Provide a comprehensive 5+ paragraph analysis with detailed insights"
        }

        synthesis_prompt += f"\n\n{depth_instructions.get(depth, depth_instructions['overview'])}"

        # Call Claude with retry logic
        max_retries = 3
        for attempt in range(max_retries):
            try:
                await context.info(f"Calling Claude (attempt {attempt + 1}/{max_retries})...")

                response = await context.session.create_message(
                    model="claude-3-5-sonnet-20241022",
                    max_tokens=2000,
                    messages=[{
                        "role": "user",
                        "content": synthesis_prompt
                    }]
                )

                synthesis = response.content[0].text
                break  # Success, exit retry loop

            except Exception as e:
                if attempt < max_retries - 1:
                    await context.warning(f"Attempt {attempt + 1} failed: {str(e)}, retrying...")
                    await asyncio.sleep(2 ** attempt)  # Exponential backoff
                else:
                    raise  # All retries exhausted

        await context.info("Synthesis complete")

        return {
            "research_question": research_question,
            "synthesis": synthesis,
            "sources_used": len(document_summaries),
            "sources_skipped": len(validated_paths) - len(document_summaries),
            "depth": depth,
            "session_id": context.session.session_id
        }

    except Exception as e:
        await context.error(f"Synthesis failed: {str(e)}")
        raise

if __name__ == "__main__":
    mcp.run()
```

### Step 6: Packaging for Distribution

Create `pyproject.toml` for installable package:

```toml
[project]
name = "research-assistant-mcp"
version = "1.0.0"
description = "Production MCP server for research synthesis using Claude"
requires-python = ">=3.11"
dependencies = [
    "fastmcp>=0.1.0",
    "pydantic>=2.0",
    "anthropic>=0.25.0",
]

[project.entry-points."mcp"]
research-assistant = "research_assistant.server:mcp"

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

Install and test:

```bash
# Install locally
uv pip install -e .

# Test with MCP Inspector
mcp server run research_assistant.server
```

## Architectural Decision Framework: Evaluating Tradeoffs

As you implement, you'll face choices. Here's how to evaluate them:

### Decision: When to Use Sampling vs Deterministic Operations

**Sampling** (calls Claude):
- Use when: Need reasoning, synthesis, judgment
- Cost: Claude API tokens
- Speed: 1-10 seconds per call
- Examples: Summarization, analysis, synthesis

**Deterministic** (local logic):
- Use when: Simple retrieval, filtering, validation
- Cost: Zero API tokens
- Speed: Milliseconds
- Examples: List files, search text, validate input

**Decision framework**:
- If operation requires understanding context → Sampling
- If operation is mechanical (search, filter) → Deterministic
- If operation could go either way → Use Deterministic; add Sampling as enhancement

### Decision: File Access Security Model

**Root-based** (what we use):
- Pro: Simple, auditable, secure
- Con: Fixed directory structure
- Best for: Teams with consistent file organization

**Pattern-based** (more flexible):
- Pro: Users can define access patterns
- Con: More complex, easier to misconfigure
- Best for: Enterprise systems with policy requirements

### Decision: Progress Frequency

**Too frequent** (every operation): Overhead, client spam
**Too sparse** (only at end): Client thinks server is broken
**Optimal**: Every 10-20% progress or every 1-2 seconds

Implement as:

```python
import time

class ProgressTracker:
    def __init__(self, total: int, update_interval: float = 1.0):
        self.total = total
        self.completed = 0
        self.last_update = time.time()
        self.update_interval = update_interval

    async def update(self, context: Context, completed: int):
        """Update progress, but only if enough time has passed."""
        self.completed = completed
        now = time.time()

        if now - self.last_update >= self.update_interval:
            await context.report_progress(self.completed, self.total)
            self.last_update = now
```

## Integration With AI: Designing Together

Now that you understand the architecture, you'll work with AI to fill in the complete implementation. This is Layer 2 + Layer 4 combined: You design the spec (Layer 4), AI implements (Layer 2 collaboration).

## Try With AI

**Setup**: Use Claude Code with your project. You'll implement the complete Research Assistant server using the specification and architectural guidance above.

**Prompt 1: Complete Project Structure**

Ask Claude Code:

```
I'm building a Research Assistant MCP server. Here's my specification:
[paste the Specification section above]

Create the complete project structure with:
1. pyproject.toml with fastmcp and anthropic dependencies
2. src/research_assistant/__init__.py (empty)
3. src/research_assistant/server.py with:
   - RESEARCH_DIR constant set to ~/research
   - validate_path() function that prevents directory traversal
   - @mcp.on_startup handler that logs server starting
   - @mcp.on_shutdown handler that logs server stopping
   - list_research_documents tool (deterministic)
   - search_documents tool (deterministic)

For search_documents, support full-text search across all files matching a query string.
Return matches as {document, line_number, matched_text}.

Show the complete server.py with proper type hints and Field descriptions for all parameters.
```

**What you're learning**: How to structure a production MCP server with security boundaries and basic deterministic tools. Evaluate Claude's response:

- Does `validate_path()` prevent directory traversal (e.g., paths with `..`)?
- Are startup/shutdown handlers properly async?
- Do both tools use `context.info()` for logging?
- Does search_documents handle empty results gracefully?

**Prompt 2: Sampling + Progress**

Add to your server:

```
Now add two more tools to the server:

1. summarize_document:
   - Takes document_path and summary_length ("brief", "standard", "detailed")
   - Validates path using validate_path()
   - Reports progress 3 steps: load (1/3), prepare (2/3), Claude call (3/3)
   - Calls Claude via context.session.create_message() with the document content
   - Returns {document_path, summary, word_count, token_estimate}
   - If document not found, raise FileNotFoundError with helpful message

2. synthesize_research:
   - Takes document_paths (list), research_question (string), depth ("overview" or "detailed")
   - Validates all paths upfront; skip any that don't exist with warning logs
   - Loads each document content (first 5000 chars to keep prompts manageable)
   - Reports progress per document (completed / total)
   - Calls Claude once with all document contents to synthesize an answer
   - Implements retry logic: if Claude fails, retry up to 3 times with exponential backoff (2^attempt seconds)
   - If all retries fail, let the exception propagate with error logging
   - Returns {research_question, synthesis, sources_used, sources_skipped, depth}

For the Claude prompt, build a multi-document synthesis prompt that includes:
- The research_question at the top
- All document contents with clear headers (--- filename ---)
- Instructions for overview vs detailed depth

Show the complete updated server.py with both new tools.
```

**What you're learning**: Sampling patterns with progress, multi-step error recovery, and sampling orchestration. Evaluate:

- Does `context.report_progress()` use (completed, total) correctly?
- Does retry logic use exponential backoff (2^0, 2^1, 2^2)?
- Are Claude calls wrapped in try-except with appropriate error handling?
- Does sampling happen through `context.session.create_message()` (not a direct API call)?

**Prompt 3: Roots Configuration and Graceful Degradation**

Now add error handling and roots:

```
Improve the server with production-grade error handling:

1. Add explicit @mcp.tool() error handling:
   - Wrap all tool implementations in try-except
   - Catch FileNotFoundError separately (file not found → error message)
   - Catch Exception broadly (any unexpected error → log and re-raise)
   - All errors should call context.error() before raising

2. Explain the roots configuration:
   - Our server restricts file access to ~/research
   - What type of roots would MCP use for this? (MCP Roots allow servers to declare directory boundaries)
   - How does validate_path() enforce roots without explicit MCP roots declaration?
   - In a real deployment with roots, how would this security work differently?

3. For synthesize_research specifically, implement graceful degradation:
   - If one document can't be read, log warning and skip it (don't fail entire operation)
   - Proceed with synthesis using documents that succeeded
   - Return sources_skipped count to show what was skipped
   - Only fail entire operation if ALL documents are invalid/missing

Show the updated server.py with error handling and explain the roots strategy in a code comment.
```

**What you're learning**: Error recovery strategies, graceful degradation, and how roots work in practice. Evaluate:

- Are all exceptions caught and logged before propagating?
- Does graceful degradation skip failed documents in synthesis without stopping?
- Is the roots validation approach secure and auditable?
- Does the server log enough information to debug production issues?

**Prompt 4: Packaging for Production**

Complete the packaging:

```
Final step: Make this server installable and testable.

1. Create pyproject.toml with:
   - Project metadata (name: research-assistant-mcp, version: 1.0.0)
   - Dependencies: fastmcp, pydantic, anthropic
   - Entry point for MCP: research-assistant = research_assistant.server:mcp

2. Add to server.py:
   - if __name__ == "__main__": mcp.run() (for direct execution)
   - Add docstrings to all functions explaining their role

3. Create a test outline (don't implement, just structure):
   ```python
   # tests/test_server.py
   import pytest
   from research_assistant.server import validate_path, list_research_documents

   def test_validate_path_accepts_valid():
       # Should accept ~/research/documents.txt
       pass

   def test_validate_path_rejects_traversal():
       # Should reject ~/research/../etc/passwd (directory traversal)
       pass

   def test_list_research_documents_empty_dir():
       # Should return empty list if no documents
       pass
   ```

4. Create README.md explaining:
   - What the server does
   - How to install: pip install -e .
   - How to run: mcp server run research_assistant.server
   - Example tools and what they do

Show the pyproject.toml, updated server.py with packaging-ready code, and README.md.
```

**What you're learning**: How to package a server for production deployment as a Digital FTE component. Evaluate:

- Does pyproject.toml have correct dependencies?
- Does entry point match the module structure?
- Is the README clear enough for someone else to install and use?
- Are docstrings sufficient for maintenance?

By completing this capstone, you've experienced specification-first thinking, feature orchestration, architectural decision-making, and production-readiness practices that define Digital FTE development. The spec-driven approach separates production systems from prototypes. Every decision is documented. Every feature serves a purpose. Every error is handled. This is how you build components customers will pay to use.

---

✅ **Capstone: Production MCP Server completes Chapter 38.** You've mastered the advanced features that make MCP servers production-ready. Test thoroughly with Claude Desktop. Process real documents. Hit edge cases. Verify error handling. Then package and deploy with confidence.
