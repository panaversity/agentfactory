# Feature Specification: RAG Agent Backend

**Feature Branch**: `005-rag-agent`
**Created**: 2025-11-29
**Status**: Draft
**Input**: User description: "RAG backend with document ingestion pipeline and semantic search API for RoboLearn platform"

## Success Evals (Defined First)

These measurable outcomes MUST be achieved for feature success:

| Eval ID | Outcome | Measurement Method |
|---------|---------|-------------------|
| **Eval-001** | Students find semantically relevant content via search | Automated test suite: top-5 results contain expected content for 20 test queries |
| **Eval-002** | Hardware tier filtering ensures personalization | 100% accuracy: no Tier 3+ content shown to Tier 1 students (verified via automated tests) |
| **Eval-003** | Incremental ingestion only processes changed files | Ingestion logs show 0 unchanged files re-embedded on incremental runs |
| **Eval-004** | Authors see content updates reflected quickly | End-to-end latency from git push to searchable: under 2 minutes (webhook → index → verify) |
| **Eval-005** | Context expansion enables full lesson reading | GET /lesson/{id} returns 100% of chunks in correct order |

## Overview

Production-grade RAG (Retrieval-Augmented Generation) backend for the RoboLearn educational platform. The system ingests markdown documentation from the book repository, creates semantic embeddings, stores them in Qdrant vector database, and provides a comprehensive search API with hardware-tier-aware personalization and context expansion capabilities.

### Assumptions

- **Vector Database**: Qdrant Cloud (already provisioned)
- **Embedding Model**: OpenAI text-embedding-3-small (1536 dimensions)
- **Document Format**: Markdown files with YAML frontmatter (Docusaurus convention)
- **Directory Structure**: docs/module-N-name/chapter-N-name/NN-lesson.md
- **Multitenancy**: book_id field for tenant isolation (future multi-book support)
- **Authentication**: Admin API key for ingestion endpoints; search endpoints public
- **Deployment Target**: Google Cloud Run (serverless)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Searches for Content (Priority: P1)

A student using the RoboLearn chatbot asks a question about ROS 2 topics. The system retrieves semantically relevant content chunks filtered by the student's hardware tier (e.g., Tier 1 laptop-only students don't see Tier 4 physical robot content).

**Why this priority**: Core value proposition - personalized semantic search enables the AI chatbot to provide accurate, hardware-appropriate answers.

**Independent Test**: Can be tested by calling POST /search with a query and hardware_tier parameter, verifying results respect tier filter.

**Acceptance Scenarios**:

1. **Given** indexed content with various hardware tiers, **When** student searches "ROS 2 publisher" with hardware_tier=1, **Then** only Tier 1 content is returned
2. **Given** indexed content, **When** student searches "URDF joint limits", **Then** top 5 semantically relevant chunks are returned with score above 0.7
3. **Given** a search result, **When** user requests context expansion, **Then** surrounding chunks (prev/next) are retrievable

---

### User Story 2 - Content Ingestion on Git Push (Priority: P2)

When an author pushes updated lesson content to the GitHub repository, the system automatically detects changed files and re-indexes only the modified content (incremental update).

**Why this priority**: Enables content updates without manual re-indexing or full collection rebuild.

**Independent Test**: Can be tested by triggering POST /ingestion/trigger with mode=incremental and verifying only changed files are processed.

**Acceptance Scenarios**:

1. **Given** existing indexed content, **When** POST /ingestion/trigger with mode=incremental, **Then** system compares file hashes and processes only new/modified files
2. **Given** a deleted source file, **When** incremental ingestion runs, **Then** corresponding chunks are removed from the index
3. **Given** a modified file, **When** incremental ingestion runs, **Then** old chunks are deleted before new chunks are inserted (atomic update)

---

### User Story 3 - Full Lesson Retrieval (Priority: P2)

A student wants to read the complete lesson after finding a relevant search result. The system retrieves all chunks for that lesson in reading order.

**Why this priority**: Context expansion is critical for educational content where reading the full lesson provides better understanding.

**Independent Test**: Can be tested by calling GET /lesson/{parent_doc_id} and verifying all chunks returned in chunk_index order.

**Acceptance Scenarios**:

1. **Given** a search result with parent_doc_id, **When** GET /lesson/{parent_doc_id}, **Then** all chunks for that lesson are returned ordered by chunk_index
2. **Given** a lesson with 10 chunks, **When** retrieving lesson, **Then** response includes total_chunks=10 and complete chunk list

---

### User Story 4 - GitHub Webhook Auto-Reindex (Priority: P3)

The system exposes a webhook endpoint that GitHub can call on push events, automatically triggering incremental ingestion.

**Why this priority**: Automation reduces manual effort for content authors.

**Independent Test**: Can be tested by sending POST /ingestion/webhook/github with appropriate headers and verifying job is queued.

**Acceptance Scenarios**:

1. **Given** valid GitHub webhook signature, **When** push event received, **Then** incremental ingestion job is queued
2. **Given** invalid signature, **When** webhook called, **Then** 401 Unauthorized returned

---

### User Story 5 - Admin Monitoring (Priority: P3)

An administrator can check system health, collection statistics, and ingestion job status.

**Why this priority**: Operational visibility for debugging and monitoring.

**Independent Test**: Can be tested by calling GET /health and GET /info endpoints.

**Acceptance Scenarios**:

1. **Given** running system, **When** GET /health, **Then** returns Qdrant connection status and collection status
2. **Given** completed ingestion jobs, **When** GET /ingestion/status, **Then** returns list of recent jobs with statistics

---

### Edge Cases

- What happens when embedding API rate limit is reached? System implements exponential backoff and reports partial progress.
- How does system handle malformed frontmatter? Parser uses sensible defaults (proficiency_level=A2, hardware_tier=1) and logs warning.
- What happens when Qdrant is temporarily unavailable? Health endpoint reports degraded status; search returns 503.
- What if a chunk is too small (under 100 tokens)? Chunker merges with adjacent content or excludes if below threshold.
- What happens when document structure changes (headers added/removed)? Chunk boundaries shift. System detects prev/next ID integrity violations (references to deleted chunks) and repairs context chain by removing invalid prev_chunk_id/next_chunk_id references.

## Requirements *(mandatory)*

### Functional Requirements

#### Ingestion Pipeline

- **FR-001**: System MUST crawl docs/ directory and discover all markdown files matching module-N/chapter-N/NN-lesson.md pattern
- **FR-002**: System MUST extract frontmatter metadata (proficiency_level, hardware_tier, layer) from each file
- **FR-003**: System MUST compute SHA-256 hash of each file for change detection
- **FR-004**: System MUST split content into semantic chunks using ## headers as boundaries
- **FR-005**: System MUST target 400-512 tokens per chunk with 15% overlap between consecutive chunks
- **FR-006**: System MUST track prev_chunk_id/next_chunk_id relationships for context expansion
- **FR-007**: System MUST generate deterministic UUIDs based on content hash (not position)
- **FR-008**: System MUST batch embed chunks using OpenAI API (batch size ~20)
- **FR-009**: System MUST upsert embedded chunks to Qdrant with full payload
- **FR-010**: System MUST support three ingestion modes: incremental, full, recreate

#### Search API

- **FR-011**: System MUST generate query embedding and perform semantic search against Qdrant
- **FR-012**: System MUST filter results by book_id (tenant isolation) for every query
- **FR-013**: System MUST filter by hardware_tier using Range(lte) - return tier X or lower
- **FR-014**: System MUST support optional filters: module, chapter range, lesson, proficiency_levels (MatchAny), layer
- **FR-015**: System MUST return chunk text, score, source metadata, and context expansion IDs
- **FR-016**: System MUST provide /context/{chunk_id} endpoint for walking prev/next chain
- **FR-016a**: Context expansion MUST return null for prev_chunk_id when chunk is first in lesson
- **FR-016b**: Context expansion MUST return null for next_chunk_id when chunk is last in lesson
- **FR-017**: System MUST provide /lesson/{parent_doc_id} endpoint for full lesson retrieval

#### Payload Indexing

- **FR-018**: System MUST create Qdrant payload indexes for: book_id, module, hardware_tier, chapter, lesson, proficiency_level, layer, parent_doc_id, content_hash, source_file_hash

#### API Endpoints

- **FR-019**: System MUST expose POST /search for semantic search
- **FR-020**: System MUST expose GET /health for health check
- **FR-021**: System MUST expose GET /info for collection statistics
- **FR-022**: System MUST expose POST /ingestion/trigger for manual ingestion (admin-protected)
- **FR-023**: System MUST expose GET /ingestion/status/{job_id} for job status
- **FR-024**: System MUST expose POST /ingestion/webhook/github for automated reindexing
- **FR-025**: System MUST expose GET /context/{chunk_id} for context expansion
- **FR-026**: System MUST expose GET /lesson/{parent_doc_id} for full lesson retrieval

### Key Entities

- **DocumentChunk**: A semantic unit of content with text, embedding, and metadata. Identified by content-hash-based UUID.
- **DocumentMetadata**: Hierarchical location (book_id, module, chapter, lesson), personalization filters (hardware_tier, proficiency_level, layer), context expansion IDs (parent_doc_id, prev_chunk_id, next_chunk_id), and change detection hashes.
- **IngestionJob**: Background job tracking ingestion progress (status, files_processed, chunks_created, duration).
- **IngestionState**: Tracks indexed files and their hashes for incremental updates.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Search queries return results in under 500ms for 95% of requests
- **SC-002**: Incremental ingestion processes only changed files (0 unchanged files re-embedded)
- **SC-003**: Full ingestion of 500+ chunks completes in under 5 minutes
- **SC-004**: Hardware tier filtering is 100% accurate (no Tier 3+ content returned for Tier 1 queries)
- **SC-005**: Context expansion successfully retrieves prev/next chunks 100% of the time when non-null prev/next IDs exist (handles first/last chunk gracefully)
- **SC-006**: Full lesson retrieval returns all chunks in correct order (by chunk_index)
- **SC-007**: System handles 100 concurrent search requests without degradation
- **SC-008**: Health endpoint accurately reports Qdrant connection status

## Non-Goals

- **NG-001**: This feature does NOT include the AI chat interface (separate feature)
- **NG-002**: This feature does NOT include user authentication beyond admin API key
- **NG-003**: This feature does NOT include real-time WebSocket streaming
- **NG-004**: This feature does NOT include PDF or HTML ingestion (markdown only)
- **NG-005**: This feature does NOT include query rewriting or HyDE (Hypothetical Document Embedding)

## Dependencies

- Qdrant Cloud instance (provisioned)
- OpenAI API key (for embeddings)
- Google Cloud Run (for deployment)
- GitHub repository with docs/ folder structure
