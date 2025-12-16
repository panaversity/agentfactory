# PanaversityFS Developer Guide

> Complete guide for understanding, using, and maintaining PanaversityFS

## What is PanaversityFS?

PanaversityFS is an **MCP (Model Context Protocol) server** that provides unified storage operations for educational book content. It enables AI agents to read, write, search, and manage lessons, summaries, and binary assets across multiple books.

**Authoritative Specification**: [`specs/039-panaversity-fs-hardening/spec.md`](../../../../specs/039-panaversity-fs-hardening/spec.md)

**Key Stats** (as of 2025-12-04):
- **12 MCP tools** for content management
- **301 tests** (unit, integration, property, performance, e2e)
- **3 storage backends** (filesystem, Cloudflare R2, Supabase)
- **7 Alloy invariants** (R1-R7) verified by property tests
- **31 functional requirements** (FR-001 to FR-031)
- **10 success criteria** (SC-001 to SC-010)

## Guide Structure

| Guide | Purpose | Read When... |
|-------|---------|--------------|
| [01-quickstart.md](./01-quickstart.md) | Get running in 5 minutes | You're new to the project |
| [02-architecture.md](./02-architecture.md) | System design & components | You need to understand how it works |
| [03-tools-reference.md](./03-tools-reference.md) | All 12 MCP tools | You're integrating with the API |
| [04-codebase-map.md](./04-codebase-map.md) | Source code organization | You're making changes |
| [05-testing.md](./05-testing.md) | Test suites & how to run them | You're adding features |
| [06-extending.md](./06-extending.md) | Adding new tools/features | You're extending the system |
| [07-operations.md](./07-operations.md) | Deployment & monitoring | You're running in production |

## Quick Links

### For Users (AI Agents/Integrators)
- [MCP Tools Reference](./03-tools-reference.md) - Complete API documentation
- [Quickstart](./01-quickstart.md) - Get connected quickly

### For Developers
- [Codebase Map](./04-codebase-map.md) - Navigate the source code
- [Testing Guide](./05-testing.md) - Run and write tests
- [Extending Guide](./06-extending.md) - Add new functionality

### For Operators
- [Operations Guide](./07-operations.md) - Deploy and monitor

## Technology Stack

| Layer | Technology | Purpose |
|-------|------------|---------|
| Protocol | MCP (FastMCP) | AI agent communication |
| Storage | OpenDAL | Unified storage abstraction |
| Database | SQLAlchemy + Alembic | Audit trail & journal |
| Validation | Pydantic v2 | Input/output models |
| Metrics | prometheus-client | Observability |
| Testing | pytest + hypothesis | Comprehensive test coverage |

## Key Concepts (from Spec)

| Concept | Spec Reference | Description |
|---------|----------------|-------------|
| **Conflict Detection** | FR-002, FR-003, FR-004 | SHA256 hash-based optimistic concurrency |
| **User Overlays** | FR-015 to FR-019 | Per-user content customization without modifying base |
| **Audit Trail** | FR-020 to FR-024 | Append-only hash chain with agent provenance |
| **Schema Enforcement** | FR-007 to FR-010, R1 | Docusaurus-aligned path validation |
| **Archive Streaming** | FR-011 to FR-014, R4 | <60s, <64MB for 500-file books |
| **Delta Builds** | FR-025 to FR-027 | Manifest hash for incremental Docusaurus builds |

## Alloy Invariants

These invariants are verified by property tests in `tests/property/`:

| ID | Invariant | Test File |
|----|-----------|-----------|
| R1 | All paths conform to Docusaurus schema | `test_invariant_r1_schema.py` |
| R2 | Journal hash matches storage content | `test_invariant_r2_journal.py` |
| R3 | Delete operations are idempotent | (unit tests) |
| R4 | Archives complete in <60s with <64MB | `test_archive_throughput.py` |
| R5 | User overlays are isolated | `test_invariant_r5_overlay.py` |
| R6 | Audit log maintains hash chain | `test_invariant_r6_audit.py` |
| R7 | All entries have valid agent_id | `test_invariant_r7_agent.py` |

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.3.0 | 2025-12-04 | Production hardening (Feature 039): 12 tools, 301 tests, overlay personalization, Alloy invariants |
| 0.2.0 | 2025-11-24 | ADR-0018: Docusaurus-aligned structure |
| 0.1.0 | 2025-11-01 | Initial MCP server with 9 tools |
