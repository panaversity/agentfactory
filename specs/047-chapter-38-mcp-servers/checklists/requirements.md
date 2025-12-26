# Specification Quality Checklist: Chapter 38 - Building Custom MCP Servers

**Purpose**: Validate specification completeness and quality before proceeding to planning
**Created**: 2025-12-26 (Revised)
**Feature**: [spec.md](../spec.md)

## Content Quality

- [x] No implementation details (languages, frameworks, APIs)
- [x] Focused on user value and business needs
- [x] Written for non-technical stakeholders
- [x] All mandatory sections completed

## Requirement Completeness

- [x] No [NEEDS CLARIFICATION] markers remain
- [x] Requirements are testable and unambiguous
- [x] Success criteria are measurable
- [x] Success criteria are technology-agnostic (no implementation details)
- [x] All acceptance scenarios are defined
- [x] Edge cases are identified
- [x] Scope is clearly bounded
- [x] Dependencies and assumptions identified

## Scope Separation (CRITICAL)

- [x] Scope Clarification table clearly distinguishes Chapter 37 vs 38
- [x] FR-015: MUST NOT re-explain what primitives ARE
- [x] FR-016: MUST NOT re-explain transport theory
- [x] FR-017: MUST NOT re-explain client configuration
- [x] All user stories focus on IMPLEMENTATION, not concepts
- [x] No overlap with Chapter 37 lessons (Architecture, Transport, Primitives conceptual)

## Educational Content Quality

- [x] Assumed Knowledge section references Chapter 37 prerequisite
- [x] Proficiency level specified (B1-B2)
- [x] Layer progression (all L2 except L4 Capstone)
- [x] Chapter focuses on CODE PATTERNS, not conceptual explanations
- [x] Capstone project integrates all skills

## Feature Readiness

- [x] All functional requirements have clear acceptance criteria
- [x] User scenarios cover primary flows (project setup → tools → resources → prompts → auth → testing → packaging → capstone)
- [x] Feature meets measurable outcomes defined in Success Criteria
- [x] No implementation details leak into specification
- [x] Lesson structure is 9 lessons (vs. original 8) with proper capstone

## Comparison with Chapter 37 Structure

**Chapter 37 Lessons (from user context)**:
1. MCP Architecture Overview
2. Transport Layers (stdio vs HTTP/SSE)
3. Tools - The Model-Controlled Primitive
4. Resources - The App-Controlled Primitive
5. Prompts - The User-Controlled Primitive
6. Configuring MCP Clients (Claude Code, Cursor, etc.)
7. Using Community MCP Servers
8. Debugging & Troubleshooting MCP
9. Chapter Quiz

**Chapter 38 Lessons (revised)**:
1. Project Setup with FastMCP (NOT in Ch 37)
2. Implementing Tools (@mcp.tool) (CODE, not concept)
3. Implementing Resources (@mcp.resource) (CODE, not concept)
4. Implementing Prompts (@mcp.prompt) (CODE, not concept)
5. Server Authentication & Security (NOT in Ch 37)
6. Testing & Debugging Your Server (server-author perspective)
7. Packaging & Publishing (NOT in Ch 37)
8. Capstone - Domain-Specific Server (NOT in Ch 37)
9. Chapter Quiz

**Overlap Check**: ✅ NO OVERLAP - Chapter 38 focuses entirely on server implementation code patterns while Chapter 37 covers concepts and client usage.

## Notes

- Spec revised to eliminate overlap with Chapter 37
- Clear scope table added at top of spec
- Explicit FR-015, FR-016, FR-017 prevent re-explaining Chapter 37 content
- Ready for `/sp.plan`
