---
id: 0001
title: chapter-38-mcp-servers-spec
stage: spec
date: 2025-12-26
surface: claude-code
model: claude-opus-4-5
feature: 047-chapter-38-mcp-servers
branch: 047-chapter-38-mcp-servers
user: mjs
command: /sp.specify
labels: [chapter, mcp, educational-content]
links:
  spec: specs/047-chapter-38-mcp-servers/spec.md
  ticket:
  adr:
  pr:
files:
  - specs/047-chapter-38-mcp-servers/spec.md
  - specs/047-chapter-38-mcp-servers/checklists/requirements.md
tests:
  - N/A (specification phase)
---

## Prompt

/sp.specify Chapter 38: Building Custom MCP Servers - A comprehensive chapter teaching students to build their own MCP (Model Context Protocol) servers using Python's FastMCP SDK and TypeScript's official SDK. This chapter follows Chapter 37 (MCP Fundamentals) and builds on Python skills from Part 5.

Key content areas:
1. MCP Server Architecture - SDK structure, lifecycle, JSON-RPC communication, transport options
2. Building Your First Server - Project setup with uv, FastMCP initialization, basic tools
3. Advanced Tool Patterns - Complex parameters with Pydantic, streaming results, error handling
4. Resource Implementation - URI schemes, MIME types, direct vs templated resources
5. Prompt Templates - Pre-built prompts, parameterized prompts
6. Authentication & Security - API key handling, environment variables, OAuth patterns
7. Testing & Deployment - MCP Inspector, Postman testing, Claude Desktop configuration

Target: 8 lessons (7 content + 1 quiz) at B1-B2 proficiency level

## Response snapshot

Created comprehensive specification for Chapter 38 with:
- 8 user stories covering all MCP server development scenarios (P1-P3)
- 16 functional requirements covering tools, resources, prompts, transport, security
- Lesson structure: L1 architecture + 6 L2 collaboration lessons + quiz
- Clear assumed knowledge (Part 5 Python, Chapter 37 MCP fundamentals)
- Success criteria for educational outcomes
- Edge cases for common failure scenarios

## Outcome

- Impact: Comprehensive spec ready for planning phase
- Tests: N/A (specification phase)
- Files: spec.md, checklists/requirements.md
- Next prompts: /sp.plan or /sp.clarify if questions arise
- Reflection: User input was comprehensive; no clarification markers needed

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Checklist validation PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to planning phase
