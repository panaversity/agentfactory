# Chapter 5: How It All Started: The Claude Code Phenomenon — Lesson Plan

**Chapter Type**: Hybrid (Narrative Foundation + Technical Tutorials)
**Chapter Location**: Part 2, Chapter 5 (AI Tool Landscape)
**Approved Specification**: `specs/005-chapter-5-spec/spec.md`
**Status**: Ready for Implementation
**Estimated Total Time**: 75-95 minutes (30-35 min reading + 45-60 min hands-on)

---

## Chapter Overview

This chapter serves as the primary tutorial introduction to Claude Code for beginners who have completed Part 1 (Chapters 1-4). It combines narrative storytelling about Claude Code's origins with comprehensive, hands-on tutorials for installation, authentication, subagents, Agent Skills, and MCP integration.

**Learning Trajectory**:
- Start with "why" (Claude Code's paradigm shift) → Understand the tool's capabilities
- Move to "how" (installation, authentication) → Get the tool working
- Progress to "what now" (subagents, skills, MCP) → Learn advanced features progressively
- End with "where next" (common workflows, reflection) → Prepare for deeper exploration

---

## 5 Lesson Structure

### Lesson 1: The Claude Code Origin Story and Paradigm Shift
**Duration**: 8-10 min | **Bloom's**: Understand | **Type**: Narrative

Learn why Claude Code is revolutionary, how it differs from chat-based AI tools, and why it matters for development.

**Content**: Origin story, agentic vs passive AI, paradigm shift, terminal integration, reflection prompts
**Assessment**: Reflection on paradigm shift understanding

### Lesson 2: Installing and Authenticating Claude Code
**Duration**: 27-35 min | **Bloom's**: Apply | **Type**: Technical

Get Claude Code working with step-by-step platform-specific instructions and troubleshooting.

**Content**: Platform selection, NPM + native installation, Claude.ai + Console authentication, first-run verification
**Assessment**: Hands-on installation, verification test
**Success Target**: 95% install successfully on first attempt (SC-002, SC-003)

### Lesson 3: Understanding and Using Subagents
**Duration**: 25-32 min | **Bloom's**: Apply | **Type**: Technical

Create your first subagent and learn when specialization provides value.

**Content**: Subagent architecture, context isolation, code-reviewer tutorial, use cases, management commands
**Assessment**: Create working subagent, explain difference from main conversation
**Success Target**: 80% create custom subagent and explain usage (SC-004)

### Lesson 4: Creating and Using Agent Skills
**Duration**: 32-39 min | **Bloom's**: Create | **Type**: Technical

Build a custom Agent Skill and understand competitive advantage of domain-specific skill libraries.

**Content**: Skill structure, SKILL.md format, model-invoked discovery, python-docstring-writer tutorial, naming best practices
**Assessment**: Create skill with autonomous discovery, clear naming/description
**Success Target**: 80% create custom skill with autonomous discovery (SC-005)

### Lesson 5: Connecting MCP Servers and Common Workflows
**Duration**: 32-39 min | **Bloom's**: Apply | **Type**: Technical/Practical

Integrate external services and demonstrate real-world Claude Code workflows.

**Content**: MCP architecture, GitHub MCP setup, end-to-end workflow examples, security guidance, 4 common workflows
**Assessment**: Connect MCP server, use in practical task, explain security considerations
**Success Target**: 75% successfully connect and use MCP in task (SC-006)

---

## Progressive Scaffolding Strategy

**Pyramid of Complexity** (Bloom's taxonomy aligned):
1. **Foundation (L1)**: Understand paradigm - Narrative, minimal cognitive load
2. **Practical Setup (L2)**: Install & authenticate - Step-by-step, success building
3. **First Advanced (L3)**: Subagents - Concrete examples, clear use cases
4. **Second Advanced (L4)**: Skills - Abstract concept, template-based
5. **Integration (L5)**: MCP - Highest complexity, real-world grounded

**Scaffolding Checkpoints**:
- After L2: Readers have working tool → confidence boost
- After L3: Readers can specialize Claude Code → empowerment
- After L4: Readers can build skill libraries → ownership
- After L5: Readers can integrate external tools → professionalism

---

## Integration with Book Structure

**Part 2 (Chapters 5-8): AI Tool Landscape**
- Ch 5 (this): Claude Code deep-dive (terminal-based)
- Ch 6: Gemini CLI (another terminal tool)
- Ch 7: GitHub Copilot (IDE-integrated)
- Ch 8: Choosing the right tool (comparison)

**Prior Chapters** (1-4): Established AI-first philosophy and AIDD framework
**Subsequent**: Operationalizes concepts in Python (Ch 10+), Spec-Kit (Ch 22+), Agents (Ch 25+)

---

## Functional Requirements Coverage

All 17 functional requirements explicitly addressed in lesson plan:

| FR # | Requirement | Lesson |
|------|-------------|--------|
| FR-001 | Origin story narrative | L1 |
| FR-002 | Claude Code differentiators | L1 |
| FR-003 | Platform-specific installation | L2 |
| FR-004 | Authentication both types | L2 |
| FR-005 | Subagents explanation | L3 |
| FR-006 | Subagent creation tutorial | L3 |
| FR-007 | Agent Skills explanation | L4 |
| FR-008 | Skill creation demonstration | L4 |
| FR-009 | MCP explanation | L5 |
| FR-010 | MCP server connection | L5 |
| FR-011 | Common workflows section | L5 |
| FR-012 | Troubleshooting guidance | L2, L5 |
| FR-013 | Competitive advantage | L4 |
| FR-014 | Code tested/verified | All |
| FR-015 | Security warnings | L5 |
| FR-016 | Reflection prompts | All |
| FR-017 | Learning WITH Claude Code | All |

---

## Success Criteria Coverage

All 10 success criteria addressed:

| SC | Target | Lesson(s) | Method |
|----|--------|-----------|--------|
| SC-001 | 90% explain differences | L1 | Reflection prompts |
| SC-002 | 95% install successfully | L2 | Hands-on verification |
| SC-003 | 90% authenticate in 10 min | L2 | Hands-on test |
| SC-004 | 80% create subagent | L3 | Hands-on + explanation |
| SC-005 | 80% create skill | L4 | Hands-on + verification |
| SC-006 | 75% connect/use MCP | L5 | Hands-on test |
| SC-007 | Troubleshoot 90% errors | L2, L5 | Comprehensive guide |
| SC-008 | 25-35 read + 45-60 hands-on | All | Total 75-95 min |
| SC-009 | Confidence for basic tasks | L1, L5 | Reflection + workflows |
| SC-010 | Technical accuracy | All | Pre-publication verification |

---

## Domain Skills Integration

All 8 mandatory domain skills applied throughout:

1. **learning-objectives**: Measurable Bloom's outcomes (Understand→Apply→Create)
2. **concept-scaffolding**: Progressive introduction, clear prerequisites
3. **code-example-generator**: Tested commands, expected output, complete examples
4. **exercise-designer**: Guided tutorials with increasing independence
5. **assessment-builder**: Checkpoints, self-assessment, hands-on verification
6. **technical-clarity**: Grade 7 reading level, jargon defined, no gatekeeping
7. **book-scaffolding**: Clear positioning, cognitive load management
8. **ai-augmented-teaching**: Learning WITH Claude Code emphasized

---

## Risk Mitigation Summary

**Risk 1: Documentation Volatility** → Version dates, official links, troubleshooting section
**Risk 2: Authentication Complexity** → Decision tree, visual separation, FAQ section
**Risk 3: Platform Failures** → NPM + native methods, fallback steps, GitHub issue links
**Risk 4: MCP Security** → Security section, trusted servers only, evaluation guidance
**Risk 5: Cognitive Overload** → Progressive scaffolding, optional lessons, clear motivation

---

## Engagement Elements

- **Opening Hooks**: Origin story, quick wins, specialization, advantage, integration
- **Real-World Context**: Professional use cases, common tasks, authentic workflows
- **Success Checkpoints**: Verification tests, reflection prompts
- **Visual Breaks**: Terminal output, comparison tables, decision trees
- **Pacing**: 8-39 min per lesson, clear time estimates
- **Accessibility**: Clear language, jargon defined, no gatekeeping

---

## Constitutional Alignment

- ✅ Principle 1: AI-First Teaching (emphasizes learning WITH Claude Code)
- ✅ Principle 5: Progressive Complexity (basics → advanced with scaffolding)
- ✅ Principle 6: Consistent Structure (follows output styles, shared infrastructure)
- ✅ Principle 7: Technical Accuracy (verified against current docs)
- ✅ Principle 8: Accessibility (Grade 7 reading, no gatekeeping, platform-inclusive)

---

## Implementation Status

**This lesson plan is complete and ready for the content-implementer subagent to create detailed content for all 5 lessons.**

All specifications, requirements, and success criteria from the approved spec are addressed and mapped to specific lessons.

**Next Step**: Run `/sp.implement` to assign lesson creation tasks and generate complete lesson content.
