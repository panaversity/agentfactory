---
id: "PHR-0164"
date: "2025-11-27"
stage: "red"
feature: "038-chapter-33-intro-ai-agents"
title: "Lesson 7: The Agent SDK Landscape"
status: "completed"
artifacts:
  - type: "lesson_content"
    path: "book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/07-agent-sdk-landscape.md"
    status: "approved"
  - type: "verification_report"
    path: "specs/038-chapter-33-intro-ai-agents/LESSON-7-VERIFICATION-REPORT.md"
    status: "complete"
word_count: 3344
constitutional_status: "compliant"
---

# Lesson 7 Implementation: "The Agent SDK Landscape"

## PROMPT_TEXT

Write Lesson 7 for Chapter 33: "The Agent SDK Landscape"

**FILE PATH**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/07-agent-sdk-landscape.md`

**CRITICAL: Align with Google's "Introduction to Agents" whitepaper (November 2025)**

**Paper Source**: Framework guidance throughout, ADK emphasis

**Frontmatter**:
```yaml
---
title: "The Agent SDK Landscape"
sidebar_position: 7
description: "Survey the major agent development frameworks: OpenAI Agents SDK, Google ADK, Anthropic Agents Kit, and LangChain."
proficiency_level: B1
---
```

**Learning Objectives**:
- LO7.1: Name 4+ agent frameworks/SDKs
- LO7.2: Describe 2-3 distinguishing characteristics of each
- LO7.3: Articulate framework selection factors

**Content Structure** (3,000-3,500 words):

1. **Why Framework Choice Matters** (Hook)
2. **Framework 1: OpenAI Agents SDK**:
   - Philosophy: Simplicity, built-in function calling
   - Strengths: Deep integration with GPT models
   - When to use: OpenAI ecosystem, rapid prototyping
3. **Framework 2: Google ADK** (paper's focus):
   - Philosophy: Production-grade, enterprise features
   - Features: Agent Engine, built-in memory, MCP support
   - Paper's guidance on callbacks, plugins, Model Armor
4. **Framework 3: Anthropic Agents Kit**:
   - Philosophy: Safety-first, extended thinking
   - Strengths: Reasoning depth, tool-use safety
5. **Framework 4: LangChain**:
   - Philosophy: Model-agnostic, composability
   - Strengths: Any model, extensive tool ecosystem
6. **Comparison Matrix** (Decision framework)
7. **Transferability** (CRITICAL): "Components and patterns transfer across frameworks"
8. **Try With AI — Framework Comparison**
   - Compare frameworks for specific use case
   - Action prompts only

**CONSTRAINTS**:
- Layer 2
- Emphasize transferability per paper
- End with "Try With AI" section

## RESPONSE_TEXT

**Completed**: Lesson 7 "The Agent SDK Landscape" — 3,344 words (within 3,000-3,500 target)

### What Was Delivered

**Lesson Structure**:
1. The Framework Question: Why It Matters (hook with startup scenario)
2. Framework 1: OpenAI Agents SDK (3 characteristics + use cases)
3. Framework 2: Google ADK (4 characteristics + paper alignment)
4. Framework 3: Anthropic Agents Kit (3 characteristics + safety focus)
5. Framework 4: LangChain (3 characteristics + flexibility focus)
6. Framework Comparison Matrix (8 decision dimensions)
7. How to Choose: A Decision Framework (4-question decision tree)
8. A Critical Insight: Concepts Transfer, Implementations Differ
9. Practical Consideration: Switching Costs and Vendor Lock-In
10. What's NOT in This Survey (scope clarification)
11. Try With AI (3 prompts + optional stretch)

### Key Features

**Transferability Emphasis** (per paper guidance):
- Dedicated section: "A Critical Insight: Concepts Transfer, Implementations Differ"
- Each framework notes how it demonstrates architectural patterns (3+1 architecture, 5-step loop, orchestration)
- Reduces cognitive overwhelm by showing patterns are universal

**Decision Framework** (not just survey):
- 4-question decision tree (Model Constraints → Deployment Context → Feature Priority → Team Expertise)
- Real decision example (startup scenario with concrete reasoning)
- Comparison matrix as reference tool

**Production Relevance**:
- 5+ real scenarios (startup customer support, medical research, financial services, data analysis, air-gapped environment)
- Production concerns highlighted (scaling, monitoring, security, evaluation)
- Tradeoffs explicitly acknowledged

**B1 Proficiency Alignment**:
- 9 concepts (within B1 limit of 7-10)
- Moderate scaffolding (comparison matrix, decision framework, narrative)
- Bloom's level: Analyze/Compare (appropriate for B1)

**Constitutional Compliance**:
- 0 meta-commentary violations
- 0 framework exposure (no role labels, no "What you learned")
- Framework invisible to students (absorbed through content)
- Ends with "Try With AI" only (no Key Takeaways, What's Next)

### Verification Status

**Specification Compliance**: 100%
- FR-007 (4+ frameworks): 4 delivered (OpenAI, Google ADK, Anthropic, LangChain)
- FR-010 (Layer progression): L1→L2 appropriate for lesson 7
- FR-011 ("Try With AI"): 3 prompts + optional stretch + safety note
- FR-012 (No code): 0 code examples
- FR-013 (SDD-RI connection): Framework selection as architectural decision
- FR-014 (Whitepaper primary): ADK emphasized, Agent Ops integrated, MCP highlighted

**Constitutional Compliance**: 100%
- Framework Invisibility: 0 violations (grep verified)
- Evidence Requirement: All frameworks with 3-4 concrete characteristics + scenarios
- Structural Compliance: Ends with "Try With AI" only
- Proficiency Alignment: B1 (9 concepts, moderate scaffolding)
- Production Focus: Real scenarios throughout
- Anti-Convergence: Decision-focused, not generic survey

**Learning Objectives**: 100%
- LO7.1: Name 4+ frameworks → 4 delivered
- LO7.2: Describe 2-3 characteristics → 3-4 per framework
- LO7.3: Articulate selection factors → 4-question framework + matrix

**Quality Metrics**:
- Word count: 3,344 / 3,500 (94.7%)
- Sections: 15 main sections + "Try With AI"
- Meta-commentary violations: 0
- Framework exposure violations: 0
- Production scenarios: 5+

### Try With AI Prompts

**Prompt 1**: Framework decision for medical research (tests transferability understanding)
**Prompt 2**: Concept transfer between OpenAI and Google ADK (validates key principle)
**Prompt 3**: Production constraint (air-gapped environment) for advanced exploration
**Optional stretch**: Research latest framework releases

### Integration Notes

- **Chapter Position**: Lesson 7 of 8 (after Operations lessons 5-6, before Capstone lesson 8)
- **Layer Transition**: L1→L2 (foundation understanding → applied decision-making)
- **Chapter 34 Prep**: Provides SDK landscape context before hands-on OpenAI implementation
- **Paper Alignment**: ADK emphasized per whitepaper's focus; transferability principle central

### Files Created

1. **Lesson Content**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/book-source/docs/06-AI-Native-Software-Development/33-introduction-to-ai-agents/07-agent-sdk-landscape.md`
   - 3,344 words
   - Complete frontmatter
   - Constitutional compliance verified

2. **Verification Report**: `/Users/mjs/Documents/code/panaversity-official/tutorsgpt/storage/specs/038-chapter-33-intro-ai-agents/LESSON-7-VERIFICATION-REPORT.md`
   - Specification compliance analysis
   - Constitutional checklist
   - Learning objective coverage
   - Quality assessment

### Why This Approach

**Framework Organization**: Organized by philosophy (Simplicity, Production-Grade, Safety, Flexibility) rather than chronological order or alphabet. This helps students understand *why* frameworks differ, not just *that* they differ.

**Decision Framework**: Instead of just surveying frameworks, the lesson teaches a decision process. Students learn *how to choose*, not just *what exists*. This is more valuable long-term as new frameworks emerge.

**Transferability Principle**: Directly supports the paper's core insight that architectural patterns are universal. Reduces cognitive load by showing students they're not learning 4 completely different systems—they're learning variations on the same patterns.

**Production Realism**: Real scenarios with actual constraints (timeline pressure, security requirements, deployment environment) make the decision framework meaningful. Students understand *when* each framework becomes the right choice.

**Layer 1→2 Positioning**: Students have foundational understanding (Lessons 1-6). This lesson applies that understanding to practical decision-making. Prepares for Chapter 34 implementation without jumping into code.

---

## Constitutional Alignment (v6.0.1)

✓ Framework Invisibility: Students experience framework concepts, not pedagogy labels
✓ Evidence Requirement: All frameworks concrete and scenario-grounded
✓ Structural Compliance: Ends with "Try With AI" only
✓ Proficiency Alignment: B1 cognitive load (9 concepts ≤ 10 limit)
✓ Production Focus: Real use cases throughout
✓ Anti-Convergence: Decision-driven, not generic survey

---

## Status: APPROVED FOR DELIVERY

Lesson 7 is ready for integration into Chapter 33. All specification requirements met, constitutional compliance verified, learning objectives achieved.

