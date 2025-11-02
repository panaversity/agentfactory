# Feature Specification: Part-3 Goals - Working with AI Coding Agents

**Feature Branch**: `009-part-3-goals`
**Created**: 2025-01-15
**Status**: Draft
**Input**: User description: "Write goals for Part-3 which would consist of 2 chapters. Here is the context to the both chapters. @context\10_chap9_specs\ and @context\11_chap10_specs\ . Follow the links if present. The purpose of this book is to reimagine CS education so it should be - Beginner-friendly (no jargon without context) - Business-focused (solving problems, not just writing code) - Agent-native mindset (AI as collaborator for thinking and building) - Accessible to non-developers who want to use AI to create solutions. Here is the preface of the book @context\01_preface\readme.md"

## User Scenarios & Testing *(mandatory)*

<!--
  IMPORTANT: User stories should be PRIORITIZED as user journeys ordered by importance.
  Each user story/journey must be INDEPENDENTLY TESTABLE - meaning if you implement just ONE of them,
  you should still have a viable MVP (Minimum Viable Product) that delivers value.
  
  Assign priorities (P1, P2, P3, etc.) to each story, where P1 is the most critical.
  Think of each story as a standalone slice of functionality that can be:
  - Developed independently
  - Tested independently
  - Deployed independently
  - Demonstrated to users independently
-->

### User Story 1 - Beginner Learns to Direct AI Agents (Priority: P1)

An aspiring developer with no coding background wants to understand how to effectively communicate with AI coding agents (Claude Code and Gemini CLI) to build software. They need to learn the fundamental skill of "prompt engineering" - how to ask AI agents to help them write code.

**Why this priority**: This is the foundational skill for AI-native development. Without effective prompting, learners cannot leverage AI agents to build software. This is P1 because it's the entry point for all subsequent learning in the book.

**Independent Test**: Can be fully tested by having a learner complete a simple task like "Ask Claude Code to create a basic Python function" and evaluating whether their prompt includes clear command, context, and desired outcome. Delivers immediate value by enabling learner to get useful code from AI agents.

**Acceptance Scenarios**:

1. **Given** a learner who has never used Claude Code, **When** they read Chapter 9 (Prompt Engineering), **Then** they can construct a basic prompt with command, context, and expected output to generate a working Python function
2. **Given** a learner with a vague idea ("I want user authentication"), **When** they apply the 8-element AIDD prompting framework, **Then** they produce a structured prompt that results in production-quality code from Claude Code
3. **Given** a learner receiving generic/wrong code from an AI agent, **When** they apply technical constraints and examples to their prompt, **Then** the AI generates code that matches their project's existing patterns and tech stack

---

### User Story 2 - Beginner Manages AI Agent Context (Priority: P2)

A learner building their first multi-file project needs to understand how to manage what the AI agent "knows" - ensuring it has the right files, patterns, and project information to generate contextually appropriate code rather than generic solutions.

**Why this priority**: Context engineering is the second critical skill after prompting. While learners can get started with just prompting (P1), they'll quickly encounter issues where AI generates code that doesn't fit their project. This is P2 because it enables moving from toy examples to real projects.

**Independent Test**: Can be tested by having a learner work on a project across multiple sessions. Success is measured by whether code generated in Session 2 follows patterns established in Session 1 (via context management techniques like memory files and progressive loading).

**Acceptance Scenarios**:

1. **Given** a learner working on a FastAPI project with established patterns, **When** they apply progressive context loading (load only relevant files), **Then** Claude Code generates new code that matches their existing service layer pattern without being told explicitly
2. **Given** a learner returning to a project after 2 days, **When** they use checkpoint files and memory documents, **Then** the AI agent maintains consistency with previous architectural decisions
3. **Given** a learner experiencing "context rot" (AI forgetting earlier context), **When** they apply context compression techniques, **Then** the AI maintains coherent understanding across a long development session

---

### User Story 3 - Learner Understands Agent-Native Mindset (Priority: P3)

A learner needs to shift their mental model from "I write code" to "I design systems and direct AI agents to implement them." This involves understanding when to prompt, when to provide context, and when to iterate with the AI.

**Why this priority**: This is a mindset shift rather than a specific skill. While important for long-term success, learners can be productive with P1 and P2 skills before fully embracing this mindset. It's P3 because it's more aspirational and develops over time with practice.

**Independent Test**: Can be assessed through reflection exercises where learners describe their development process. Success is when they describe themselves as "AI orchestrators" rather than "coders" and can articulate when to use AI vs when to design manually.

**Acceptance Scenarios**:

1. **Given** a complex feature requirement, **When** a learner plans the implementation, **Then** they break it into: (a) architecture/design they'll do, (b) implementation AI will generate, (c) validation they'll perform
2. **Given** AI-generated code with a bug, **When** the learner debugs it, **Then** they don't abandon AI but instead refine their prompt with better constraints/examples
3. **Given** a choice between learning syntax details or learning better prompting, **When** the learner allocates study time, **Then** they prioritize understanding concepts over memorizing syntax

---

### Edge Cases

- What happens when a learner tries to use advanced prompting techniques without understanding basic concepts (e.g., using complex technical constraints before understanding command and context)?
- How does the book handle learners who want to "skip" prompting/context skills and just get code from AI?
- What if a learner becomes over-reliant on AI and doesn't develop understanding of what the code does?
- How do we address situations where AI agents give incorrect or insecure code?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Part-3 introduction MUST clearly articulate learning goals: mastering prompt engineering and context engineering for AI-Driven Development
- **FR-002**: Part-3 goals MUST connect to book philosophy: AI as co-learning partner, not replacement for understanding
- **FR-003**: Part-3 goals MUST emphasize practical outcomes: learners will be able to build real software by directing AI agents
- **FR-004**: Part-3 introduction MUST explain the relationship between Chapter 9 (Prompt Engineering) and Chapter 10 (Context Engineering)
- **FR-005**: Part-3 goals MUST establish the complexity tier: beginner-friendly with no jargon without explanation
- **FR-006**: Part-3 introduction MUST frame AI agents as "collaborators for thinking and building," not just code generators
- **FR-007**: Part-3 goals MUST include success criteria: what learners will be able to do after completing both chapters
- **FR-008**: Part-3 introduction MUST address common misconceptions: "AI will write all the code for me" vs "I'll learn to direct AI intelligently"

### Key Entities *(include if feature involves data)*

- **Learning Objectives**: Concrete skills learners will acquire (prompt engineering techniques, context management strategies)
- **Chapter Progression**: How Chapter 9 and Chapter 10 build on each other sequentially
- **Success Indicators**: Measurable outcomes that indicate learner readiness to proceed to Part-4

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Learners can construct effective prompts for Claude Code/Gemini CLI that produce working code on first attempt 70% of the time (for routine tasks)
- **SC-002**: Learners can manage AI agent context across multi-session projects, maintaining pattern consistency 90%+ of the time
- **SC-003**: Learners demonstrate understanding of agent-native mindset by describing development process in terms of "design → AI implementation → validation" rather than "write code myself"
- **SC-004**: 80% of learners report feeling confident they can use AI agents to build real projects after completing Part-3
- **SC-005**: Learners can identify when code is AI-generated incorrectly and refine prompts/context to fix it (validation-first mindset)

## Assumptions

- Learners have completed Part-1 (Foundations) and Part-2 (Understanding the Landscape) before starting Part-3
- Learners have access to Claude Code or Gemini CLI (or similar AI coding agents)
- Learners have basic computer literacy (can navigate file systems, run commands in terminal)
- The book's philosophy (AI as co-learning partner, specification-driven, validation-first) has been established in earlier parts

## Dependencies

- **Part-1**: Learners understand the AI-native software development vision
- **Part-2**: Learners have overview of AI development tools (Claude Code, Gemini CLI)
- **Chapter Index**: Part-3 positioning in book structure determines what learners know coming in

## Out of Scope for Part-3 Goals

- Specific Python or TypeScript syntax (covered in later parts)
- Building complete applications (Part-3 teaches skills; applications come in Parts 4+)
- Advanced AI agent techniques (multi-agent orchestration, custom tools - covered in later parts)
- Production deployment concerns (covered in Parts 9-13)

## Constraints

- **Beginner-Friendly**: No jargon without context; all technical terms explained
- **Business-Focused**: Frame skills around solving problems and building products, not abstract theory
- **Agent-Native**: Consistently position AI as collaborator, not tool
- **Non-Developer Accessible**: Assume zero programming background; build from first principles
- **Concise**: Part-3 introduction should be 400-600 words maximum to respect learner attention span
- **Actionable**: Goals must be concrete and achievable, not aspirational fluff

## Non-Goals

- Teaching programming fundamentals (variables, loops, functions) - covered in Parts 4-8
- Comparing all available AI coding tools - focus on Claude Code and Gemini CLI as exemplars
- Theoretical computer science or AI/ML concepts - focus on practical application
- Comprehensive coverage of all prompting techniques - focus on AIDD-specific techniques

## Notes for Implementation

When writing the Part-3 introduction/goals section:

1. **Tone**: Encouraging and empowering. "You don't need to memorize syntax - you need to learn how to think with AI."
2. **Structure**:
   - Hook: Why these skills matter (the paradigm shift)
   - Overview: What Part-3 covers (Chapter 9 + Chapter 10)
   - Learning Path: How the chapters build on each other
   - Success Vision: What learners will be able to do
3. **Language Patterns**:
   - Use "you will learn to..." rather than "this chapter covers..."
   - Use "AI as your co-pilot/partner" rather than "AI tool"
   - Use "direct AI agents" rather than "use AI"
   - Use "validation-first" to emphasize learner responsibility
4. **Connection to Book Philosophy**:
   - Reference colearning (human + AI learning together)
   - Reference spec-driven approach (will be applied in later parts)
   - Reference beginner accessibility (no gates based on prior experience)

## Validation Plan

The Part-3 goals will be validated by:

1. **Clarity Check**: Can a non-technical reader understand what they'll learn and why it matters?
2. **Alignment Check**: Do the goals align with actual Chapter 9 and Chapter 10 content?
3. **Motivation Check**: Does the introduction inspire learners to invest time in these skills?
4. **Realism Check**: Are the promised outcomes achievable within the scope of two chapters?
5. **Constitution Check**: Do the goals align with book philosophy (agent-native, beginner-friendly, business-focused)?
