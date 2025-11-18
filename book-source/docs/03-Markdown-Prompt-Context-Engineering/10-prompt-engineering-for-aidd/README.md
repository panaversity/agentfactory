# Chapter 10: Prompt Engineering for AI-Driven Development

**Part**: 3 — Markdown, Prompt, and Context Engineering
**Proficiency Range**: A2-B1 (CEFR)
**Duration**: ~4.5 hours (8 lessons × 30-60 min)
**Prerequisites**: Chapter 9 (Markdown fundamentals)

---

## Overview

This chapter teaches **prompt engineering** as a **specification skill** for AI-native development—the ability to communicate intent, constraints, and success criteria to AI systems with the same precision product managers use to specify requirements.

You'll learn the frameworks that Jake Heller (Casetext founder, $650M exit to Thomson Reuters) used to refine AI prompts from 60% accuracy to 97% accuracy through iterative specification writing. By the end of this chapter, you'll write prompts that consistently produce high-quality AI outputs and create reusable prompt templates for recurring development tasks.

**What This Chapter Teaches**:
- Prompts as specifications (WHAT to build, not HOW)
- Prompt anatomy and structure (Intent → Constraints → Success Criteria)
- Iterative refinement frameworks (60% → 97% through iteration)
- Specification-first prompting (define success before prompting)
- Question-Driven Development (AI asks clarifying questions)
- Reusable prompt templates for recurring tasks
- Template selection decision frameworks
- Writing peer-reviewable specifications

**What This Chapter Does NOT Teach** (See Chapter 11):
- Context engineering (context windows, token limits, progressive loading)
- Memory files (CLAUDE.md, architecture.md, decisions.md)
- Context compression and session management
- Tool selection based on context capacity

---

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Explain prompt engineering as specification skill** (A2, Understand)
   - Distinguish between specification prompts (WHAT) and vague requests
   - Analyze prompts for completeness using Intent-Constraints-Success framework

2. **Write structured prompts with clear intent** (A2-B1, Apply)
   - Construct prompts using Intent → Constraints → Success Criteria structure
   - Apply 8 technical action verbs (Create, Debug, Refactor, Analyze, Optimize, Generate, Explain, Validate)

3. **Apply iterative refinement to improve prompt quality** (B1, Apply)
   - Identify specification gaps in AI outputs
   - Refine prompts through iteration to achieve 90%+ quality

4. **Write specification documents before prompting** (B1, Apply)
   - Define measurable success criteria
   - Specify constraints and non-goals explicitly
   - Create validation tests for acceptance criteria

5. **Use Question-Driven Development to elicit requirements** (B1, Apply)
   - Prompt AI to ask clarifying questions
   - Transform AI questions and your answers into structured specifications

6. **Create reusable prompt templates** (B1, Create)
   - Recognize recurring prompt patterns (2+ occurrences)
   - Design templates with placeholders and usage guidance
   - Evolve templates based on usage patterns

7. **Apply template selection decision frameworks** (B1, Evaluate)
   - Classify tasks by category (Diagnostic, Transformative, Generative, Explanatory, Evaluative)
   - Evaluate template applicability in \<30 seconds
   - Decide between template, custom prompt, or hybrid approach

8. **Write peer-reviewable specifications** (B1, Create)
   - Produce specification documents implementable without clarification
   - Define measurable success criteria and validation tests
   - Identify and document open questions

---

## Chapter Structure (4-Stage Progression)

This chapter follows the constitutional **4-Stage Teaching Framework**:

### Stage 1: Manual Foundation (Lessons 1-2)
**Teaching Modality**: Specification-first instruction

**Lessons**:
- **Lesson 1**: Prompts as Specifications (30 min)
  - Jake Heller's framework (60% → 97% accuracy)
  - WHAT vs HOW thinking
  - Specification-first principle
  - 5 examples (bug fix, refactoring, documentation, features, testing)

- **Lesson 2**: Anatomy of Effective Prompts (35 min)
  - Intent → Constraints → Success Criteria structure
  - 8 technical action verbs
  - 4 constraint types (technical, scope, quality, format)
  - Anti-patterns and common mistakes

**Student Capability After Stage 1**: Analyze prompts for specification quality, distinguish vague vs specific prompts

---

### Stage 2: AI Collaboration (Lessons 3-5)
**Teaching Modality**: Socratic dialogue + hands-on

**Three Roles Demonstrated**: AI as Teacher (suggests patterns), AI as Student (learns constraints), AI as Co-Worker (iterates to convergence)

**Lessons**:
- **Lesson 3**: Iterative Prompt Refinement (40 min)
  - Jake Heller's iteration framework
  - Git commit message refinement example (5 iterations, 40% → 95%)
  - Convergence criteria (success, diminishing returns, time budget)
  - **Try With AI**: 5-part active collaboration

- **Lesson 4**: Specification-First Prompting (40 min)
  - Writing specifications BEFORE prompting
  - File backup script example (specification → validation → prompt → implementation)
  - 5-section specification structure
  - **Try With AI**: Write specification, prompt AI, validate

- **Lesson 5**: Question-Driven Development (35 min)
  - Prompting AI to ask clarifying questions
  - QDD workflow (questions → answers → spec → validation → implementation)
  - Meta-prompt structure ("Before implementing, ask 5-8 questions about...")
  - **Try With AI**: Use QDD for Git workflow guide

**Student Capability After Stage 2**: Write specification-first prompts, use QDD to elicit requirements, iterate prompts to 90%+ quality

---

### Stage 3: Intelligence Design (Lessons 6-7)
**Teaching Modality**: Design-focused (creating reusable patterns)

**Reusable Intelligence Created**: Prompt templates as skills (2-4 decision points each)

**Lessons**:
- **Lesson 6**: Creating Reusable Prompt Templates (45 min)
  - Template creation criteria (recurs 2+ times, consistent structure, cross-project value)
  - 3 templates provided: Bug Fix, Refactoring, Documentation
  - Template structure (purpose, sections, placeholders, usage notes)
  - Template evolution (v1 → v2 → v3 based on usage)
  - **Try With AI**: Create template for personal recurring task

- **Lesson 7**: Template Selection and Decision Frameworks (35 min)
  - 3-question decision tree (recurring? template exists? assumptions match?)
  - 5 task categories (Diagnostic, Transformative, Generative, Explanatory, Evaluative)
  - Template vs custom tradeoff analysis
  - Hybrid approach (template + custom sections)
  - **Try With AI**: Evaluate 5 tasks, select template or custom

**Student Capability After Stage 3**: Create and organize reusable prompt templates, select appropriate template in \<30 seconds

---

### Stage 4: Spec-Driven Integration (Lesson 8)
**Teaching Modality**: Specification-driven (no implementation, specification only)

**Orchestration Task**: Write complete specification for Prompt Template Library tool

**Lesson**:
- **Lesson 8**: Capstone — Prompt Template Library Specification (60 min)
  - Integrate all Chapter 10 concepts into specification document
  - Define measurable success criteria
  - Specify 5-7 core features with user workflows
  - Create 8-10 validation tests
  - Produce peer-reviewable specification (implementable without clarification)
  - **Optional Extension**: Use QDD with AI to refine specification to v1.1

**Student Capability After Stage 4**: Write complete, peer-reviewable specifications for AI-assisted tools

---

## Cognitive Load Management

**Concept Distribution** (by lesson):
- L1: 5 concepts (prompts as specs, WHAT vs HOW, spec-first, iteration, quality=output)
- L2: 8 concepts (intent, constraints, success, action verbs, anti-patterns)
- L3: 6 concepts (iteration, convergence, quality thresholds, Three Roles)
- L4: 7 concepts (specification-first, success criteria, constraints, non-goals, validation)
- L5: 6 concepts (QDD, clarifying questions, assumption discovery, requirement elicitation)
- L6: 8 concepts (templates, pattern recognition, template structure, evolution)
- L7: 7 concepts (decision frameworks, template applicability, task classification)
- L8: 5 concepts (specification integration, template library architecture, validation)

**Total**: 52 concepts across 8 lessons (avg 6.5 per lesson, within B1 cognitive load limits)

**Tier Alignment**:
- A2 tier: Lessons 1-2 (5-8 concepts, heavy scaffolding, 2 options max)
- B1 tier: Lessons 3-8 (6-8 concepts, moderate scaffolding, 3-4 options)

---

## Practice Vehicles (NO Python)

**Why no Python**: Students in Part 3 have not learned programming yet (Python starts Chapter 12, Part 4).

**Practice domains used**:
- **Bash commands** (file backup scripts, log monitoring, Git operations)
- **Git workflows** (commit messages, branch strategies, code review)
- **Markdown documents** (documentation, specifications, README files)
- **Conceptual tasks** (analyzing prompts, classifying task types, decision frameworks)

**Pedagogical rationale**: Practice prompt engineering through accessible tools students already know (Bash, Git, Markdown from Chapters 8-9).

---

## Constitutional Compliance

This chapter adheres to Constitution v6.0.1:

**Principle 1: Specification Primacy** ✓
- Every lesson demonstrates WHAT before HOW
- Lesson 8 capstone is specification-only (no implementation)

**Principle 2: Progressive Complexity** ✓
- 4-Stage progression (Manual → Collaboration → Intelligence → Spec-Driven)
- Cognitive load validated (A2: 5-8 concepts, B1: 6-8 concepts)

**Principle 3: Factual Accuracy** ✓
- Jake Heller claims verified with YouTube timestamps
- All examples tested and validated
- No unverified productivity claims

**Principle 4: Coherent Structure** ✓
- Each lesson builds on previous (L1 foundation → L2 structure → L3-5 application → L6-7 intelligence → L8 integration)
- Prerequisites explicit (Chapter 9 Markdown knowledge)

**Principle 5: Intelligence Accumulation** ✓
- Lessons 6-7 create reusable templates (captured intelligence)
- Lesson 8 orchestrates accumulated concepts

**Principle 6: Anti-Convergence** ✓
- Chapter 9 used direct teaching modality
- Chapter 10 uses specification-first + Socratic modality (distinct approach)
- 4 modalities within chapter (specification-first, Socratic, design-focused, spec-driven)

**Principle 7: Minimal Content** ✓
- All sections map to learning objectives
- No context engineering content (deferred to Chapter 11)
- Focus exclusively on prompt communication (WHAT you say to AI)

**Student-Facing Language Protocol** ✓
- No meta-commentary about stages or frameworks
- No scaffolding exposure (students experience Three Roles without studying framework)
- Only "Try With AI" sections include AI tool usage

---

## Quick Reference: Lesson Summaries

| Lesson | Title | Duration | Stage | Key Concepts |
|--------|-------|----------|-------|--------------|
| 1 | Prompts as Specifications | 30 min | Stage 1 | WHAT vs HOW, specification-first, Jake Heller framework |
| 2 | Anatomy of Effective Prompts | 35 min | Stage 1 | Intent-Constraints-Success, 8 action verbs, anti-patterns |
| 3 | Iterative Prompt Refinement | 40 min | Stage 2 | Three Roles, iteration loop, convergence criteria |
| 4 | Specification-First Prompting | 40 min | Stage 2 | Write specs before prompting, validation tests |
| 5 | Question-Driven Development | 35 min | Stage 2 | AI asks questions, QDD workflow, meta-prompts |
| 6 | Creating Reusable Prompt Templates | 45 min | Stage 3 | Template structure, 3 templates (bug/refactor/docs), evolution |
| 7 | Template Selection Decision Frameworks | 35 min | Stage 3 | 3-question decision tree, 5 task categories, template vs custom |
| 8 | Capstone: Prompt Template Library | 60 min | Stage 4 | Complete specification writing, orchestration |

**Total Duration**: ~4.5 hours (270 minutes)

---

## Tools Required

**AI Coding Assistants** (at least one):
- Claude Code (VS Code extension or CLI)
- GitHub Copilot Chat
- Cursor
- Codeium Chat
- Gemini Code Assist
- Any AI assistant with code context awareness

**Development Tools**:
- Terminal/Command line (Bash commands)
- Git (version control operations)
- Text editor (VS Code, Vim, Sublime, etc.)
- Markdown preview capability

**No Programming Knowledge Required**: This chapter uses Bash commands, Git, and Markdown (all covered in Chapters 8-9). Python programming starts in Part 4 (Chapter 12).

---

## Additional Resources

**Authoritative Sources**:
- Jake Heller (Casetext founder) YC Talk: [Building CoCounsel with GPT-4](https://www.youtube.com/watch?v=l0h3nAW13ao)
  - Prompt refinement: [20:03]
  - Define success criteria: [15:27-18:16]
  - Steps to prompts: [12:52]

**Related Chapters**:
- **Chapter 9**: Markdown fundamentals (prerequisite)
- **Chapter 11**: Context Engineering for AI-Driven Development (continuation)
- **Chapter 12+**: Python programming (enables implementing Lesson 8 specification)

**Constitutional Framework**:
- `.specify/memory/constitution.md` (v6.0.1) — Pedagogical principles governing chapter design

---

**Next Chapter**: [Chapter 11 — Context Engineering for AI-Driven Development →](../11-context-engineering-for-ai-driven-development/README.md)
