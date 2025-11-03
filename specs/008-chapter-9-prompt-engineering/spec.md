# Feature Specification: Chapter 9 - Prompt Engineering for AI-Driven Development

**Feature Branch**: `008-chapter-9-prompt-engineering`
**Created**: 2025-11-02
**Status**: Draft
**Input**: User description: "Design Chapter 9 Prompt Engineering as per the goal specified in Part 3. The chapter shall be designed for beginners focusing on AI-native Software Development which is reimagining CS education where AI is not just a tool but a collaborator. The chapter should be designed as beginner friendly, business focused and agent-native mindset."

**Target Audience**: Complete beginners (no programming experience required)
**Complexity Tier**: Tier 1 - Beginner (Parts 1-3)
**Part**: Part 3 - Working with AI Coding Agents
**Chapter Position**: Chapter 9 (First chapter in Part 3)

## Overview

Chapter 9 introduces prompt engineering as the foundational skill for AI-native software development. Unlike traditional programming education that focuses on memorizing syntax, this chapter teaches beginners how to communicate effectively with AI coding agents (Claude Code and Gemini CLI) to build software. The chapter positions the learner as an "AI orchestrator" who guides intelligent agents rather than a code writer who types every line.

This is the paradigm shift: readers learn to think with AI as a collaborative partner, expressing intent clearly and validating AI-generated outputs. The chapter emphasizes that clear prompts generate working code on the first try, while vague prompts lead to hours debugging AI mistakes. By mastering the 8-element AIDD prompting framework, beginners will construct prompts that generate usable code 70% of the time.

**Core Philosophy**: Prompting is WHAT you SAY to your AI agent. Students learn to articulate their software needs clearly, provide appropriate context, and iteratively refine AI outputs. This is not about memorizing commands—it's about thinking strategically and communicating precisely with an AI partner.

## User Scenarios & Testing

### User Story 1 - Understanding AI Coding Agents as Collaborative Partners (Priority: P1)

A complete beginner with no programming background needs to understand how AI coding agents work fundamentally differently from traditional tools (like autocomplete or search engines) before they can use them effectively. They need to grasp the concept of context windows, token-by-token generation, and why clear communication matters.

**Why this priority**: Without understanding how AI agents think and operate, learners will struggle to create effective prompts. This foundational mental model determines success in all subsequent lessons. This is the prerequisite for everything else.

**Independent Test**: Student can explain in their own words (without technical jargon) how an AI coding agent processes a prompt and why providing context improves results. Can be tested through a short explanation exercise or verbal check-in.

**Acceptance Scenarios**:

1. **Given** a beginner has never used AI coding tools, **When** they read the "How AI Agents Think" section, **Then** they can explain the concept of a context window in simple terms (e.g., "It's like the AI's short-term memory that holds our conversation and files")

2. **Given** a student understands context windows, **When** they see examples of good vs. bad prompts, **Then** they can identify which prompt will produce better results and explain why (e.g., "This prompt is better because it tells the AI what language to use and what the code should do")

3. **Given** a learner is writing their first prompt, **When** they consider what information to include, **Then** they remember to provide context (project type, language, what they want to achieve) rather than just issuing a command

---

### User Story 2 - Constructing Basic Developer Prompts (Priority: P2)

A beginner needs to write their first prompts that generate actual working code using clear commands and basic context. They practice the fundamental structure: Command (what to build) + Context (project info) + Basic Logic (step-by-step instructions).

**Why this priority**: This is the first hands-on application of prompt engineering concepts. Once students understand how AI agents work (P1), they need to immediately practice writing prompts that produce results. Quick wins build confidence.

**Independent Test**: Student writes 3-5 basic prompts (create function, debug code, explain concept) that produce usable AI responses. Can be validated by running prompts with Claude Code or Gemini CLI and checking if output makes sense.

**Acceptance Scenarios**:

1. **Given** a beginner wants to create a simple function, **When** they write a prompt using the Command + Context pattern, **Then** the AI generates working code that matches their intent (e.g., "Create a Python function that calculates the sum of two numbers")

2. **Given** a student has buggy AI-generated code, **When** they write a debugging prompt with context about the error, **Then** the AI identifies the issue and provides a fix

3. **Given** a learner encounters an unfamiliar programming concept in AI output, **When** they write a prompt asking for explanation, **Then** the AI explains the concept at an appropriate beginner level without jargon

4. **Given** a beginner is comparing their prompt to examples, **When** they review the 8-element framework checklist, **Then** they can identify which elements they included and which they missed

---

### User Story 3 - Adding Technical Constraints and Examples (Priority: P3)

A beginner needs to refine their prompts beyond basic commands by adding technical constraints (specific requirements like "use Python 3.11" or "handle errors gracefully") and providing examples of desired code style or patterns.

**Why this priority**: After mastering basic prompts (P2), students need to learn how to get more precise, project-specific code. This level of control transforms generic AI outputs into code that fits their actual project needs.

**Independent Test**: Student takes a working basic prompt and enhances it with 2-3 technical constraints and an example pattern, producing more tailored AI output. Can be tested by comparing before/after AI responses.

**Acceptance Scenarios**:

1. **Given** a student has a basic prompt that works but produces generic code, **When** they add technical constraints (Python version, error handling requirements, specific libraries), **Then** the AI generates code that meets those specific requirements

2. **Given** a learner has existing code with a particular style, **When** they include that code as an example in their prompt, **Then** the AI generates new code that matches the existing style and patterns

3. **Given** a beginner wants code formatted a specific way, **When** they specify formatting requirements in their prompt, **Then** the AI outputs code with proper structure (comments, docstrings, type hints as requested)

---

### User Story 4 - Using Question-Driven Development with AI (Priority: P4)

A beginner learns the most powerful AIDD technique: asking the AI to ask clarifying questions BEFORE generating code. Instead of accepting the first AI output, students learn to request 5-10 targeted technical questions that help produce the most appropriate solution.

**Why this priority**: This represents advanced prompt mastery. After students can write effective prompts (P2-P3), teaching them to collaborate iteratively with AI as a technical consultant produces dramatically better results.

**Independent Test**: Student initiates a question-driven workflow for a non-trivial task (e.g., "build a user authentication system") by prompting AI to ask 10 questions, provides thoughtful answers, and receives tailored code. Can be validated by comparing the specificity of final code vs. what a basic prompt would produce.

**Acceptance Scenarios**:

1. **Given** a student has a complex feature to build, **When** they prompt the AI to ask 10 clarifying questions before implementing, **Then** the AI asks relevant technical questions about requirements, constraints, and preferences

2. **Given** a learner receives clarifying questions from AI, **When** they provide detailed answers, **Then** the AI generates code specifically tailored to their answers (not generic boilerplate)

3. **Given** a beginner is debugging or optimizing code, **When** they use the question-driven pattern for troubleshooting, **Then** the AI asks diagnostic questions that lead to identifying root causes rather than surface symptoms

4. **Given** a student compares question-driven results with basic prompts, **When** they evaluate code quality, **Then** they can articulate why the question-driven approach produced better, more suitable code

---

### User Story 5 - Validating AI-Generated Code (Priority: P2)

A beginner must learn to validate AI outputs before using them. This is a safety-critical skill: students learn to read AI-generated code, ask questions about unfamiliar parts, test for correctness, and check for security issues (like hardcoded passwords). Validation is taught alongside generation.

**Why this priority**: As important as P2 (basic prompting). Students must NEVER blindly trust AI outputs. This validation mindset must be established from the very first prompt exercise. Teaching validation alongside generation prevents dangerous habits.

**Independent Test**: Student receives AI-generated code (intentionally containing a minor issue like missing error handling), reviews it using validation checklist, identifies the issue, and prompts AI to fix it. Can be tested with prepared scenarios.

**Acceptance Scenarios**:

1. **Given** a beginner receives AI-generated code, **When** they apply the validation checklist (read first, understand purpose, check for secrets, test it), **Then** they can identify whether the code is safe to use or needs review

2. **Given** a student encounters code they don't understand, **When** they prompt the AI to explain specific lines or concepts, **Then** the AI provides beginner-friendly explanations without jargon

3. **Given** a learner finds AI-generated code with hardcoded secrets or poor error handling, **When** they recognize these red flags, **Then** they prompt the AI to fix the issues rather than using the flawed code

4. **Given** a beginner tests AI-generated code and it fails, **When** they iterate with the AI by providing error context, **Then** the AI debugs and produces working code

---

### User Story 6 - Building a Personal Prompt Library (Priority: P5)

A beginner creates reusable prompt templates for common development tasks they'll encounter repeatedly (new API endpoint, bug fix, refactoring, adding tests). This accelerates future work and reinforces best practices.

**Why this priority**: After mastering core prompting skills (P1-P4), students benefit from systematizing their knowledge into reusable templates. This is a productivity optimization that assumes foundational competence.

**Independent Test**: Student creates 3-5 prompt templates for tasks relevant to their learning path, uses them successfully in later exercises or projects. Can be validated by template quality and successful reuse.

**Acceptance Scenarios**:

1. **Given** a student has successfully completed several prompt exercises, **When** they identify patterns in their effective prompts, **Then** they create template structures capturing those patterns

2. **Given** a learner has created prompt templates, **When** they encounter a new instance of a familiar task (e.g., "create another function"), **Then** they adapt their template rather than writing a prompt from scratch

3. **Given** a beginner reviews provided example templates (new feature, bug fix, refactoring), **When** they customize these for their needs, **Then** they can use them effectively in real scenarios

---

### Edge Cases

- **What happens when a beginner writes a completely vague prompt?**
  - Chapter teaches recognizing vague vs. specific prompts through before/after comparisons
  - Students see examples of AI confusion from vague prompts
  - Practice exercise: transform vague prompt into specific one using 8-element framework

- **How does a non-programmer validate code they don't fully understand?**
  - Validation section teaches asking AI targeted questions ("What does this line do?" "Why did you use X?")
  - Red flags checklist provides pattern-matching rules (hardcoded secrets, missing error handling)
  - Students learn "trust but verify" approach: run code in safe environment, ask questions, test functionality

- **What if AI generates code that seems overly complicated for a simple task?**
  - Chapter includes "simplicity" as a constraint students can specify
  - Example prompts show requesting "simplest possible solution" or "explain like I'm a beginner"
  - Students learn to iterate: "This seems complicated. Can you simplify it?"

- **How do beginners handle AI-generated code with errors or bugs?**
  - Iterative refinement section teaches error recovery prompting
  - Pattern: provide exact error message, describe unexpected behavior, ask AI to debug
  - Students practice correction loops rather than accepting first outputs

- **What if students over-rely on AI and don't learn underlying concepts?**
  - Every code example includes: specification → AI prompt → generated code → validation → explanation
  - "Why It Matters" sections connect commands to concepts
  - Exercises require explaining AI outputs in student's own words
  - Validation checklist forces engagement with code rather than blind copy-paste

- **How do beginners choose between Claude Code and Gemini CLI?**
  - Chapter briefly introduces both with simple guidance: "Both work well. This book uses Claude Code in examples, but concepts apply to any AI agent."
  - Focus on universal prompting principles rather than tool-specific features
  - Students learn transferable skills that work across AI platforms

## Requirements

### Functional Requirements

**Content Structure**

- **FR-001**: Chapter MUST introduce prompt engineering as the primary skill for AI-native development, positioning learners as "AI orchestrators" rather than code writers

- **FR-002**: Chapter MUST explain how AI coding agents work (context windows, token-by-token generation, why clear prompts matter) using non-technical analogies suitable for complete beginners

- **FR-003**: Chapter MUST teach the 8-element AIDD prompting framework: Command, Context, Logic, Roleplay, Formatting, Technical Constraints, Examples, and Iterative Questions

- **FR-004**: Chapter MUST provide beginner-friendly examples of each framework element with before/after comparisons showing vague vs. specific prompts

- **FR-005**: Chapter MUST include complete prompt examples for common tasks (create function, debug code, refactor, generate tests) that beginners can adapt and reuse

- **FR-006**: Chapter MUST teach validation as a core skill alongside prompt writing, including safety checklists (check for secrets, test code, understand before running)

- **FR-007**: Chapter MUST demonstrate question-driven development: prompting AI to ask clarifying questions before generating code

- **FR-008**: Chapter MUST show both Claude Code and Gemini CLI examples where appropriate, with tool-agnostic principles emphasized

**Pedagogical Approach (Tier 1 - Beginner)**

- **FR-009**: Chapter MUST limit new concepts to maximum 5 per section (cognitive load management for beginners)

- **FR-010**: Chapter MUST use the Concept-First pattern for every new tool/concept: WHAT (explain without jargon) → WHY (value for their work) → HOW (actual command) → PRACTICE (try with AI)

- **FR-011**: Chapter MUST position AI as the decision-maker for tool choices, with phrases like "Your agent knows which tool to use" and "You understand concepts; your agent executes"

- **FR-012**: Chapter MUST avoid presenting more than 2 options for beginners (e.g., teach Claude Code and Gemini CLI, not 5+ AI tools)

- **FR-013**: Chapter MUST remove theoretical scenarios and edge cases that beginners won't encounter in next 2 chapters, focusing only on immediately practical knowledge

- **FR-014**: Chapter MUST use real-world analogies and non-programmer examples before introducing technical definitions

- **FR-015**: All code examples MUST follow the pattern: problem statement → specification → AI prompt → generated code → validation steps → explanation of why it works

**Interactive Learning & Exercises**

- **FR-016**: Chapter MUST include 5-8 hands-on exercises where students write prompts with Claude Code or Gemini CLI and receive immediate feedback

- **FR-017**: Chapter MUST provide example prompt templates for common tasks that students can copy, customize, and reuse (new API endpoint, bug fix, refactoring, testing)

- **FR-018**: Chapter MUST include self-assessment checklist for prompt quality: "Did I include context? Did I specify the language? Did I explain what success looks like?"

- **FR-019**: Chapter MUST include at least one error recovery exercise: student receives buggy AI output, practices debugging prompts with error context

- **FR-020**: Chapter MUST include validation exercise: student reviews AI-generated code using checklist to identify potential issues (security, correctness, clarity)

**Business & Career Framing**

- **FR-021**: Chapter MUST frame prompt engineering around business value: shipping products faster, validating ideas without large teams, reducing iteration cycles

- **FR-022**: Chapter MUST include "real-world scenario" examples relevant to aspiring developers (building portfolio projects, creating MVPs, automating tasks)

- **FR-023**: Chapter MUST connect prompt quality to productivity metrics: "Clear prompts = 70% success rate on first try" vs. "Vague prompts = hours debugging"

**Constitution Alignment**

- **FR-024**: Chapter MUST teach specification-first workflow: every code example shows the specification that produced it

- **FR-025**: Chapter MUST emphasize AI as co-reasoning partner, not just coding assistant: students learn to think through problems before prompting

- **FR-026**: Chapter MUST demonstrate validation alongside generation: never show AI-generated code without validation steps

- **FR-027**: Chapter MUST include both Python AND TypeScript examples where appropriate (bilingual development approach)

**Content Quality Standards**

- **FR-028**: Chapter MUST avoid programming jargon without definition or use beginner-friendly alternatives ("variables" explained as "containers for storing information")

- **FR-029**: Chapter MUST use consistent voice and tone: encouraging, practical, focused on building things

- **FR-030**: Chapter MUST include visual aids where helpful: diagrams of context windows, flowcharts of prompt structure, annotated prompt examples

- **FR-031**: All sections MUST end with "Key Takeaways" summarizing 3-5 main points in bullet form

- **FR-032**: Chapter MUST conclude with clear next steps connecting to Chapter 10 (Context Engineering)

### Key Entities

**Educational Components**

- **Prompt Framework Element**: One of 8 components (Command, Context, Logic, Roleplay, Formatting, Technical Constraints, Examples, Questions). Each has: definition, purpose, beginner examples, common mistakes, best practices.

- **Code Example**: Demonstrates specific prompting technique. Includes: problem statement, specification, AI prompt used, generated code, validation steps, explanation. Must be tested and working.

- **Exercise**: Hands-on practice task. Includes: learning objective, instructions, starter prompt template, success criteria, solution example, common pitfalls.

- **Prompt Template**: Reusable structure for common tasks. Includes: template structure with placeholders, usage instructions, customization examples, when to use.

- **Validation Checklist**: Safety and quality checks for AI outputs. Includes: checklist items, explanation of each item, examples of passing/failing code, remediation steps.

**Learning Progression**

- **Lesson**: Self-contained learning unit covering 1-2 framework elements or techniques. Includes: concept introduction, examples, practice exercises, key takeaways.

- **Common Mistake**: Anti-pattern or pitfall beginners encounter. Includes: description, why it's problematic, before/after example, how to fix.

- **Tool Reference**: Brief guide for Claude Code or Gemini CLI. Includes: basic usage, simple example, when to use this tool vs. alternatives.

## Success Criteria

### Measurable Outcomes

**Learning Effectiveness**

- **SC-001**: 80% of students can write a basic prompt (Command + Context + Logic) that generates working code on first attempt after completing the chapter

- **SC-002**: Students can identify and fix at least 3 common prompt mistakes (vague commands, missing context, no validation) when reviewing example prompts

- **SC-003**: 90% of students can explain in their own words why clear prompts produce better code than vague prompts, using concepts from the chapter

- **SC-004**: Students can complete a 3-prompt exercise sequence (create, debug, refactor) independently within 20 minutes using the 8-element framework

**Skill Application**

- **SC-005**: Students successfully validate AI-generated code using the safety checklist (check for secrets, understand code, test functionality) in practice exercises

- **SC-006**: 75% of students create at least 3 reusable prompt templates for tasks they'll encounter in later chapters

- **SC-007**: Students can initiate question-driven development by prompting AI to ask 5-10 clarifying questions before implementation

- **SC-008**: Students demonstrate iterative refinement by taking initial AI output and improving it through 2-3 follow-up prompts with specific feedback

**Confidence & Mindset**

- **SC-009**: Students self-report increased confidence in using AI coding tools after completing the chapter (measured via brief survey)

- **SC-010**: Students adopt "AI as partner" mindset, as evidenced by using collaborative language ("I'll ask my AI to..." vs. "The AI will...") in exercises

- **SC-011**: 85% of students recognize when to validate AI outputs vs. when to trust them (based on code complexity and their understanding level)

**Business & Career Readiness**

- **SC-012**: Students can articulate at least 2 business benefits of effective prompt engineering (faster MVP development, reduced debugging time, ability to build without large teams)

- **SC-013**: Students successfully apply prompt engineering to build a small practical project (e.g., "create a simple to-do list API") that they could include in a portfolio

**Content Quality**

- **SC-014**: Chapter reading time is 45-60 minutes for beginner learners (measured through reading time estimates)

- **SC-015**: All code examples execute successfully when students run provided prompts with Claude Code or Gemini CLI

- **SC-016**: Students report that explanations are clear and jargon is appropriately defined (measured via feedback survey)

**Retention & Transfer**

- **SC-017**: Students successfully apply prompt engineering techniques learned in this chapter to subsequent chapters (measured by tracking prompt quality in later exercises)

- **SC-018**: 70% of students can transfer prompting skills to new AI tools not explicitly covered in the chapter (demonstrates understanding of universal principles)

## Assumptions

**Technology & Tools**

- Students have access to either Claude Code or Gemini CLI (chapter works with either)
- Students have internet connection for AI agent interactions
- Students have completed Parts 1-2 and understand basic software development concepts (what code is, what a project is, why testing matters)
- Students can read and execute basic command-line instructions (covered in earlier chapters)

**Learning Environment**

- Students are self-paced learners reading the chapter independently
- Students have 2-3 hours available for completing chapter content and exercises
- Students can run code in safe local environment or sandbox (established in earlier chapters)
- Students have basic text editor or IDE installed (from previous chapters)

**Pedagogical Approach**

- Beginner tier constraints apply: max 5 concepts per section, max 2 tool options, concept-first pattern
- Students learn best through immediate practice (exercises follow explanations closely)
- Visual aids and concrete examples are more effective than abstract theory for beginners
- Students benefit from reusable templates and checklists that reduce cognitive load

**Content Boundaries**

- Chapter focuses on prompt engineering fundamentals; context engineering is separate (Chapter 10)
- Advanced prompting techniques (multi-step chains, complex roleplay scenarios) are out of scope for Tier 1
- Tool-specific features are minimized; focus is on universal prompting principles
- Production deployment and security-hardening prompts are covered in later professional-tier chapters

**AI Capabilities**

- Claude Code and Gemini CLI maintain current capabilities throughout book lifecycle
- AI agents can understand beginner-level prompts and generate appropriate code
- AI models provide helpful error messages and debugging assistance when prompted
- Students understand AI can make mistakes (validation-first safety is explicitly taught)

## Dependencies

**Prerequisites (from earlier chapters)**

- Part 1: Introduction to AI-Native Development (mindset and philosophy)
- Part 2: Development Environment Setup (tools installed, basic command-line usage)
- Understanding of what software development involves (problems, solutions, code, testing)
- Familiarity with concept of specifications (what you want to build before building it)

**Enabling Later Content**

- Chapter 10: Context Engineering (builds on prompt engineering by adding context management)
- Part 4: Python Foundations (students use prompting to learn Python syntax)
- Part 5: TypeScript Fundamentals (students apply prompting to different language)
- Parts 6-13: All subsequent chapters assume prompt engineering competency

**External Dependencies**

- Claude Code or Gemini CLI availability and API access
- AI model capabilities (GPT-4, Claude 3, Gemini models)
- Example code repositories or sandbox environments for testing prompts
- Internet connectivity for AI agent interactions

## Out of Scope

**Advanced Prompting Techniques** (covered in later professional-tier chapters)

- Prompt chaining for complex multi-step workflows
- Advanced roleplay scenarios with multiple personas
- Meta-prompting and prompt optimization strategies
- Production-scale prompt engineering (cost optimization, rate limiting strategies)

**Tool-Specific Features**

- Deep dives into Claude Code vs. Gemini CLI differences
- MCP (Model Context Protocol) advanced features
- IDE integrations and extensions beyond basic usage
- Custom AI agent configurations or fine-tuning

**Context Engineering** (separate chapter)

- Progressive context loading strategies
- Memory files and session persistence
- Context compression techniques
- Multi-session context management

**Code Generation Specifics**

- Language-specific prompting strategies (covered in Python/TypeScript chapters)
- Framework-specific prompts (FastAPI, React, etc.)
- Database or infrastructure prompts (covered in backend/deployment chapters)
- Testing framework specifics beyond basic test generation

**Business & Production Concerns** (professional-tier content)

- Cost optimization for AI usage
- Compliance and regulatory prompting requirements
- Enterprise security constraints in prompts
- Team collaboration and shared prompt libraries

**Theoretical Depth**

- How large language models work internally (tokenization, transformers, attention)
- AI training and fine-tuning processes
- Prompt injection attacks and security research
- Academic research on prompt engineering effectiveness

## Notes & Open Questions

**Pedagogical Decisions to Validate with Users**

- **Concept load**: Is 5 concepts per section still too many for complete beginners? Should we reduce to 3-4 for this critical first chapter?

- **Tool choice**: Should we standardize on ONE AI tool (Claude Code) for all examples to reduce cognitive load, or maintain tool-agnostic approach with both Claude Code and Gemini CLI?

- **Exercise difficulty**: Are 5-8 hands-on exercises appropriate for a 60-minute chapter, or should we reduce to 3-4 with more guidance?

**Content Structure Questions**

- Should the 8-element framework be introduced all at once (overview) then detailed individually, OR introduced progressively (teach 2-3 elements, practice, then add more)?

- Should validation be a dedicated section at the end, OR integrated throughout (teach prompt element → show validation for that element)?

- Should we include a complete worked example at the end showing all 8 elements together, OR keep examples focused on 1-2 elements each?

**Accessibility & Inclusion**

- Are analogies and examples culturally universal, or do they assume specific cultural context?
- Do exercises accommodate different learning speeds (fast learners vs. those needing more time)?
- Are there alternative pathways for students who struggle with the primary teaching approach?

**Alignment with Constitution v3.0.0**

- Does the chapter adequately demonstrate AI as "co-reasoning partner" vs. just coding assistant?
- Are validation steps sufficiently emphasized alongside generation?
- Does the specification-first workflow come through clearly in examples?

**Technical Validation Needs**

- All code examples must be tested with current versions of Claude Code and Gemini CLI
- Prompt templates must be validated to actually produce described outputs
- Exercise solutions should be verified by having beginners (not the authors) complete them

**Future-Proofing Concerns**

- How do we handle rapid AI capability changes (models get better, prompting techniques evolve)?
- Should we create a companion online resource for updated examples as AI tools change?
- How do we balance teaching universal principles vs. current tool realities?

## Risk Analysis

**High Risk**: Students skip validation and blindly trust AI outputs
- **Mitigation**: Make validation mandatory part of every exercise. Include deliberate buggy examples that students must catch.
- **Backup Plan**: Add "common AI mistakes" section showing real failures if validation skipped.

**Medium Risk**: Chapter becomes outdated as AI tools evolve
- **Mitigation**: Focus on universal principles over tool-specific features. Use version-agnostic examples.
- **Backup Plan**: Maintain online errata/updates page for tool-specific changes.

**Medium Risk**: Beginners feel overwhelmed by 8-element framework
- **Mitigation**: Introduce progressively (3 core elements first, then add advanced). Provide simple templates.
- **Backup Plan**: If beta testers struggle, split chapter into "Basic Prompting" and "Advanced Prompting."

**Low Risk**: Students can't access Claude Code or Gemini CLI
- **Mitigation**: Provide alternative tools list. Ensure concepts work with any AI coding agent.
- **Backup Plan**: Create sandbox/playground environment for students without local AI access.
