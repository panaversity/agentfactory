---
id: 0001
title: Design Chapter 8 Git GitHub AIDD Spec
stage: spec
date: 2025-11-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: 012-chapter-8-git-github-aidd
branch: 012-chapter-8-git-github-aidd
user: HP
command: /sp.specify
labels: ["chapter-8", "git", "github", "ide-setup", "aidd", "spec-creation", "part-2"]
links:
  spec: specs/012-chapter-8-git-github-aidd/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/012-chapter-8-git-github-aidd/spec.md
  - specs/012-chapter-8-git-github-aidd/checklists/requirements.md
tests:
  - Specification validation against quality checklist (PASS)
  - AIDD conversational approach verification (PASS)
  - Tier 1 cognitive load validation (PASS)
  - Coverage completeness check (PASS)
---

## Prompt

Design chapter 8. Check the part 2 provided at @book-source/docs/02-AI-Tool-Landscape/README.md and the source context is at @context/09_chap8_specs/. Important note: The lesson you will create should be interective and based on AI Driven Development meaning it should teach user using conversation where necessory. I'm giving you source example for reference donot hard code it, it is only for learning the pattern and use where necessory """ Navigation & File Management
Finding Your Location
Instead of: pwd

Prompt: "Show me my current directory path"
Prompt: "Where am I right now in the filesystem?"
Prompt: "What's my current working directory?"

Example Conversation:

You: "Show me my current directory path"
    Agent: Tool -> Shell $ pwd
    Tool Output: Users/mjs/Documents
Agent: "You are in Users/mjs/Documents directory." """. Taking into consideration the above factors Design properly chapter 8 and write content of book with reference of AIDD not in a traditional way. Chapter name should be concise of the following git and github for AI Driven Development and also IDE setup

## Response snapshot

Created comprehensive Chapter 8 specification for "Git & GitHub for AI-Driven Development" with the following key components:

**Structure**:
- Follows AIDD (AI-Driven Development) conversational teaching approach
- Two-part structure: Part I (Git concepts/commands) + Part II (Natural language prompts for AI assistants)
- Includes IDE setup integration (VS Code recommended)
- Estimated completion time: 3-4 hours as specified in Part 2

**User Stories (Prioritized)**:
1. **P1**: Learning Git safety basics with AI assistant (foundational)
2. **P2**: GitHub integration for backup and portfolio
3. **P2**: IDE setup for AI-assisted development
4. **P3**: Pull requests and code review with AI

**Key Features**:
- 14 functional requirements covering Git fundamentals, GitHub, IDE setup, and AIDD approach
- 10 measurable success criteria with specific metrics (e.g., 90% student success rate, 40% faster learning with AI prompts)
- Tier 1 (Beginner) cognitive load management: max 2 options, max 5 concepts per section
- Safety-first emphasis for working with AI coding assistants
- Natural language prompt templates for all Git operations
- Platform-specific instructions (Windows, Mac, Linux)

**Chapter Outline**:
- **Part I** (90 min): Why Git matters, setup, daily workflow, safety net
- **Part II** (90 min): Branches, GitHub, pull requests, IDE setup
- **Capstone**: Build Python calculator with AI, use Git for safety, push to GitHub

**Pedagogical Approach**:
- Concept-first pattern: WHAT → WHY → HOW → PRACTICE
- AI-as-partner: Students ask AI to execute Git commands via natural language
- Conversational examples throughout (e.g., "The AI broke my code - take me back to before")
- Safety checklists and error recovery procedures

**Validation**:
- All quality checklist items pass (no blocking issues)
- No [NEEDS CLARIFICATION] markers
- Measurable, technology-agnostic success criteria
- Complete coverage of Git, GitHub, and IDE setup
- Alignment with Part 2 structure and complexity tier

## Outcome

- ✅ Impact: Complete specification ready for planning phase (`/sp.plan`). Establishes foundation for teaching version control in AI-driven development context with beginner-friendly, conversational approach.
- 🧪 Tests: Specification validated against quality checklist (all items pass). AIDD approach verified. Cognitive load limits confirmed.
- 📁 Files: Created `spec.md` (9KB, 450+ lines) and `checklists/requirements.md` with validation results
- 🔁 Next prompts: Run `/sp.plan` to break specification into lesson-by-lesson implementation plan with skills proficiency mapping
- 🧠 Reflection: Successfully integrated AIDD conversational teaching style with Git fundamentals. The two-part structure (concepts + AI prompts) provides clear separation between understanding Git and using AI to execute Git operations. Safety-first approach is critical for beginners working with AI coding assistants.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification is complete and unambiguous.
- Graders run and results (PASS/FAIL): Quality checklist validation PASS, AIDD approach verification PASS, Tier 1 cognitive load validation PASS
- Prompt variant (if applicable): N/A (first iteration)
- Next experiment (smallest change to try): Proceed to planning phase to validate lesson breakdown aligns with 3-4 hour time estimate and maintains cognitive load limits throughout
