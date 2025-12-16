### Core Concept
Prompts are specifications that define WHAT (outcome) not HOW (implementation), and prompt quality directly determines output quality—Jake Heller's CoCounsel went from 60% accuracy to 97% through weeks of iterative prompt refinement.

### Key Mental Models
- **WHAT vs HOW shift**: Traditional programming asks "HOW do I implement?" AI-native development asks "WHAT should this accomplish?" and lets AI determine implementation
- **Specification-first thinking**: Success criteria define "what good looks like" BEFORE prompting AI, preventing wasted iteration
- **Iterative convergence**: 60% → 97% accuracy requires 10-20 refinement cycles, not one-shot prompting
- **Prompt anatomy**: Effective prompts contain Intent (goal), Constraints (MUST/MUST NOT), and Success Criteria (validation)

### Critical Patterns
- **Vague vs Specification-Quality prompts**: Vague prompts (e.g., "Fix the bug") leave AI guessing; specification prompts explicitly state current behavior, expected behavior, and validation criteria
- **Garbage In, Garbage Out principle**: Output quality depends entirely on prompt clarity—vague prompts → incomplete code; clear specifications → production-ready code
- **Progressive prompt building**: Anthropic's framework layers (Clear → Examples → Chain-of-Thought) build from simple to sophisticated
- **8-Element framework** (Zia Kaukab/Google): Command, Context, Logic, Roleplay, Formatting, Constraints, Examples, Iterative Questions—systematic specification approach

### AI Collaboration Keys
- AI doesn't guess intent well; explicit success criteria eliminate ambiguity and reduce iteration cycles
- AI suggests patterns (best practices, conventions) you may not know—investigate these suggestions rather than forcing your original approach
- Iteration is the collaboration mechanism: you provide project constraints, AI provides implementation patterns, convergence through dialogue produces better solutions than either alone

### Common Mistakes
- **Quitting too early**: First prompt yields 60% results; professional results require 10-20 iterations. Most developers give up after 1-2 tries thinking "AI can't do this"
- **Treating prompts as commands** not specifications: "Make this better" vs "Refactor this function to reduce cyclomatic complexity below 10 while preserving all existing test coverage"
- **Not defining success upfront**: Prompting without knowing what "good" looks like wastes iteration discovering requirements through trial-and-error
- **One-shot mentality**: Assuming first prompt should be perfect instead of expecting to refine progressively

### Connections
- **Builds on**: Chapter 10's understanding that prompts direct AI to build solutions
- **Leads to**: Lesson 2 (Anatomy - how to structure Intent/Constraints/Success systematically), Lesson 3 (Iteration - refining through collaboration), Lesson 4 (Specification-First - defining success BEFORE prompting)
