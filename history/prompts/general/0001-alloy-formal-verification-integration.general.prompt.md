---
id: 0001
title: Alloy Formal Verification Integration
stage: general
date: 2025-11-29
surface: claude-code
model: claude-opus-4-5-20251101
feature: none
branch: main
user: mjs
command: conversation
labels: [formal-verification, alloy, software-abstractions, spec-architect, constitution]
links:
  spec: N/A
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: .claude/agents/engineering/spec-architect.md
    action: modified
    description: Added Alloy-style formal verification (Section 6) with invariant identification, small scope testing, counterexample generation
  - path: .claude/commands/sp.orchestrate.md
    action: modified
    description: Added Phase 1.5 Formal Verification routing with trigger conditions and output format
  - path: .specify/memory/constitution.md
    action: modified
    description: Added Principle 8 Formal Verification (Analyzable Over Informal) and Formal Verification Quality checklist
  - path: CLAUDE.md
    action: modified
    description: Added Step 5 Small Scope Verification, updated principles to 8, added Formal Verification Checklist
tests:
  - N/A (documentation/architecture changes)
---

## Prompt

User asked about knowledge of "Software Abstractions: Logic, Language, and Analysis" by Daniel Jackson (MIT Press) and requested education on key essentials. After comprehensive explanation of Alloy concepts, user asked if this knowledge should be integrated into the RoboLearn platform's RII components (constitution, CLAUDE.md, .claude/).

User selected Option C: Implement all three enhancements:
1. Enhance spec-architect agent with formal verification
2. Add routing to orchestrator
3. Update constitution and CLAUDE.md with formal verification principle

## Response snapshot

Implemented comprehensive Alloy-style formal verification across four RII components:

### 1. spec-architect Agent (v3.0)
- Added Section 6: Formal Verification (Alloy-Style Analysis)
- Small Scope Hypothesis: Generate 3-5 instances, test invariants
- Invariant Identification: Express as `∀ x: Type | constraint(x)`
- Relational Constraint Analysis: No cycles, coverage, uniqueness, reachability
- Counterexample Report Format
- Updated output format with Formal Verification Results section
- Added formal verification to self-monitoring checklist and success criteria

### 2. sp.orchestrate Command
- Added Phase 1.5: Formal Verification (Conditional)
- Trigger conditions: 5+ entities, safety-critical, multi-component, coverage requirements
- Formal verification output JSON structure
- Updated routing decision output to include formal verification
- Added to success/failure metrics

### 3. Constitution (v1.0.0 → Principle 8)
- Added Principle 8: Formal Verification (Analyzable Over Informal)
- Decision framework with complexity thresholds
- Verification techniques: Small scope, invariant identification, relational constraints
- Application examples with Alloy-style expressions
- Added Formal Verification Quality checklist

### 4. CLAUDE.md
- Added Step 5: Small Scope Verification (before stating understanding)
- Updated to 8 Core Principles
- Added Formal Verification Checklist (items 13-17)
- Updated success/failure metrics

## Outcome

- ✅ Impact: High - Enables systematic detection of specification bugs before implementation
- 🧪 Tests: N/A (documentation/architecture changes)
- 📁 Files: 4 files modified
- 🔁 Next prompts: Test formal verification with a real complex spec
- 🧠 Reflection: Alloy's small scope hypothesis is powerful for spec validation - most bugs visible with 3-5 instances

## Evaluation notes (flywheel)

- Failure modes observed: None - clean integration following existing patterns
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment: Apply formal verification to an actual platform spec (e.g., agent handoff protocol, hardware tier coverage) to validate the framework works in practice
