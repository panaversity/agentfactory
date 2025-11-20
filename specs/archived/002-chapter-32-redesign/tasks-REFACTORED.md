# Chapter 32: Real-World SpecKit Workflows — REFACTORED Task Checklist

**Feature Branch**: `002-chapter-32-redesign`
**Chapter Type**: AI-Native Development (Decomposition thinking + autonomous coordination)
**Status**: Planning Refactored — Ready for Implementation
**Last Updated**: 2025-11-06
**Estimated Total Effort**: 50-65 story points (8 lessons, focused on practical orchestration)

---

## Overview

This REFACTORED task checklist reflects the improved pedagogical approach discovered through collaborative planning:

**Key Changes from Original:**
1. ✅ **DELETED Lesson 5 (CI/CD)** - Not core to orchestration capability
2. ✅ **MOVED Sandboxing to Lesson 4** - Foundation for safe multi-session execution
3. ✅ **KEPT Lesson 5 as FIRST CLIMAX** - Manual 5-agent coordination (was L4)
4. ✅ **NEW Lesson 6: Contract-Based Coordination** - Replaces headless mode complexity
5. ✅ **REDESIGNED Lesson 7: SpecKit-Orchestrated Execution** - Uses SpecKit Plus as orchestrator (NOT bash scripts)
6. ✅ **RENUMBERED Lesson 8: Capstone** - Final proof (was L9)

**Why This is Better:**
- More practical (uses tools students already know)
- More AI-native (human + AI co-reasoning, not bash scripting)
- More honest (shows what works NOW vs future tooling)
- More pedagogically sound (clear progression to creative independence)

---

## North Star Vision: Create "Creative Orchestrators"

**THE TRANSFORMATIVE PROMISE**: After completing this chapter, students become **"creative orchestrators"** who can coordinate 5-10 autonomous AI agents through decomposition thinking and contract-based coordination.

**Two Climaxes Design (REFINED)**:

### FIRST CLIMAX (Lesson 5): Manual Parallel Coordination
**What:** Student acts as "team lead" coordinating 5 agent teams manually
**How:** Run SpecKit Plus workflows across 5 worktrees simultaneously
**Proof:** 2.5-3x measured speedup through decomposition
**Belief:** "I can manage agent teams like human teams, specs eliminate coordination chaos"

### SECOND CLIMAX (Lesson 7): SpecKit-Orchestrated Autonomous Execution
**What:** Student uses SpecKit Plus itself as orchestrator for 5-10 agents
**How:**
1. `/sp.specify` → generates contract.md
2. `/sp.plan` → multi-feature planning
3. `/sp.tasks` → parallelizable tasks (human can see)
4. SpecKit spawns 5-10 sessions (student opens terminals or uses tmux)
5. Hooks notify completion
6. Human reviews results

**Proof:** 5-10 features completed autonomously while human focuses on strategy
**Belief:** "I have creative independence - SpecKit coordinates execution, I focus on decomposition"

**KEY INNOVATION**:
- ❌ NO bash scripts with headless API (too complex, not practical at 5-10 scale)
- ✅ YES SpecKit Plus as orchestrator (uses familiar tools, hooks for coordination)
- ✅ YES contract.md as integration mechanism (clear contracts enable autonomous work)

---

## The Refined Student Journey (8 Lessons)

```
FOUNDATION (Lessons 1-3):
├─ L1: Git worktrees + parallel specs (2-3 agents)
├─ L2: Parallel planning + tasks
└─ L3: Parallel implementation + integration (proof decomposition works)

SAFETY FOUNDATION (Lesson 4):
└─ L4: Sandboxing (safe isolation for multi-agent coordination)

FIRST CLIMAX (Lesson 5):
└─ L5: Scaling to 5+ manual agents (team lead capability proven)

AUTONOMOUS COORDINATION (Lessons 6-7):
├─ L6: Contract-based coordination (contracts + hooks enable autonomy)
└─ L7: ✨ SECOND CLIMAX ✨ SpecKit-orchestrated execution
    ├─ SpecKit Plus generates contract.md
    ├─ SpecKit Plus plans multiple features
    ├─ SpecKit Plus spawns autonomous sessions
    ├─ Hooks notify completion
    └─ Human reviews (creative independence achieved)

PROOF OF MASTERY (Lesson 8):
└─ L8: Capstone project (portfolio-worthy demonstration)
```

**Graduate Identity**: "I am a creative orchestrator who coordinates autonomous agents through decomposition and contracts"

---

## Task List by Phase

### Phase 1: Chapter Structure & README

#### Task 1.1: Update Chapter README.md

**Priority**: MUST
**Effort**: 2 hours
**Changes from Original**: Update to reflect 8-lesson structure (not 9)

**Description**: Update existing README to reflect refined chapter structure with 8 lessons

**Acceptance Criteria**:
- [ ] README shows 8 lessons (not 9)
- [ ] Lesson 4: Sandboxing (moved earlier)
- [ ] Lesson 5: FIRST CLIMAX (clearly labeled)
- [ ] Lesson 6: Contract-Based Coordination (NEW, described)
- [ ] Lesson 7: SECOND CLIMAX - SpecKit-orchestrated execution (redesigned paradigm)
- [ ] Lesson 8: Capstone (renumbered from L9)
- [ ] Journey section updated to show L4→L5→L6→L7 progression
- [ ] NO mention of "headless bash scripts" (outdated approach)
- [ ] YES mention of "SpecKit Plus as orchestrator" (new paradigm)
- [ ] Time commitment: 10-12 hours (8 lessons)

**Technical References**:
- Current README: `book-source/docs/05-Spec-Driven-Development/32-real-world-spec-kit-workflows/README.md`
- Refactored tasks: This document
- Output style: `.claude/output-styles/chapters.md`

---

### Phase 2: Foundation Lessons (L1-3, UNCHANGED)

#### Task 2.1: Lesson 1 — Git Worktrees & Parallel Specifications
**Status**: ✅ COMPLETE (no changes needed)

#### Task 2.2: Lesson 2 — Parallel Planning & Tasks
**Status**: ✅ COMPLETE (no changes needed)

#### Task 2.3: Lesson 3 — Parallel Implementation & Integration
**Status**: ✅ COMPLETE (no changes needed)

---

### Phase 3: Safety Foundation & Scaling

#### Task 3.1: Lesson 4 — Sandboxing for Multi-Session Safety (MOVED from old Task 3.3)

**Priority**: MUST
**Effort**: 5 hours (content exists, needs adaptation)
**Changes**: Moved from L7 to L4, reframed as foundation

**Description**: Adapt existing sandboxing content (currently in 07-sandboxing-safe-session-isolation.md) to become Lesson 4, positioning it as safety foundation BEFORE scaling attempts

**Acceptance Criteria**:
- [ ] File: `04-sandboxing-multi-session-safety.md` (renamed from 07-)
- [ ] Duration: 90 minutes
- [ ] Framing updated: "Before coordinating 5+ agents, you need safe isolation"
- [ ] Content shows sandboxing enables L5-L7 (not just L8)
- [ ] YAML frontmatter: 4 skills at B1 level
- [ ] Cognitive load: 8 concepts (within B1 limit)
- [ ] Try With AI: 3 prompts about multi-session safety
- [ ] Scale connection: "This enables safe 5-10 agent coordination"
- [ ] Forward reference: "In L5 you'll coordinate 5 agents - sandboxing keeps them isolated"

**Implementation Guidance**:
1. Read existing `07-sandboxing-safe-session-isolation.md`
2. Keep technical content (sandbox setup, isolation tests)
3. Update framing: Move from "before Super Orchestrator" to "before scaling to 5+ agents"
4. Add forward references to L5 (FIRST CLIMAX)
5. Simplify: Remove references to "7-9 agent orchestration" (that's L7)

**Validation**:
- [ ] Sandboxing positioned as prerequisite for L5 scaling
- [ ] No references to bash orchestrator scripts
- [ ] Clear bridge to L5 manual coordination

---

#### Task 3.2: Lesson 5 — FIRST CLIMAX: Scaling Decomposition Thinking (RENUMBERED from L4)

**Priority**: MUST
**Effort**: 2 hours (content exists, just renumber and update references)
**Changes**: L4 → L5, update to reference L4 sandboxing

**Description**: Renumber existing Lesson 4 to Lesson 5, update internal references

**Acceptance Criteria**:
- [ ] File renamed: `04-scaling-decomposition-thinking.md` → `05-scaling-decomposition-thinking.md`
- [ ] YAML frontmatter updated: `lesson: 5`
- [ ] Introduction references L4: "Now that you understand sandboxing (L4)..."
- [ ] Forward reference to L6: "In L6, you'll learn contracts that enable autonomous coordination"
- [ ] FIRST CLIMAX label remains
- [ ] All content quality maintained

**Implementation**:
1. Rename file 04→05
2. Update YAML `lesson: 5`
3. Update intro to reference L4 sandboxing
4. Update forward references to L6 (not L5)
5. Verify all internal consistency

---

### Phase 4: Autonomous Coordination (NEW & REDESIGNED)

#### Task 4.1: Lesson 6 — Contract-Based Autonomous Coordination (NEW)

**Priority**: MUST
**Effort**: 8 hours (new content)
**Replaces**: Old L6 (Headless Mode) - DELETED

**Description**: Create NEW lesson teaching integration contracts, acceptance criteria, and completion hooks as foundation for autonomous agent coordination

**Learning Objective**: Students write integration contracts (contract.md) that enable 5-10 AI agents to work autonomously with clear boundaries, then use completion hooks for async coordination

**Skills Taught** (CEFR Framework):
- **Integration Contract Design** — B1 (Upper-Intermediate) — Technical — Students can write contracts defining feature boundaries and dependencies
- **Acceptance Criteria Definition** — B1 (Upper-Intermediate) — Technical — Students can define testable acceptance criteria for autonomous work
- **Async Coordination with Hooks** — B1 (Upper-Intermediate) — Technical — Students configure hooks for completion notification
- **Autonomous Work Requirements** — B1 (Upper-Intermediate) — Conceptual — Students understand what makes tasks autonomous-friendly

**Content Outline** (90 minutes):

1. **Why Contracts Enable Autonomy** (15 min)
   - Problem: At L5, you monitored 5 agents constantly (micromanagement)
   - Solution: Clear contracts = agents work autonomously, human reviews checkpoints
   - Key insight: Specifications define WHAT, contracts define HOW they integrate

2. **Writing Integration Contracts** (25 min)
   - contract.md structure
   - Defining "Provides" and "Depends on" for each feature
   - Integration points (APIs, data models, shared state)
   - Code Example 1: Complete contract.md for 3-feature system

3. **Defining Acceptance Criteria** (20 min)
   - What makes criteria "autonomous-friendly"
   - Testable, unambiguous, independent
   - Code Example 2: Acceptance criteria table

4. **Configuring Completion Hooks** (20 min)
   - Hook basics: `after-tool-use`, `on-session-complete`
   - Writing notification scripts
   - Code Example 3: Hook script writing to orchestrator-status.log
   - Code Example 4: Main orchestrator polling for ALL_COMPLETE

5. **Reflection: From Monitoring to Trusting** (10 min)
   - Journal: "What enables me to trust autonomous agents?"
   - Analysis: "How do contracts reduce coordination overhead?"

**Code Examples** (4 total):
1. **contract.md template** for e-commerce (auth, products, cart, payments, orders)
2. **Acceptance criteria table** with testable conditions
3. **Completion hook script** (.claude/hooks/notify-orchestrator.sh)
4. **Orchestrator polling script** (monitors orchestrator-status.log)

**Exercises** (4 total):
1. Write contract.md for provided 3-feature system
2. Define acceptance criteria for each feature
3. Configure completion hooks in test worktree
4. Simulate 3-agent completion, verify hooks work

**Try With AI** (3 prompts):
1. "Review my contract.md. Are integration points clear?"
2. "How would I configure hooks for 10 parallel sessions?"
3. "What acceptance criteria enable fully autonomous work?"

**Real-World Examples**:
- How Anthropic uses contracts for distributed teams
- How Vercel coordinates 50+ engineers with clear boundaries
- How solo founders use contracts to coordinate AI agents

**Validation**:
- [ ] contract.md template is complete and reusable
- [ ] Hooks actually work (tested with 3 worktrees)
- [ ] Students understand: "Contracts enable autonomy, not micromanagement"
- [ ] Forward reference to L7: "In L7, SpecKit Plus will generate these contracts for you"

---

#### Task 4.2: Lesson 7 — SECOND CLIMAX: SpecKit-Orchestrated Autonomous Execution (REDESIGNED)

**Priority**: MUST (THIS IS THE PAYOFF)
**Effort**: 10 hours (complete redesign)
**Replaces**: Old L8 (bash script orchestrator) - DELETED paradigm

**Description**: REDESIGN as SpecKit-orchestrated execution where SpecKit Plus itself acts as orchestrator, NOT bash scripts

**THE NEW PARADIGM**:

**Phase 1: SpecKit Generates Contract**
```bash
Human: "Build e-commerce platform"
Claude: /sp.specify "E-commerce with auth, products, cart, payments, orders. Include integration contracts."

Output:
- spec.md (vision)
- contract.md (integration contracts - NEW!)
```

**Phase 2: SpecKit Plans Multiple Features**
```bash
Claude: /sp.plan

Output:
- plan.md (5 features planned)

Claude: /sp.tasks

Output:
- tasks.md (15 parallelizable tasks)

[Human CAN see tasks.md - it's a file in the repo]
```

**Phase 3: SpecKit Coordinates Session Spawning**
```bash
Claude: "I've analyzed tasks.md. Ready to spawn 5 autonomous sessions:

Setup commands:
Terminal 2: cd worktree-001-auth && claude
Terminal 3: cd worktree-002-products && claude
Terminal 4: cd worktree-003-cart && claude
Terminal 5: cd worktree-004-payments && claude
Terminal 6: cd worktree-005-orders && claude

OR use tmux: ./spawn-sessions.sh (I'll generate this)

Each session has:
- contract.md (knows integration requirements)
- Sandboxed environment
- Completion hooks configured

I'll monitor orchestrator-status.log. Proceed?"
```

**Phase 4: Autonomous Execution**
```
Human: "Yes, go"
[Opens 5 terminals, starts 5 Claude sessions]
[Walks away for 2-3 hours]

Each Claude session:
- Reads contract.md
- Executes assigned tasks from tasks.md
- Writes completion to hook
- Works independently
```

**Phase 5: Completion Notification**
```
[Hook triggers]
Claude: "🎉 All 5 features complete!

Status:
✅ auth: 3 commits, tests passing
✅ products: 4 commits, tests passing
✅ cart: 2 commits, tests passing
✅ payments: 5 commits, tests passing
✅ orders: 3 commits, tests passing

Integration validation:
✅ All contracts satisfied
⚠️ Payments needs manual review (API key)

Ready to merge?"
```

**Learning Objective**: Students use SpecKit Plus as their orchestrator to coordinate 5-10 autonomous AI agent sessions through contracts, experiencing creative independence

**Skills Taught** (CEFR Framework - B2 level):
- **SpecKit-Based Orchestration** — B2 (Upper-Intermediate/Advanced) — Technical — Student uses SpecKit Plus commands to orchestrate multi-feature workflows
- **Contract Generation with AI** — B2 — Technical — Student co-creates contract.md via /sp.specify
- **Multi-Session Coordination** — B2 — Technical — Student spawns and monitors 5-10 sessions via hooks
- **Strategic Checkpoint Review** — B2 — Conceptual — Student reviews results strategically (not micromanagement)

**Content Outline** (90 minutes):

1. **The SpecKit Orchestrator Paradigm** (15 min)
   - NOT: Bash scripts with headless API (too complex)
   - YES: SpecKit Plus generates contracts, plans, tasks
   - YES: Hooks notify completion
   - Human focuses on: Decomposition (30 min) + Review (1 hour)
   - AI handles: Execution (2-3 hours autonomous)

2. **Phase 1: Generate Contract with SpecKit** (20 min)
   - Using /sp.specify to create contract.md
   - Code Example 1: /sp.specify prompt that generates contracts
   - Reviewing generated contract.md
   - Human + AI co-refine contracts

3. **Phase 2: Plan Multi-Feature System** (15 min)
   - /sp.plan for 5 features
   - /sp.tasks for parallelizable task breakdown
   - Code Example 2: tasks.md showing 15 parallelizable tasks
   - Human reviews: "Are tasks truly parallelizable?"

4. **Phase 3: Spawn Autonomous Sessions** (20 min)
   - Manual approach: Open 5 terminals, cd to worktrees, start Claude
   - tmux approach: Use generated script
   - Code Example 3: tmux session spawning script (generated by Claude)
   - Each session reads contract.md and tasks.md

5. **Phase 4: Autonomous Execution + Hooks** (10 min)
   - What happens while human is away
   - Hook writes to orchestrator-status.log
   - Main Claude polls for ALL_COMPLETE
   - Code Example 4: Hook notification flow

6. **Phase 5: Strategic Review** (5 min)
   - Review completion summary
   - Validate against contracts
   - Merge strategy
   - Integration testing

7. **Reflection: Creative Independence Achieved** (5 min)
   - "What enabled me to walk away for 2-3 hours?"
   - "How does this differ from L5 manual coordination?"
   - "What's the path from 5 to 10 to 50 agents?"

**Code Examples** (5 total):
1. `/sp.specify` prompt that generates contract.md
2. Generated tasks.md showing 15 parallelizable tasks
3. tmux session spawning script (generated by SpecKit)
4. Hook flow diagram + notification script
5. Integration validation checklist

**Exercises** (4 total):
1. Use /sp.specify to generate contract for provided project
2. Review tasks.md - identify which are parallelizable
3. Configure hooks and test with 3 sessions
4. Run full workflow: specify → plan → tasks → spawn → review

**Try With AI** (3 prompts):
1. "Help me refine this contract.md generated by /sp.specify"
2. "Review these tasks.md - which can run in parallel?"
3. "How would I scale this to 10 features? What would change?"

**Real-World Context**:
- Technical AI Product Manager workflow (this is the pattern)
- How solo founders ship 10-feature MVPs in weeks
- How Anthropic coordinates internal feature development

**KEY DISTINCTIONS** from old L8:
- ❌ NO bash scripts with `claude -p` (too complex)
- ❌ NO JSON parsing with jq (unnecessary)
- ❌ NO session ID management (not needed)
- ✅ YES SpecKit Plus commands (/sp.specify, /sp.plan, /sp.tasks)
- ✅ YES contract.md generation (practical)
- ✅ YES hooks for completion (simple, works)
- ✅ YES parallel interactive sessions (students already know)

**Honest Scoping**:
- Students WILL: Use SpecKit Plus to generate contracts and coordinate 5-10 sessions
- Students WON'T: Build production orchestrator (that's infrastructure work)
- Students WILL understand: Path to 50+ requires tooling, but pattern is the same

**Validation**:
- [ ] /sp.specify generates usable contract.md
- [ ] tasks.md shows clear parallelizable tasks
- [ ] Hooks notify completion successfully
- [ ] Students can spawn 5 sessions and walk away
- [ ] SECOND CLIMAX achieved: Creative independence proven
- [ ] NO mention of bash orchestrator scripts
- [ ] YES emphasis on SpecKit Plus as orchestrator

---

### Phase 5: Proof of Mastery

#### Task 5.1: Lesson 8 — Capstone Project & Measurement (RENUMBERED from L9)

**Priority**: MUST
**Effort**: 3 hours (minor updates only)
**Changes**: L9 → L8, update references

**Description**: Renumber existing capstone lesson, update to reference new lesson structure

**Acceptance Criteria**:
- [ ] File renamed: `09-capstone-project-measurement.md` → `08-capstone-project-measurement.md`
- [ ] YAML frontmatter: `lesson: 8`
- [ ] Student choice updated:
  - Option A: Manual coordination (L1-5)
  - Option B: SpecKit-orchestrated (L6-7)
- [ ] NO references to "headless bash scripts"
- [ ] YES references to "SpecKit orchestrator pattern"
- [ ] Reflection questions about contract-based coordination
- [ ] Portfolio narrative emphasizes decomposition + contracts

**Implementation**:
1. Rename file 09→08
2. Update YAML
3. Find/replace references to "L8" with "L7" (Super Orchestrator)
4. Update workflow options to reflect SpecKit orchestration
5. Ensure consistency

---

## File Operations Summary

**FILES TO DELETE**:
- `05-ci-cd-validation-hooks.md` (not core to orchestration)

**FILES TO RENAME**:
- `07-sandboxing-safe-session-isolation.md` → `04-sandboxing-multi-session-safety.md`
- `04-scaling-decomposition-thinking.md` → `05-scaling-decomposition-thinking.md`
- `08-building-the-super-orchestrator.md` → DELETE (redesigned as L7)
- `09-capstone-project-measurement.md` → `08-capstone-project-measurement.md`

**FILES TO CREATE**:
- `06-contract-based-autonomous-coordination.md` (NEW)
- `07-speckit-orchestrated-execution.md` (REDESIGNED)

**FILES UNCHANGED**:
- `README.md` (update content, keep name)
- `01-git-worktrees-parallel-specifications.md` ✅
- `02-parallel-planning-and-tasks.md` ✅
- `03-parallel-implementation-and-integration.md` ✅
- `06-headless-mode-programmatic-speckit-plus.md` → DELETE (replaced by new L6)

---

## Success Metrics (Updated)

**Chapter Completeness**: 8 lessons (was 9)
**Core Innovation**: SpecKit Plus as orchestrator (not bash scripts)
**Practical Value**: Contract-based coordination (works NOW)
**Graduate Identity**: "I am a creative orchestrator using SpecKit Plus to coordinate autonomous agents"

**Student Outcomes**:
- ✅ Can coordinate 5 agents manually (L5)
- ✅ Can write contracts enabling autonomy (L6)
- ✅ Can use SpecKit Plus as orchestrator for 5-10 agents (L7)
- ✅ Understands path to 50+ (tooling, not manual)
- ✅ Has portfolio-worthy capstone (L8)

**Time to Complete**: 10-12 hours (8 lessons, more focused)

---

## Next Steps for Implementation

1. ✅ Review this refactored tasks.md with human
2. Execute file operations (delete, rename, create)
3. Update README with 8-lesson structure
4. Create NEW L6 (Contract-Based Coordination)
5. Create REDESIGNED L7 (SpecKit-Orchestrated Execution)
6. Update L4, L5, L8 with new references
7. Final validation against refactored structure

**This refactored approach is:**
- More practical (uses familiar tools)
- More AI-native (human + AI co-reasoning)
- More honest (shows what works NOW)
- More pedagogically sound (clear progression)
- Better aligned with AIDD philosophy

**Ready for implementation!**
