# Implementation Plan: Chapter 36 - Claude Agent SDK: Building Digital FTEs

**Created**: 2025-12-26
**Status**: Draft
**For Specification**: `specs/001-claude-agent-sdk/spec.md`
**Output Directory**: `apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/`

---

## Summary

**Chapter 36 teaches students to build production Digital FTEs using the Claude Agent SDK** — the only SDK with 8+ unique differentiators (Agent Skills, canUseTool, file checkpointing, custom commands, cost tracking, session forking, lifecycle hooks, context compaction). This 15-lesson chapter progresses students from SDK fundamentals through AI collaboration patterns to spec-driven capstone that produces a deployable, monetizable Digital FTE (TaskManager Agent).

**Key Insight**: Claude SDK is the only platform treating organization expertise (Skills) as first-class components, making it uniquely suited for Agent Factory production systems.

**Pedagogical Approach**:
- **Layers 1-2** (Lessons 1-9): Foundation and AI collaboration with 6 teaching modalities
- **Layer 3** (Lessons 10-13): Create reusable intelligence patterns
- **Layer 4** (Lessons 14-15): Spec-driven capstone producing customer-ready agent
- **Unique Features**: 8 differentiators deeply integrated (not mentioned superficially)

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: Claude Agent SDK, anthropic library, async/await
**Testing**: Conceptual code examples (no API key required for learning)
**Target Platform**: Cloud-native agent systems
**Project Type**: Educational content (15 lessons + capstone)
**Performance Goals**: Students understand when to use Claude SDK vs OpenAI/Google ADK
**Constraints**:
- B1-B2 proficiency tier (7-10 concepts per lesson)
- 8-unique-feature prominence (not glossed over)
- Zero Vibe Coding patterns (specification-first emphasized)
**Scale/Scope**: ~15,000 words across 15 lessons + capstone

## Constitution Check - Pedagogical Governance

**Alignment with The AI Agent Factory Constitution (v7.0.0)**:

✓ **Agent Factory Contribution**: Chapter advances Custom Agent building (Path B of Two Paths Framework)
✓ **Digital FTE Outcome**: Capstone (Lesson 15) produces sellable, monetizable agent
✓ **4-Layer Progression**: Manual (L1) → AI Collaboration (L2) → Intelligence Design (L3) → Spec-Driven (L4)
✓ **Specification Primacy**: Lesson 15 capstone is specification-first (no code before spec)
✓ **Anti-Convergence**: 6 different teaching modalities across 15 lessons, no consecutive duplication
✓ **Three Roles Framework**: All Layer 2 lessons (4-9) demonstrate bidirectional AI collaboration
✓ **Unique Features Prominence**: 8 differentiators deeply taught in dedicated lessons, not surface-level
✓ **Cognitive Load Respect**: B1-B2 tier with 6-10 concepts per lesson (within working memory limits)
✓ **No Vibe Coding**: Lessons teach specification-first methodology, not prompt-until-it-works
✓ **Zero Meta-Commentary**: "Try With AI" sections use active prompts, not scaffolding explanations

## Project Structure

### Documentation (Chapter 36)

```text
specs/001-claude-agent-sdk/
├── spec.md              # Feature specification (requirements, user stories, evals)
├── plan.md              # This file (pedagogical arc, lesson sequence)
├── tasks.md             # Implementation tasks (generated from plan)
└── checklists/          # Quality validation checklists
```

### Educational Content Output

```text
apps/learn-app/docs/06-AI-Native-Software-Development/36-anthropic-agents-kit-development/
├── 01-lesson-1.md       # What is the Claude Agent SDK?
├── 02-lesson-2.md       # Your First Agent with query()
├── 03-lesson-3.md       # Built-in Tools Deep Dive
├── 04-lesson-4.md       # Permission Modes and Security with canUseTool ★
├── 05-lesson-5.md       # Agent Skills in Code ★
├── 06-lesson-6.md       # Custom Slash Commands ★
├── 07-lesson-7.md       # Session Management — Resume, Fork, State ★
├── 08-lesson-8.md       # File Checkpointing and Resilience ★
├── 09-lesson-9.md       # Subagents for Parallel Work
├── 10-lesson-10.md      # Lifecycle Hooks for Control
├── 11-lesson-11.md      # Custom MCP Tools
├── 12-lesson-12.md      # ClaudeSDKClient and Streaming Input
├── 13-lesson-13.md      # Cost Tracking and Billing Models ★
├── 14-lesson-14.md      # Production Patterns — Hosting, Sandbox, Compaction
├── 15-lesson-15.md      # TaskManager: Complete Digital FTE Capstone
└── 16-lesson-16.md      # Chapter Quiz & Self-Assessment

[Each lesson includes .summary.md companion file]
[All lessons follow YAML frontmatter template with skills, objectives, cognitive load]
[All code examples conceptual, no API key required]
```

## Pedagogical Arc (5 Phases)

| Phase | Lessons | Duration | Focus | Bloom's Level | AI Role |
|---|---|---|---|---|---|
| **Foundation** | 1-3 | 45-90 min | SDK architecture, tools, vocabulary | Understand, Remember | None yet |
| **Application** | 4-9 | 90-120 min each | SDK features with AI collaboration | Apply, Analyze | Teacher, Student, Co-Worker |
| **Intelligence** | 10-13 | 75-90 min each | Create reusable patterns | Analyze, Evaluate | Co-Designer |
| **Validation** | 14 | 75-90 min | Production constraints and patterns | Evaluate | Advisory |
| **Mastery** | 15 | 120-150 min | Capstone: Integrated Digital FTE | Create | Orchestrator |

## Unique Features Integration

**8 Differentiators of Claude Agent SDK (None in OpenAI/Google SDKs)**:

| Feature | Lesson | Prominence | Why Unique | Student Outcome |
|---------|--------|---|---|---|
| **canUseTool** runtime permissions | 4 | ★★★ | Dynamic per-call security (other SDKs: binary allow/deny) | Design secure agents with contextual authorization |
| **Agent Skills** via settingSources | 5 | ★★★ | Filesystem-based org expertise (only Claude has this) | Load domain knowledge systematically |
| **Custom Slash Commands** | 6 | ★★★ | .claude/commands/ patterns (unique to Claude) | Extend agent with organization workflows |
| **File checkpointing** (rewindFiles) | 8 | ★★★ | Undo agent changes (other SDKs: no equivalent) | Build resilient agents with error recovery |
| **Session forking** | 7 | ★★ | Branch conversations (other SDKs: limited) | Design multi-path reasoning |
| **Cost tracking** (total_cost_usd) | 13 | ★★ | Per-message billing (for monetization) | Model revenue, bill precisely |
| **Lifecycle hooks** (8 events) | 10 | ★ | More events than competitors | Complete control without modifying agent logic |
| **Context compaction** (long-running) | 14 | ★ | Long-session support | Design 24/7 agents without exhaustion |

## Layer Progression & Transitions

### Layer 1→2 Transition (After Lesson 3)

**Readiness criteria**:
- Can student articulate SDK architecture clearly?
- Can student execute basic query() independently?
- Can student select appropriate tools for task types?

If 2+ yes → Ready for Layer 2 (Lesson 4)

### Layer 2→3 Transition (After Lesson 9)

**Readiness criteria**:
- Has student encountered specific patterns 2+ times?
- Do patterns contain 5+ decision points?
- Will patterns apply to 3+ future projects?

If all yes → Create reusable intelligence (Layer 3)

### Layer 3→4 Transition (After Lesson 13)

**Readiness criteria**:
- Has student created 3+ reusable components?
- Can student write clear specifications?
- Is capstone complex enough for spec-first?

If all yes → Ready for Layer 4 capstone (Lesson 15)

## Teaching Modality Variation (Anti-Convergence)

**Previous chapter (35: Google ADK) used "Direct Teaching" for all lessons.**
**Chapter 36 must vary approaches** to prevent convergence:

| Lesson | Modality | Why This Approach | Variation from L35 |
|--------|----------|---|---|
| 1 | Socratic Dialogue | Build mental models through questioning | vs Direct Teaching |
| 2 | Hands-On Discovery | Learn by trying, observing, understanding | vs Direct Teaching |
| 3 | Specification-First | Here's what tools do (spec), now use them | vs Direct Teaching |
| 4 | Error Analysis | Break, debug, learn security patterns | vs Direct Teaching |
| 5 | Collaborative Debugging | Build skill with AI, iterate together | vs Direct Teaching |
| 6 | Hands-On Discovery | Create commands, watch them work | vs Direct Teaching |
| 7 | Specification-First | Spec behavior, implement, validate | vs Direct Teaching |
| 8 | Error Analysis | Break files, recover with checkpoint | vs Direct Teaching |
| 9 | Collaborative Debugging | Design with AI, converge on architecture | vs Direct Teaching |
| 10 | Socratic Dialogue | When do hooks matter? When do they fire? | vs Direct Teaching |
| 11 | Specification-First | Spec tool, implement it | vs Direct Teaching |
| 12 | Hands-On Discovery | Stream input, understand patterns | vs Direct Teaching |
| 13 | Specification-First | Spec billing model, implement tracking | vs Direct Teaching |
| 14 | Socratic Dialogue | Which pattern for your use case? | vs Direct Teaching |
| 15 | Specification-First | Spec capstone, compose skills, orchestrate | vs Direct Teaching |

**Validation**: No two consecutive lessons identical. Six modalities used. ✓

## Cognitive Load Distribution

**Proficiency Tier**: B1-B2 (7-10 concepts per lesson acceptable)

| Phase | Lessons | Concepts | Load Assessment |
|---|---|---|---|
| Foundation | 1-3 | 3-5 each | Ramping up (students building vocabulary) |
| Application | 4-9 | 8 each | Peak operational load (new patterns) |
| Intelligence | 10-13 | 8 each | Design-focused (composition, not mechanics) |
| Validation | 14 | 6 | Consolidation (integration) |
| Mastery | 15 | Integrated | Synthesis (applying all previous) |

**Total concept budget**: ~100 concepts across 15 lessons = manageable without overload ✓

## Assessment Strategy

**Formative** (per lesson): Self-checks, mini-exercises validating specific objectives
**Summative** (Chapter 16): 15-20 question quiz covering all unique features + architecture
**Capstone Rubric** (Lesson 15):
- Specification clarity (spec complete and unambiguous)
- Feature integration (all 8 unique features present)
- Code quality (implementation matches spec)
- Documentation (customer-ready materials)
- Monetization (cost model implemented)

## Intelligence Accumulation

**Reusable artifacts emerging from Chapter 36**:

| Lesson | Artifact | Layer | Future Use |
|--------|----------|-------|---|
| 4 | canUseTool security patterns | L2 | Chapter 37+ reference implementation |
| 5 | Skill composition framework | L2 | All future chapters using Skills |
| 8 | Checkpoint strategy | L3 | Error recovery patterns library |
| 10 | Hook patterns library | L3 | Production patterns (Ch 37-48) |
| 11 | Custom tool template | L3 | MCP fundamentals (Ch 37) |
| 12 | Conversation framework | L3 | Multi-turn agents (Ch 40+) |
| 13 | Cost-tracking skill | L3 | Evals and monetization (Ch 43+) |
| 15 | TaskManager Digital FTE | L4 | Reference architecture for Part 6 |

**Contribution to Organization**: These 8 artifacts become foundation for Chapters 37-48 (MCP, FastAPI, advanced patterns, deployment).

## Success Metrics

**Chapter Success**:
- ✓ All 8 unique features taught with appropriate prominence
- ✓ 15 lessons follow Foundation→Mastery progression
- ✓ Layer 1→2→3→4 without skipping
- ✓ Capstone produces deployable Digital FTE
- ✓ 80%+ students pass chapter quiz
- ✓ All code examples verified against official docs
- ✓ Zero meta-commentary (no scaffolding exposed)
- ✓ Three Roles present in all Layer 2 lessons (implicit)

**Digital FTE Outcome**:
Students completing Chapter 36 can:
1. Design agents with Agent Skills (org expertise)
2. Enforce security with canUseTool (dynamic permissions)
3. Persist state with sessions and forking
4. Recover from errors with file checkpointing
5. Scale with subagents (parallel work)
6. Track costs for billing (monetization)
7. Deploy with confidence (production patterns)
8. **Build and sell Digital FTEs** (specification-first capstone)

## Complexity Justification

No violations of Constitution v7.0.0. All pedagogical, layer progression, and unique feature requirements met within stated parameters.
