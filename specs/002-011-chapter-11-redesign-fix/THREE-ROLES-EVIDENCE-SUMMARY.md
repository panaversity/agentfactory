# Three Roles Framework Evidence — Lessons 3-5

**Feature**: 002-011-chapter-11-redesign-fix
**Date**: 2025-11-18
**Agent**: content-implementer v1.0.0

---

## What Is Three Roles Framework?

The Three Roles framework demonstrates **bidirectional learning** where:
- **AI as Teacher**: AI suggests patterns student didn't know
- **AI as Student**: AI learns project constraints from student
- **AI as Co-Worker**: Multiple rounds of iteration converge on solution better than either alone

**Critical**: Students **EXPERIENCE** these roles through natural collaborative narrative, NOT through explicit pedagogical labels.

---

## Lesson 3: Progressive Loading Strategy

### Evidence Location
**Section**: "Collaborative Loading Strategy Design"

### AI as Teacher (Pattern Suggestion)
**Transcript excerpt**:
> **Your request:**
> "I'm working on a documentation project with 50+ markdown files. Today I'm writing lesson 1 for the Fundamentals chapter. What files should I load to give you the right context without wasting tokens?"
>
> **AI's recommendation:**
> "For efficient context management, I'd suggest a three-tier approach:
> 
> **Foundation (always load)**: CLAUDE.md, chapter-index.md, style-guide.md — these establish project-wide patterns
> 
> **Current (today's task)**: 02-fundamentals/outline.md, any completed lessons in chapter 02 for consistency
> 
> **On-Demand (fetch if needed)**: Examples from other chapters, only when we need specific reference patterns"

**What AI taught**: Foundation/Current/On-Demand three-tier loading model student hadn't formulated

---

### AI as Student (Learning Project Constraints)
**Transcript excerpt**:
> **Your response:**
> "That makes sense as a general approach, but our style-guide.md changes frequently as we refine standards. Should it still be Foundation, or would that risk loading stale versions?"
>
> **AI's adaptation:**
> "Good point. For frequently-changing files, Foundation isn't ideal—you'd load outdated patterns. Let's adjust:
>
> **Foundation (stable)**: CLAUDE.md (core patterns rarely change), chapter-index.md (structure is stable)
>
> **Current (include fresh version)**: style-guide.md (fetch latest each session), 02-fundamentals/outline.md, completed lesson files"

**What AI learned**: style-guide.md volatility constraint (student taught AI that frequent changes disqualify Foundation tier)

---

### AI as Co-Worker (Iterative Convergence)
**Convergence loop** (3 rounds):

1. **Round 1**: AI suggests general Foundation/Current/On-Demand allocation
2. **Round 2**: Student teaches AI about style-guide.md volatility → AI adapts to move it from Foundation to Current
3. **Round 3**: Student teaches AI about chapter-index.md criticality → AI revises budget to accommodate oversized Foundation file

**Final result** (from "What Emerged from Collaboration"):
> Through three rounds of iteration, you arrived at a loading strategy that:
> - Started with AI's general recommendation (Foundation/Current/On-Demand)
> - Incorporated your domain knowledge (style-guide.md changes frequently)
> - Refined based on project priorities (chapter-index.md is critical despite size)

**What neither had alone**: Loading strategy balancing efficiency (minimal Foundation) with effectiveness (critical references loaded)

---

## Lesson 4: Context Compression and Session Restart

### Evidence Location
**Section**: "Creating a Checkpoint: Collaborative Extraction"

### AI as Teacher (Checkpoint Structure)
**Transcript excerpt**:
> **Your request:**
> "We've been working for 90 minutes and we're at 85% context utilization. I need to create a checkpoint so we can restart with clean context. Help me extract the key decisions and progress we've made."
>
> **AI's initial checkpoint draft:**
> ```markdown
> # CHECKPOINT — Authentication Session Ending 2025-11-18 11:30 AM
>
> ## Architectural Decisions Made:
> 1. **Authentication Method**: Password + JWT tokens
>    - Why: Balance security with stateless API design
>    - Rejected: OAuth (too complex for MVP)
> [... 720 tokens total]
> ```

**What AI taught**: Checkpoint structure (Decisions/Progress/Next Steps/Context to Preserve) that student hadn't designed

---

### AI as Student (Learning Size Constraint)
**Transcript excerpt**:
> **Your response:**
> "This is comprehensive, but it's 720 tokens—too large. Our target is <600 tokens. What can we compress without losing essential context?"
>
> **AI's analysis:**
> "Let's compress by removing redundancy and condensing explanations:
>
> **What we can remove**:
> - Rejected alternatives (OAuth, Argon2) — interesting but not actionable
> - Implementation details already in code
> - Timestamp precision
>
> **What we must keep**:
> - Core architectural decisions
> - Non-obvious constraints
> - Deferred items"

**What AI learned**: 600-token size constraint + what's essential vs redundant (student taught AI compression priorities)

---

### AI as Co-Worker (Iterative Compression)
**Convergence loop** (2 rounds):

1. **Round 1**: AI creates comprehensive 720-token checkpoint preserving everything
2. **Round 2**: Student teaches size constraint → AI compresses to 487 tokens (30% reduction)

**Final result** (from "What Emerged from Compression"):
> Through two rounds of iteration, you arrived at a checkpoint that:
> - Started with AI's comprehensive draft (preserved everything)
> - Identified compression targets through your feedback (what's essential vs redundant)
> - Converged on a concise format that balances preservation with efficiency

**What neither had alone**: Checkpoint capturing **what you need to know** without conversational noise

---

## Lesson 5: Context Isolation and Parallel Tasks

### Evidence Location
**Section**: "Deciding When to Isolate"

### AI as Teacher (Similarity Scoring Framework)
**Transcript excerpt**:
> **Your request:**
> "I have four tasks to complete today. Should I work on them in the same session or create separate sessions?"
>
> **AI's analysis:**
> "Let's evaluate task similarity to decide:
>
> **Task A + Task B** (Auth bug + Auth logging):
> - Same domain (authentication): +30
> - Same service (auth service): +20
> - Same files (api/auth.py likely): +15
> → Score: 65 points → WORK TOGETHER
>
> **Task A + Task C** (Auth bug + Product search):
> - Different domains: 0
> - Different services: 0
> → Score: 0 points → ISOLATE"

**What AI taught**: Similarity scoring algorithm (domain +30, models +20, service +20, routes +15, tests +15) student hadn't formulated

---

### AI as Student (Learning Docstring Constraint)
**Transcript excerpt**:
> **Your clarification:**
> "Actually, for our project, documentation and code live in the same files—we use docstrings heavily. Task D (auth documentation) means updating docstrings in auth.py, not writing separate markdown docs."
>
> **AI's revised analysis:**
> "Ah, that changes the similarity score:
>
> **Task A + Task D** (Auth bug + Auth docs/docstrings):
> - Same domain (authentication): +30
> - Same service (auth service): +20
> - Same files (auth.py): +15
> → Score: 65 points → WORK TOGETHER
>
> Since all three tasks touch the same file and domain, working together creates better context."

**What AI learned**: Documentation = docstrings constraint (student taught AI that "documentation" means in-file updates, not separate markdown)

---

### AI as Co-Worker (Session Grouping Convergence)
**Convergence loop** (2 rounds):

1. **Round 1**: AI applies general similarity scoring → recommends 3 separate sessions (Task A, Task B+D, Task C)
2. **Round 2**: Student teaches docstring constraint → AI revises to 2 sessions (Tasks A+B+D together, Task C isolated)

**Final result** (from "What Emerged from Analysis"):
> Through two rounds of iteration, you arrived at a session strategy that:
> - Started with AI's general similarity scoring
> - Incorporated your project-specific constraint (docstrings, not separate docs)
> - Refined to group all authentication work together

**What neither had alone**: Session structure preventing pollution (product search isolated) while leveraging shared context (auth tasks together)

---

## Pattern Consistency Across All Three Lessons

### Successful Natural Narrative Elements

1. **Section headings describe actions, not roles**:
   - ✅ "Exploring Loading Options" (not "AI as Teacher")
   - ✅ "Discovering Project Constraints" (not "AI as Student")
   - ✅ "Refining Through Iteration" (not "AI as Co-Worker")

2. **Dialogue shown without pedagogical labels**:
   - ✅ "**Your request:**" / "**AI's recommendation:**" (natural exchange)
   - ❌ NOT USED: "This is AI as Teacher. AI taught you..."

3. **Convergence numbered explicitly**:
   - ✅ "Round 1" → "Round 2" → "Round 3" (iteration visible)
   - ✅ "Through three rounds of iteration, you arrived at..."

4. **Outcomes described without attribution**:
   - ✅ "**What emerged from collaboration:**" (outcome focus)
   - ❌ NOT USED: "**What you learned:**" / "**What AI learned:**" (framework exposition)

---

## Constitutional Compliance Summary

| Requirement | Lesson 3 | Lesson 4 | Lesson 5 |
|-------------|----------|----------|----------|
| **Three Roles Demonstrated** | ✅ | ✅ | ✅ |
| **Natural Narrative (No Labels)** | ✅ | ✅ | ✅ |
| **Complete Convergence Loops** | ✅ (3 rounds) | ✅ (2 rounds) | ✅ (2 rounds) |
| **Bidirectional Learning Visible** | ✅ | ✅ | ✅ |
| **Students EXPERIENCE (Not Told)** | ✅ | ✅ | ✅ |

---

## Why This Implementation Works

1. **Students see collaboration happen**: Transcripts show AI suggesting, student teaching, iteration converging
2. **No scaffolding exposure**: Framework architecture hidden behind natural dialogue
3. **Outcome focus**: "What emerged" emphasizes result, not pedagogical mechanism
4. **Numbered rounds**: Iteration visible without meta-commentary
5. **Action-oriented headings**: "Exploring", "Discovering", "Refining" describe activities, not instructional design

This is the pattern to replicate for ALL Layer 2 lessons going forward.

---

**Validation Date**: 2025-11-18
**Validated By**: content-implementer v1.0.0

