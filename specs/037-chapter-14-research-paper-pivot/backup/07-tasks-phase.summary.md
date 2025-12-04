### Core Concept
Tasks are 15-30 minute atomic work units with single acceptance criteria that keep you in control through checkpoints (Agent→Review→Approve→Commit→Continue).

### Key Mental Models
- **Atomic decomposition**: Plan complexity breaks into independent, verifiable units with no hidden dependencies
- **Checkpoint pattern**: Human review points prevent downstream failure propagation (catch issues early, not after 5 failed downstream tasks)
- **Specification lineage**: Every task traces back to intent (spec → plan → task, not code-first improvisation)
- **Session persistence**: Browser automation requires saved login state (not re-login per task, reuse across multiple generations)

### Critical Patterns
- **Task sizing**: 15-30 minutes is sweet spot (under 10 min = too many checkpoints overhead, over 45 min = hard to review/fix)
- **Single criterion**: Each task has ONE testable acceptance criterion (not "setup stuff," but "Session file exists at `.session-storage/gemini-session.json` AND contains valid cookies")
- **Dependency graph**: Visual mapping of which tasks depend on which (sequential phases: Setup → Session → Generation → Validation)
- **Video prompt engineering**: Scene-by-scene structure with explicit timing (Scene 1: 0-10s, Scene 2: 10-20s) + style keywords + constraints

### AI Collaboration Keys
- AI executes tasks within defined boundaries (not autonomous continuation)
- You review output at each checkpoint (AI can't skip your approval)
- AI can retry failed tasks with your permission
- Checkpoints create feedback loop: Agent proposes → You evaluate → Agent adjusts

### Common Mistakes
- **Tasks too large**: "Complete entire video generation (2+ hours)" hides failures until late stages
- **Combining manual and automated**: Mixing login + wait + download + validate in one task makes failure recovery unclear
- **Vague acceptance criteria**: "Setup is complete" leaves success undefined vs. "Session file exists AND file size > 1KB AND contains valid tokens"

### Connections
- **Builds on**: Specification (Lesson 4) and Plan (Lesson 6) define WHAT and HOW; tasks define WHEN and WHO-CHECKS
- **Leads to**: Implement phase (Lesson 8) executes these 12 tasks under checkpoint control
- **Critical for**: Maintaining human control in automation (checkpoints prevent runaway autonomous execution)
