### Core Concept
Implementation is orchestrated task execution with human checkpoint review at phase boundaries, proving that specification-driven development produces reliable outputs you can validate.

### Key Mental Models
- **Control through visibility**: Checkpoints show intermediate outputs before next phase starts (you see setup works before attempting login, prevents cascading failures)
- **Validation gates**: Every output is testable (ffprobe verifies codec/duration, playback confirms content matches spec)
- **Iteration on failure**: If Phase 3 times out, you debug ONE task (Task 3.3) not the whole workflow (checkpoint isolation enables surgical recovery)
- **PHR capture**: Implementation decisions recorded automatically (history of why you chose this architecture becomes future onboarding document)

### Critical Patterns
- **Four-phase workflow**: Setup (environment ready) → Session (authentication working) → Generation (AI produces artifact) → Validation (proof artifact meets spec)
- **ffprobe for proof**: Technical validation isn't "video looks ok," it's measurable properties (codec=h264, duration=48s, file size=3.2MB)
- **Checkpoint review checklist**: "Does this match the plan?" + "Are there issues I should fix?" + "Is this ready for next phase?"
- **Recovery paths**: If failure occurs, you know exactly which task failed and which ones DON'T need re-running

### AI Collaboration Keys
- AI executes tasks and shows progress (not hidden background work)
- You approve before each phase boundary (human judgment gates automation)
- You can pause and correct the situation (change prompt, retry, escalate)
- AI documents what happened (PHRs capture decisions for future learning)

### Common Mistakes
- **Skipping checkpoint review**: "Phase 1 complete, next phase!" without reading output → hidden failures surface later
- **Accepting video without validation**: "Generated! Done!" but video is 15 seconds (truncated) or wrong codec
- **Not iterating on failure**: "Timeout = failed project" instead of "Network issue, let me retry Task 3.3"

### Connections
- **Builds on**: Tasks (Lesson 7) define work units; Implementation executes them systematically
- **Leads to**: Reusable skills (Lesson 9) encode the patterns discovered during implementation for future projects
- **Critical for**: Proof that SDD-RI produces reliable, measurable outcomes (not just theory, but executable reality)
