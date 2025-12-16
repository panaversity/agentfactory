### Core Concept
Antigravity treats AI not as a tool you control, but as an intelligent agent that decides when to involve you—through configurable autonomy modes and structured approval gates that catch mistakes before any code is written.

### Key Mental Models
- **Autonomy Spectrum**: Three modes—Ask Always (maximum control), Ask Sometimes (agent handles routine, asks for architecture decisions), Full Auto (agent decides everything, you review after)
- **Artifact-Driven Workflow**: Request → Task List → Implementation Plan → Approval → Execution → Walkthrough—each stage produces a reviewable artifact
- **Approval Gates**: Reviewing the plan BEFORE code is written catches errors when they're cheap to fix; reviewing Walkthroughs AFTER catches what wasn't tested

### Critical Patterns
- **Task List**: What needs to be created, why, and how to verify it's done
- **Implementation Plan**: Research findings, architecture decisions with rationale, component structure, verification strategy
- **Walkthrough**: Files created, test results, screenshots proving features work, code quality review
- **Parallel Execution**: Foreground task (you work on) + background task (agent researches simultaneously) reduces total project time
- **Browser Integration**: Agent controls Chrome (blue border = agent acting), runs automated tests, captures screenshots as evidence

### Common Mistakes
- Using Full Auto mode on foundational systems (auth, payments)—these need Ask Always for oversight
- Approving plans without checking if research seems thorough or decisions match project needs
- Skipping the verification strategy review—this determines whether you can trust the Walkthrough later
- Not decomposing projects into parallel tasks—sequential work wastes time waiting for agents

### Connections
- **Builds on**: Cursor autonomous agents (Lesson 4-5), diff-based review patterns, specification-first thinking
- **Leads to**: Comparative Capstone (Lesson 8) where you choose an IDE and build a real project with documented workflow
