### Core Concept
UV is a unified, Rust-based Python package manager that solves decades of fragmentation (pip, poetry, conda, pipenv) by providing one tool for virtual environments, dependency management, and reproducibility—10–100x faster than alternatives.

### Key Mental Models
- **Package Manager Abstraction**: Handles four functions automatically—installs libraries, manages versions, isolates environments, ensures reproducibility—so developers express intent rather than execute commands
- **Unified Tooling Philosophy**: One tool replaces fragmented ecosystem (pip + venv + virtualenv + poetry), reducing cognitive load and eliminating "which tool?" decisions
- **Speed as Feedback**: Rust implementation enables fast operations, keeping developers in flow state rather than waiting for installations (30 seconds vs. 5 minutes)
- **Professional Default**: UV represents modern Python standards (2025+); learning it first means adopting industry patterns from day one

### Critical Patterns
- **Tool Selection Framework**: Scenario-based decisions (new project → UV, legacy project → stick with existing tool, data science → conditional conda/UV)
- **Industry Adoption Trajectory**: Created by Astral (Ruff makers), open source, solving real problems, gaining adoption—signals future direction

### AI Collaboration Keys
- AI helps evaluate tool tradeoffs and decide which manager fits specific project requirements without memorizing comparison matrices
- AI explains why Python accumulated fragmentation, deepening understanding beyond "UV is faster"

### Common Mistakes
- Assuming all projects need same tool (legacy projects have valid reasons for existing tools)
- Memorizing version constraint syntax (`^1.0.0` vs `>=1.0.0`) instead of asking AI for generation
- Treating tool choice as permanent (switching mid-project creates git conflicts; new projects can choose freely)

### Connections
- **Builds on**: Basic understanding of dependencies (what libraries are, why reuse matters)
- **Leads to**: Installing UV, creating first project, managing dependencies, team workflows, code quality tools (Ruff, Pyright)
