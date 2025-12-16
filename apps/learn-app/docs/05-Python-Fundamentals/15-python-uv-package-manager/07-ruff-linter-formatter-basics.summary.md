### Core Concept
Ruff consolidates three separate Python tools (Black, isort, Flake8) into one, providing formatting (consistent style) and linting (catching bugs) 10–100x faster, allowing developers to express code quality intent without memorizing multiple tools or syntax rules.

### Key Mental Models
- **Unified Quality Tool**: Ruff replaces tool proliferation (before: choose Black + isort + Flake8, decide order, manage conflicts; now: one tool, two commands, done)
- **Formatting vs. Linting Distinction**: Formatting fixes appearance (spaces, quotes, indentation)—cosmetic only; linting finds actual errors (unused imports, undefined variables)—semantic
- **Error Code Taxonomy**: F401 (unused import), E501 (line too long), E225 (missing whitespace)—codes identify problem types, delegated to AI for interpretation rather than memorization
- **Automatic Fixing Philosophy**: `[*]` markers indicate auto-fixable errors; `ruff check --fix` corrects them without manual edits, reducing developer friction

### Critical Patterns
- **Dual-Command Pattern**: `ruff format .` (cosmetic), then `ruff check .` (semantic), applied before commits—prevents style disagreements and catches bugs
- **PEP 8 Alignment**: Defaults follow Python style standards, suitable for learning and production

### AI Collaboration Keys
- AI interprets error codes and suggests fixes without requiring memorization of Ruff rule catalogs
- AI handles configuration generation, allowing students to understand concepts without TOML syntax burden

### Common Mistakes
- Running only format without linting (misses logical errors)
- Assuming all Ruff warnings are errors (E501 line-too-long often overridden for URLs)
- Memorizing error codes instead of asking AI when messages appear

### Connections
- **Builds on**: Project structure, dependency management, understanding code quality
- **Leads to**: Advanced Ruff configuration, type checking (Pyright), production code standards
