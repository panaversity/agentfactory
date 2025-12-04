### Core Concept
Installing Python correctly is foundational—it's the bridge between understanding Python and executing Python code. Proper installation involves not just downloading but verifying the setup through terminal commands and understanding platform-specific differences (PATH configuration, python vs python3 naming).

### Key Mental Models
- **Installation as System Configuration**: Python installation isn't just copying files; it's configuring your system's PATH so terminal recognizes Python commands from any directory
- **Platform Differences are Real**: Windows PATH handling, Mac's Python 2/3 naming conflict, Linux package managers—each OS requires different approaches but same verification principle
- **Verification as Validation**: Running `python --version` and executing test programs aren't optional steps; they're how you confirm configuration actually works before attempting real projects

### Critical Patterns
- **Platform-Specific Root Causes**: "python: command not found" on Windows = PATH not set; same error on Mac/Linux = trying `python` instead of `python3` or broken installation
- **Version Matters**: Python 3.14+ is chosen intentionally; older versions lack features used throughout Part 4; checking version prevents silent compatibility failures
- **Three-Step Verification**: Check version → Run test command → Interpret error messages; this pattern applies to all system configuration problems

### AI Collaboration Keys
- Installation errors contain cryptic system messages (PATH variables, permissions, OS configuration); AI excels at translating these into actionable fixes
- Troubleshooting becomes learning: providing complete error context (OS version, exact message, what you tried) trains you in professional debugging practices
- When stuck, system errors are research opportunities; asking AI "What does 'command not found' mean?" teaches you how real developers debug configuration problems

### Common Mistakes
- Installing Python 2 or outdated versions instead of 3.14+ (creates silent incompatibilities when running later code)
- Skipping the PATH checkbox on Windows or not verifying installation before moving to coding lessons (wastes hours on cryptic errors)
- Assuming "installed" means working; installation succeeds but configuration fails (PATH not set, wrong terminal used, Python not in system paths)

### Connections
- **Builds on**: Understanding file systems and terminal basics (from earlier chapters)
- **Leads to**: Running Python programs, creating .py files, debugging installation-related errors, professional system configuration practices
