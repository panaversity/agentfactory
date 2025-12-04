# Installation and Setup

This lesson teaches students how to install Spec-Kit Plus framework and initialize their first project. Students install the framework via `pip install specifyplus`, run `specifyplus init` to create a properly structured project, and verify slash commands work in their AI tool (Claude Code or Gemini CLI). The lesson emphasizes that Spec-Kit Plus is a framework separate from the AI tool—you need BOTH.

### Mental Models

- **Framework vs Tool Distinction**: Spec-Kit Plus is a framework (installed via pip) that provides structure, templates, and slash commands. Claude Code/Gemini CLI is the AI tool that executes those commands. They work together but are installed separately.

- **Artifact Organization by Purpose**: Different types of thinking artifacts belong in different directories (`.specify/` for constitution and templates, `.claude/commands/` for slash commands, `specs/` for specifications). This organization helps both humans and AI understand where each artifact type belongs.

- **Constitution as Decision Framework**: The constitution file in `.specify/memory/` captures project values upfront, then acts as a reference point when making tradeoffs throughout the project.

- **Slash Commands as Workflow Steps**: Each `/sp.` command represents a phase in the SDD-RI workflow—from constitution to specification to planning to implementation. The commands are your interface to the methodology.

### Key Patterns

- **Python 3.12+ Prerequisite**: Spec-Kit Plus requires Python 3.12 or higher. Check version before installation with `python --version`.

- **pip install specifyplus**: The framework is installed like any Python package. This gives you the `specifyplus` command-line tool.

- **specifyplus init <project-name>**: Creates complete project structure with templates, slash commands, scripts, and configuration. Never create folders manually.

- **Directory Structure**: `.claude/commands/` holds slash commands, `.specify/memory/` holds constitution, `.specify/templates/` holds templates for specs/plans/tasks. `specs/` and `history/` are created when you start features.

- **Command Verification**: Type `/sp.` in your AI tool to see available commands. If they appear, installation is complete.

### Common Mistakes

- **Confusing framework with AI tool**: Installing Claude Code doesn't give you Spec-Kit Plus. You need `pip install specifyplus` separately.

- **Creating folders manually**: Running `mkdir` commands instead of `specifyplus init` means missing critical infrastructure (templates, scripts, configuration).

- **Wrong Python version**: Spec-Kit Plus requires Python 3.12+. Older versions will fail. Check version first.

- **Skipping verification**: Not testing `/sp.` commands means you might discover installation problems later during actual work.

### Progression Context

- **Builds on**: Lesson 1 (conceptual understanding of Spec-Kit Plus as SDD-RI framework, Horizontal/Vertical Intelligence)

- **Leads to**: Lesson 3 (Constitution Phase) where students run `/sp.constitution` to create their project's decision framework. The constitution file created by `specifyplus init` is a starter version—Lesson 3 expands it.
