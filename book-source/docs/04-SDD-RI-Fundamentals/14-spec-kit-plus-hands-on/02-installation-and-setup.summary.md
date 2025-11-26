# Installation and Setup

This lesson teaches students how to set up a Spec-Kit Plus project by creating a folder structure that organizes decision-making frameworks, specifications, and AI collaboration patterns. Students create four key directories—.specify/memory/, .claude/skills/, specs/research-paper/, and the paper.md deliverable—then establish a minimal constitution file to guide their entire project. The lesson emphasizes that folder organization is not bureaucratic overhead but a system for helping AI companions understand project values and reusing intelligent patterns across projects.

### Mental Models

- **Artifact Organization by Purpose**: Different types of thinking artifacts belong in different directories (.specify/ for principles, specs/ for planning documents, .claude/ for reusable patterns), which signals to both humans and AI what kind of work each folder contains
- **Constitution as Decision Framework**: The constitution file captures project values upfront, then acts as a reference point when making tradeoffs later (clarity vs. formality, rigor vs. brevity)
- **Folder Structure as AI Communication**: The physical organization of folders teaches the AI companion where to find decision-making context, so it doesn't have to guess what matters in this project
- **Specification Layers**: The three-file structure (spec.md → plan.md → tasks.md) represents increasing levels of implementation detail, moving from intent → architecture → actionable steps

### Key Patterns

- Hidden configuration folders (prefixed with dots) contain meta-work about the project, while regular folders contain deliverables
- Constitution is written once at project start and referenced throughout, establishing stable decision criteria before implementation begins
- Directory structure mirrors the Spec-Kit Plus workflow (specify → plan → task), making the methodology visible in how files are organized
- Skills directory starts empty but grows as reusable patterns are discovered during the project
- Mental model table (artifact type → storage location) makes it clear where new documents belong as students work

### Common Mistakes

- Treating folder structure as busywork rather than as a communication system with the AI companion
- Skipping or delaying the constitution, making decisions ad-hoc instead of guided by established values
- Creating folders but not understanding what they represent (assuming .specify/ and .claude/ are just naming conventions rather than functional divisions)
- Mixing artifact types (putting planning documents in wrong folders, making reusability harder)
- Not verifying the AI companion can actually access the project structure before proceeding (troubleshooting access issues later)

### Progression Context

- **Builds on**: Lesson 1 (conceptual understanding of Spec-Kit Plus as framework for capturing working artifacts AND reusable intelligence)
- **Leads to**: Lesson 3 (Writing a constitution using `/sp.constitution` command to expand the minimal version created here), followed by Lesson 4 (Writing formal specification using `/sp.specify`)
