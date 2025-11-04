---
title: "Build Constitution - The Project RuleBook"
chapter: 31
lesson: 2
duration_minutes: 120

skills:
- name: "Constitution Writing"
  proficiency_level: "A2"
  category: "Technical"
  bloom_level: "Understand, Apply"
  digcomp_area: "Content Creation, Communication"
  measurable_at_this_level: "Student writes complete spec with all 6 components; components are related and consistent"
- name: "Requirements Clarity"
  proficiency_level: "A2"
  category: "Soft"
  bloom_level: "Apply, Analyze"
  digcomp_area: "Communication & Problem-Solving"
  measurable_at_this_level: "Student identifies ambiguous requirements and clarifies them; explains relationships between requirements"
- name: "Acceptance Criteria Mastery"
  proficiency_level: "A2"
  category: "Technical"
  bloom_level: "Apply"
  digcomp_area: "Content Creation"
  measurable_at_this_level: "Student writes detailed acceptance criteria using SMART framework; criteria are testable and complete"

learning_objectives:
- objective: "Write complete Constitution with all required components and understand the relationship between components"
  proficiency_level: "A2"
  bloom_level: "Apply"
  assessment_method: "Given a feature description, student writes complete specification.md with all sections filled; each section is clear and internally consistent"

cognitive_load:
  new_concepts: 5
  assessment: "5 new concepts (Constitution Components, Component Relationships, Template as Tool, Ready for Planning, Revision Cycle) within A2 limit of 7 ✓"

differentiation:
  extension_for_advanced: "Write specifications for multi-service architectures; include non-functional requirements for scale, security, compliance"
  remedial_for_struggling: "Provided outline with fill-in-the-blank sections; example specification with each section annotated"
---

# Build Constitution - The Project RuleBook

The constitution is your project's rule book. It sets up your coding standards and technical choices that apply to ALL features you'll build. You only do this **once** at the start of your project.

**What you'll accomplish:**

- Set up project-wide coding rules and standards
- Choose your technical tools and preferences
- Establish principles that apply to ALL features (not just one app)
- Create the foundation for learning Spec-Driven Development

## Prerequisites

- Have SpecKit installed in your project and proejct setup
- Be ready to work with your chosen AI agent

## Step-by-Step Instructions

### Step 1: Navigate to Your Project and understand structure

**Your Prompt:**

```
show me the current project structure.
```

**Agent Does:**

- Shows the created directory structure
- Confirms SpecKit installation and setup

### Step 2: Run Constitution Command

**Your Prompt:**

```
/sp.constitution

Project principles and standards:
- Write tests first (TDD approach)
- Use Python 3.12+ with type hints everywhere
- Keep code clean and easy to read
- Document important decisions with ADRs
- Follow essential OOP principles: SOLID, DRY, KISS

Technical stack:
- Python 3.12+ with UV package manager
- pytest for testing
- Keep all project files in git

Quality requirements:
- All tests must pass
- At least 80% code coverage
- Use dataclasses for data structures
```

**Agent Does:**

- Creates comprehensive constitution file
- Sets up project standards and OOP principles
- Defines coding principles and best practices
- Establishes technical preferences and quality gates

### Step 3: Review Generated Constitution and Commit to GitHub

**Your Prompt:**

```
Show me the generated constitution file and explain what it contains.
```

**Agent Does:**

- Displays the constitution content
- Explains each section:
  - **Project Principles** - Core development philosophy and OOP principles
  - **Technical Standards** - Code quality, structure, and best practices
  - **Testing Requirements** - TDD and coverage standards
  - **Architecture Decisions** - When to create ADRs and design patterns
  - **Quality Gates** - CI/CD requirements and code quality checks

**Your Prompt**

Use the agent to commit and open a PR for the constitution: 

```
/sp.git.commit_pr Commit and push the constitution along with current work.
```

**Agent Does:**

- Create a conventional commit for the constitution and push to a new feature branch
- Create a draft PR (or share the compare URL if `gh` auth is missing)

### Step 4: Customize Constitution (Optional)

**Your Prompt:**

```
Review the constitution and suggest any modifications based on my preferences:
- I want comprehensive type hints everywhere
- I want detailed docstrings for all public functions
- I want error handling to be explicit, not silent
- I want to use dataclasses for data structures
- I want to follow SOLID principles
- Focus on learning and teaching value
```

**Agent Does:**

- Analyzes current constitution
- Suggests specific modifications
- Updates constitution file if requested
- Explains the impact of each change


## What's Next

The constitution is now established and will guide all future development. The next step is to begin the first spec loop with the Basic Operations feature.

**Key Points:**

- Constitution is a living document - update as needed
- All future spec loops will reference this constitution
- This establishes HOW to build (principles), not WHAT to build (specific apps)
- These rules apply to ALL features you'll create in this project

---
