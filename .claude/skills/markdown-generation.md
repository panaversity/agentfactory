# markdown-generation

**Description**: Systematic framework for creating production-quality markdown documentation that serves readers efficiently, applicable to any document type (README, API docs, tutorials, specifications).

**When to use this skill**: Apply when generating technical documentation to ensure reader-focused, minimal essential content with effective structure.

---

## Persona

"Think like a technical writer creating production documentation that serves readers efficiently, not comprehensive brain-dumps"

**Cognitive stance**: Documentation generation is NOT dumping all information. It's strategic content design: (1) Who reads this? (2) What problem do they need solved? (3) What's the minimum to solve it? (4) How can structure guide them? Prioritize reader goals over writer knowledge.

---

## Questions

Apply these 5 questions sequentially when generating any markdown documentation:

### 1. Who is the reader (audience proficiency)?

**Purpose**: Understand reader's background to calibrate content level.

**Ask**:
- What's their technical proficiency? (Beginner, intermediate, expert)
- What do they already know? (Prerequisites)
- What terms/concepts need explanation? (What can be assumed?)
- What's their role? (Developer, DevOps, Manager, User)

**Examples**:
- **Beginner developer**: Explain concepts, provide step-by-step instructions, assume little
- **Senior engineer**: Skip basics, focus on architecture/trade-offs, assume technical fluency
- **Product manager**: Focus on capabilities/benefits, minimize technical jargon

**Bad audience definition**: "Anyone"
**Good audience definition**: "Backend engineers with 2+ years Python experience evaluating framework adoption"

---

### 2. What problem does this document solve for them?

**Purpose**: Define document's job-to-be-done.

**Ask**:
- What task is the reader trying to accomplish?
- What question are they trying to answer?
- What decision are they trying to make?
- What would happen if this document didn't exist?

**Example document purposes**:
- **README**: Enable reader to install, run, and understand project in <10 minutes
- **API docs**: Enable reader to integrate API successfully without trial-and-error
- **Tutorial**: Enable reader to learn concept through guided practice
- **Architecture spec**: Enable reader to understand system design and make informed decisions

**Focus**: Outcome for reader (what they can DO after reading), not input from writer (what I want to explain)

---

### 3. What's the minimum information needed?

**Purpose**: Eliminate bloat, focus on essential content.

**Ask**:
- What information is REQUIRED to accomplish reader's goal?
- What information is NICE-TO-HAVE but not essential?
- What can be linked/referenced instead of explained inline?
- What sections serve learning objectives vs just "completeness"?

**Example (README)**:
- **Essential**: Installation, Quick Start, Core Commands
- **Nice-to-have**: Architecture details, contribution guidelines
- **Can be separated**: Internal design (move to docs/architecture.md)

**Principle**: If removing a section doesn't hurt reader's ability to accomplish their goal → REMOVE it or move to separate doc.

---

### 4. How can structure guide comprehension?

**Purpose**: Design information architecture that leads reader logically.

**Ask**:
- What should reader learn first? (Foundation before advanced)
- What's the logical sequence? (Concept → Example → Practice)
- Where do readers scan vs read deeply? (Use headings, bullets, code blocks)
- How can visual hierarchy help? (H1 = major sections, H2 = subsections, bullets = lists)

**Common structures**:

**Tutorial structure**:
```
1. Concept explanation (WHY)
2. Step-by-step walkthrough (HOW)
3. Practice exercise (DO)
4. Validation (CHECK)
```

**API documentation structure**:
```
1. Authentication (prerequisite)
2. Endpoints table (overview)
3. Detailed examples (reference)
4. Error codes (troubleshooting)
```

**README structure**:
```
1. Quick Start (get running in 3 steps)
2. Installation (detailed setup)
3. Usage (common commands/examples)
4. Configuration (environment variables)
5. Testing (how to run tests)
```

**Principle**: Structure should match reader's mental model of the task.

---

### 5. What examples demonstrate concepts effectively?

**Purpose**: Show, don't just tell. Concrete examples beat abstract explanations.

**Ask**:
- What code examples clarify concepts?
- What real-world scenarios illustrate use cases?
- What expected outputs help validate understanding?
- What edge cases or gotchas should be demonstrated?

**Good example characteristics**:
- **Concrete**: Real inputs/outputs, not placeholders ("your-api-key")
- **Minimal**: Simplest case that demonstrates concept
- **Runnable**: Reader can copy-paste and execute
- **Annotated**: Comments explain WHY, not just WHAT

**Example (API endpoint documentation)**:
```markdown
### POST /users

**Request**:
```json
{
  "email": "user@example.com",
  "password": "SecurePass123!",
  "role": "developer"
}
```

**Success Response (201 Created)**:
```json
{
  "id": "user_abc123",
  "email": "user@example.com",
  "role": "developer",
  "created_at": "2025-01-18T10:30:00Z"
}
```

**Error Response (400 Bad Request)**:
```json
{
  "error": "Invalid email format",
  "field": "email"
}
```
```

---

## Principles

These are decision frameworks, not rigid rules. Apply judgment to context.

### Principle 1: Reader-First (Optimize for Comprehension)

**Framework**: "Every section must serve reader's goal. If it doesn't help reader accomplish their task, remove or move it."

**What this means**:
- Start with reader's goal, not writer's knowledge
- Prioritize what reader NEEDS over what writer WANTS to explain
- Test: Would removing this section hurt reader's ability to succeed? If no → remove

**Example**:
- ❌ Bad: Include detailed internal architecture in README (readers want quick start, not design details)
- ✅ Good: Link to docs/architecture.md for readers who need that context

---

### Principle 2: Minimal Essential (Every Section Serves Learning Objective)

**Framework**: "Include minimum information needed for reader to succeed. Comprehensiveness is NOT quality."

**What this means**:
- Define learning objectives first (what should reader be able to DO?)
- Map every section to an objective
- If section doesn't map → cut it

**Example (Tutorial objective: "Run API locally")**:
- ✅ Include: Installation, configuration, running commands
- ❌ Exclude: Internal design philosophy (doesn't help run locally)

---

### Principle 3: Progressive Disclosure (Simple First, Complex After Foundation)

**Framework**: "Layer information: essential basics → common scenarios → advanced edge cases. Don't front-load complexity."

**What this means**:
- Start with simplest use case (1 command to get started)
- Add complexity progressively (flags, options, configuration)
- Advanced topics last (or separate document)

**Example (CLI tool README)**:
```markdown
## Quick Start (Simple)
```bash
tool run
```

## Common Commands (Intermediate)
```bash
tool run --config dev    # Development mode
tool run --verbose       # Debug output
```

## Advanced Configuration (Complex)
See docs/advanced.md for:
- Custom plugins
- Scripting integration
- Performance tuning
```

---

### Principle 4: Falsifiable Quality (Can Reader Complete Task After Reading?)

**Framework**: "Documentation quality is measurable: Can reader accomplish stated goal after reading? If not, doc failed."

**What this means**:
- Define success criteria (reader can install and run project)
- Test with real user (can they follow instructions?)
- Validate: Did reader succeed? Where did they get stuck?

**Validation checklist**:
- ✅ Can reader complete task without external help?
- ✅ Are all commands copy-pasteable and working?
- ✅ Are error messages anticipated and explained?
- ✅ Are expected outputs shown for validation?

---

### Principle 5: Structural Clarity (Headings, Bullets, Code Blocks)

**Framework**: "Use markdown structure to guide scanning and comprehension. Walls of text fail."

**What this means**:

**Headings**: Break content into scannable sections
```markdown
# Main Topic (H1)
## Subtopic (H2)
### Detail (H3)
```

**Bullets**: Lists of items, steps, or options
```markdown
- Item 1
- Item 2
- Item 3
```

**Code blocks**: Executable examples with syntax highlighting
```markdown
```python
def example():
    return "formatted code"
```
```

**Tables**: Structured data (API endpoints, configuration)
```markdown
| Method | Path | Description |
|--------|------|-------------|
| GET    | /users | List users |
```

**Avoid**: Long paragraphs without breaks, missing code formatting, unclear structure

---

## Reusability

**This skill transfers across**:
- **README documentation** (project setup, usage)
- **API documentation** (endpoints, authentication, examples)
- **Tutorial content** (step-by-step learning)
- **Architecture specifications** (system design, decision records)
- **Contributing guides** (onboarding, development workflow)
- **Any markdown documentation**

**Evidence of reusability** (tested in Chapter 10, Lesson 7):
- ✅ Generated README for REST API project
- ✅ Generated API documentation for endpoints
- ✅ Generated tutorial for git workflow
- Same 5 questions, same process, different document types

**Future application** (Part 4+):
- Python project READMEs (Part 4)
- Module documentation (Part 4)
- Agent specification docs (Part 6-7)
- Protocol is domain-agnostic (reasoning pattern, not content-specific)

---

## Usage Example

**Scenario**: Generate README for Python CLI tool

**Apply markdown-generation**:

**Question 1: Who is the reader?**
- Backend engineers familiar with Python, new to this CLI tool
- Proficiency: Intermediate (know Python, don't know tool internals)

**Question 2: What problem does this solve?**
- Reader wants to install and run tool locally for development
- Goal: Get tool running in <5 minutes

**Question 3: What's minimum info needed?**
- Essential: Installation, Quick Start, Core Commands
- Nice-to-have: Configuration options, advanced usage
- Separate doc: Architecture, contributing guidelines

**Question 4: How to structure?**
```markdown
# Quick Start (immediate value)
# Installation (setup)
# Common Commands (usage)
# Configuration (optional)
# Troubleshooting (FAQs)
```

**Question 5: What examples?**
- Quick Start: One command to run tool
- Commands: 3-5 common commands with outputs
- Configuration: Example .env file

**Generated README**:

```markdown
# MyTool - Python CLI for Data Processing

## Quick Start

```bash
# Install
pip install mytool

# Run on sample data
mytool process data.csv

# Output: processed_data.csv (cleaned, validated)
```

## Installation

**Requirements**: Python 3.8+

```bash
# From PyPI
pip install mytool

# From source
git clone https://github.com/user/mytool.git
cd mytool
pip install -e .
```

## Common Commands

### Process CSV
```bash
mytool process input.csv --output result.csv
```

### Validate Data
```bash
mytool validate data.csv
# Output: ✓ 1000 rows valid, 5 errors found
```

### Generate Report
```bash
mytool report data.csv --format html
```

## Configuration

Create `.mytool.conf`:
```ini
[processing]
chunk_size = 1000
parallel = true

[validation]
strict_mode = false
```

## Troubleshooting

**Error: "Module not found"**
→ Install dependencies: `pip install -r requirements.txt`

**Error: "File not found"**
→ Check file path: `mytool process ./path/to/data.csv`
```

**Validation**:
- ✅ Reader can install in <2 minutes
- ✅ Reader can run tool successfully
- ✅ Common errors anticipated
- ✅ Examples are concrete and runnable

---

---

## Transfer Validation

**This skill claims to transfer to Python development (Part 4, Chapters 12-29).**

**Validation checkpoint**: When Part 4 is implemented, validate that the Persona + Questions + Principles pattern works for Python documentation generation (READMEs, docstrings, module docs) without modification:

**Test cases**:
1. **Python module README**: Does Question 1 (What problem solved?) generate clear problem statements for Python packages?
2. **API documentation**: Does Question 2 (What sections needed?) identify appropriate sections for Python library docs (Installation, Quickstart, API Reference, Examples)?
3. **Docstring generation**: Does Question 3 (What concrete examples?) produce runnable Python code examples with expected outputs?
4. **Type hint documentation**: Does the framework handle documenting type hints, Pydantic models, and TypedDict effectively?
5. **Integration guides**: Does Question 5 (What quick win?) create effective Python integration examples (e.g., "Install and use in 5 minutes")?

**Expected outcome**: Framework works without Python-specific modifications. Principles (code-first, runnable examples, progressive disclosure) apply to Python docstring and module documentation as well as general markdown generation.

**If Python-specific adjustments needed**: Document as extensions (e.g., "Include type hints in code examples", "Add docstring format examples"), not replacements of core pattern.

**Validation date**: [To be completed when Part 4 Python chapters are implemented]

---

## Version

**Version**: 1.0.0
**Created**: 2025-01-18 (Chapter 10, Lesson 7)
**Domain**: Cross-domain markdown documentation generation
**Pattern**: Persona + Questions + Principles (reasoning-activated)
**Constitution**: v6.0.0 Compliance
