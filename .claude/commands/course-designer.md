---
description: Intelligently design courses from book content or raw ideas - generates database-ready Python files with compelling copy
argument-hint: [course-code] [description or "parts X-Y"]
---

# /course-designer

## Purpose
Design compelling, constitution-aligned courses from book content OR raw ideas. Generates database-ready Python files with professionally crafted copy using advanced copywriting techniques. Perfect for both published content and future course planning.

## Contract

**Inputs:**
- `course_code` — Course code (e.g., "AI-400")
- `description` — Either:
  - Book reference: "parts 1-3" or "chapters 5-8"
  - Raw ideas: Multi-line description of topics/concepts
  - Context reference: "@context/folder" for external materials

**Outputs:**
Single Python file at `docs/course_outlines/py_format/{course_code}_{sanitized_name}.py` with:
- Native Python types (lists, dicts - not JSON strings)
- Compelling copywriting (hook → value prop → transformation)
- Database-ready structure
- Helper functions for easy insertion

**Status Output:** `STATUS=WROTE PATH=docs/course_outlines/py_format/{filename}.py`

---

## Instructions

### Phase 1: Intelligence Gathering

#### 1.1 Load Constitution & Copywriting Intelligence

**Read Constitution:**
```
.specify/memory/constitution.md (v3.1.2)
```

Extract:
- ✅ Nine Pillars framework
- ✅ AI Development Spectrum (Assisted → Driven → Native)
- ✅ "Specs Are the New Syntax" philosophy
- ✅ Co-learning partnership principles
- ✅ Target audience tiers (beginner/intermediate/advanced/professional)
- ✅ 10x-99x multiplier mindset

**Activate Copywriting Intelligence:**

Use proven copywriting frameworks:

1. **AIDA Framework** (Attention → Interest → Desire → Action)
   - Attention: Bold paradigm shift statement
   - Interest: Concrete problem → solution
   - Desire: Transformation narrative (before → after)
   - Action: Call to mastery

2. **PAS Framework** (Problem → Agitate → Solve)
   - Problem: Current broken state
   - Agitate: Why it matters NOW
   - Solve: This course as solution

3. **Storytelling Arc**
   - Setup: Old world (line-by-line coding)
   - Conflict: Paradigm shift (AI era demands new skills)
   - Resolution: New world (AI-native mastery)

4. **Transformation Language**
   - From "coder" → to "architect/orchestrator"
   - From "typing syntax" → to "specifying intent"
   - From "debugging code" → to "validating systems"
   - From "tool user" → to "AI co-creator"

5. **Power Words & Phrases**
   - Paradigm shift, revolution, breakthrough
   - Master, architect, orchestrate, build
   - Production-ready, professional-grade
   - Autonomous, intelligent, agentic
   - 10x-99x multiplier, exponential growth

#### 1.2 Discover Content Source

**If book reference (e.g., "parts 1-3"):**
- Use Glob to find chapters in `book-source/docs/`
- Read part-level README files
- Extract chapter titles and learning objectives
- Identify tools, technologies, patterns covered

**If raw ideas (multi-line description):**
- Parse topics/concepts from user description
- Identify tools mentioned (Claude Code, Docker, Kubernetes, DAPR, etc.)
- Determine complexity level from topics
- Cross-reference with book content if available

**If context reference (e.g., "@context/cloud/"):**
- Read files from specified context folder
- Extract key concepts, architectures, patterns
- Integrate with book content where overlap exists

**Content Synthesis:**
- Map topics to Nine Pillars
- Determine AI Development Spectrum stage (Assisted/Driven/Native)
- Identify prerequisites
- Note which content exists vs. planned

#### 1.3 Determine Course Level

Derive course code tier from complexity:
- **AI-100 series** (Beginner): Parts 1-3, foundational AI-native thinking
- **AI-200 series** (Intermediate/Advanced): Parts 4-8, SDD, advanced Python
- **AI-300 series** (Professional): Parts 9-13, production agentic AI
- **AI-400 series** (Infrastructure): Cloud-native to agent-native, Docker/K8s/DAPR

---

### Phase 2: Intelligent Course Design with Compelling Copy

#### 2.1 Generate Course Metadata

**course_code**: User-provided or derived (AI-100/200/300/400)

**course_name**: Synthesize from:
- Primary technology/domain
- Key transformation (e.g., "Cloud Native to Agent Native")
- Constitution alignment

Format: "{Primary Tech}: {Transformation/Focus}"
Examples:
- "AI-Driven Development with Python and Agentic AI"
- "Cloud Native to Agent Native Infrastructure: Docker, Kubernetes, DAPR & Autonomous Computing"
- "Specification-Driven Development: Building Software with AI Partners"

**course_initials**: Extract meaningful acronym
- Example: "CNTAN" for "Cloud Native to Agent Native"
- Example: "AIDD-PAI" for "AI-Driven Development with Python and Agentic AI"

**created_by / updated_by**: Default to "db_admin"

#### 2.2 Craft Course Description (2-3 sentences, hook + value)

**Structure:**
1. **Hook**: Bold claim or paradigm shift statement
2. **Value**: What they'll master (tools + mindset)
3. **Outcome**: Who they become (role transformation)

**Copywriting Techniques:**
- Lead with transformation, not topics
- Use active voice and present tense
- Emphasize AI-native methodology
- Include specific tools/technologies
- Connect to constitution principles

**Example (AI-400):**
```
Transform from cloud-native to agent-native infrastructure. Master Docker, Kubernetes, and DAPR to build autonomous, self-organizing systems where AI agents orchestrate containers, manage workflows, and adapt to production demands. Learn the infrastructure patterns powering the next generation of AI-native applications.
```

#### 2.3 Extract Course Outcomes (5-7 outcomes, Bloom's taxonomy)

**Structure:** Start broad (Understand) → specific skills (Apply/Analyze) → mastery (Create)

**Formula per outcome:**
- Verb (Bloom's taxonomy: Understand/Apply/Analyze/Design/Build/Create)
- Object (what they're learning)
- Context (why it matters / how it connects)

**Required Outcomes:**
1. **AI-Native Mindset** (always first)
   - "Understand AI-native thinking: specs-first, validation-focused, co-learning"
2. **Tool/Tech Mastery** (2-3 outcomes)
   - Specific to course (Claude Code, Docker, Python, etc.)
3. **Pattern Application** (1-2 outcomes)
   - SDD, AIDD, architecture patterns
4. **Production Skills** (1-2 outcomes)
   - Build/deploy/orchestrate real systems
5. **Advanced Capability** (optional, for AI-300/400)
   - Multi-agent, autonomous systems, infrastructure orchestration

**Copywriting:**
- Start with strong action verbs
- Be specific about what they'll build
- Connect to real-world production scenarios
- Avoid generic phrases ("learn about", "understand basics")

**Example Outcome (Good):**
✅ "Build production-quality Docker containers orchestrated by AI agents using DAPR actors for autonomous scaling and self-healing"

**Example Outcome (Bad):**
❌ "Learn about Docker and containers"

#### 2.4 Write Long Description (3-4 paragraphs, compelling narrative)

**Paragraph 1: The Paradigm Shift (Hook + Problem)**

Structure: "The era of X is over. The future isn't about Y; it's about Z."

Copywriting techniques:
- Bold opening statement (paradigm shift)
- Contrast old vs. new world
- Create urgency (why NOW matters)
- Use emotional language (revolution, transformation)

**Template:**
```
The era of [OLD PARADIGM] is over. The future isn't about [OLD SKILL];
it's about [NEW CAPABILITY]. [COURSE FOCUS] represents [TRANSFORMATION TYPE],
the fundamental shift from [OLD ROLE] to [NEW ROLE]. It's a [MOVEMENT/METHODOLOGY]
that moves you from [OLD STATE] to [NEW STATE]—where you'll [NEW CAPABILITY],
not just [OLD CAPABILITY]. This is how [DOMAIN] will be [DONE] for the next decade.
```

**Example (AI-400):**
```
The era of static infrastructure is over. The future isn't about deploying
containers; it's about orchestrating autonomous systems. Cloud Native to Agent
Native represents the infrastructure revolution, the fundamental shift from
"ops engineer" to "infrastructure architect." It's a methodology that moves you
from "deploying services" to "designing self-organizing systems"—where you'll
architect infrastructure that adapts, heals, and optimizes itself through AI agents.
```

**Paragraph 2: The Methodology (Solution + Framework)**

Structure: Introduce framework, explain approach, emphasize uniqueness

Copywriting techniques:
- Name your methodology (AIDD, SDD, Agent-Native)
- Connect to constitution principles
- Explain the learning approach (not just content)
- Highlight AI partnership (co-learning)

**Template:**
```
This course introduces [METHODOLOGY], the [DESCRIPTION]. You'll learn [FRAMEWORK]
following [APPROACH]. Using [CONSTITUTION PRINCIPLE], you'll master [SKILL]
through [METHOD]. This isn't about [OLD WAY]; it's about [NEW WAY].
```

**Paragraph 3: The Journey (Learning Path + Experience)**

Structure: Walk through modules, emphasize progression, show breadth

Copywriting techniques:
- Use journey language ("start with", "then", "finally")
- Name 3-5 key stops (modules/topics)
- Show progression (foundations → advanced)
- Emphasize hands-on building
- Mention specific tools/technologies

**Template:**
```
You'll start with [MODULE 1 FOCUS], building [FOUNDATION]. Then [MODULE 2 FOCUS],
where you'll [SPECIFIC SKILL]. You'll [MODULE 3 FOCUS] to [OUTCOME]. Finally,
you'll [MODULE 4-5 FOCUS], [ADVANCED CAPABILITY]. Throughout, you'll [CO-LEARNING
ELEMENT] with [AI TOOL] to [COLLABORATION PATTERN].
```

**Paragraph 4: The Transformation (Outcome + Call to Action)**

Structure: "This isn't just X; it's Y." + who they become + economic opportunity

Copywriting techniques:
- Use contrast ("not just tools, but mindset")
- Paint the future state
- Connect to 10x-99x multiplier
- End with empowerment

**Template:**
```
This isn't just [SURFACE LEVEL]; it's [DEEPER LEVEL]. By the end, you won't just
[OLD CAPABILITY]—you'll [NEW CAPABILITY]. You'll understand [MINDSET SHIFT] and
[ECONOMIC OPPORTUNITY]. This is the [MULTIPLIER] advantage: [OUTCOME].
```

#### 2.5 Design Learning Modules (4-6 modules, cohesive narrative)

**Module Structure:**

Each module needs:
- `module_id`: Sequential (1-6)
- `module_name`: Format: "Theme: Specific Focus"
- `module_description`: 2-3 sentences (purpose → skills → value)

**Module Naming Convention:**
```
[Number]. [Conceptual Theme]: [Specific Technology/Focus]
```

Examples:
- "Foundations: The AI-Native Infrastructure Paradigm"
- "Container Orchestration: Docker with AI-Driven Development"
- "Autonomous Systems: DAPR Actors for Agent Coordination"

**Module Description Formula:**
1. **Sentence 1**: What this module establishes (purpose/foundation)
2. **Sentence 2**: Specific skills/tools covered
3. **Sentence 3**: Connection to next module or constitution principle

**Copywriting:**
- Start each module with purpose (why this matters)
- Use specific tool/tech names
- Show progression across modules
- Connect to Nine Pillars where relevant
- Avoid generic language

**Module Progression Patterns:**

For AI-100/200 (Beginner/Intermediate):
1. Foundations (paradigm/mindset)
2. Tools Mastery (primary tool)
3. Communication (prompt/context engineering)
4. Core Development (language/framework)
5. Integration/Application (putting it together)

For AI-300/400 (Professional/Infrastructure):
1. Paradigm Shift (old → new)
2. Foundation Technology (Docker, Kubernetes, etc.)
3. AI-Native Patterns (SDD, AIDD application)
4. Orchestration Layer (DAPR, agents, coordination)
5. Production Deployment (autonomous, self-healing)

**Example Module (AI-400):**
```python
{
    "module_id": 3,
    "module_name": "Specification-Driven Containers: Docker with AIDD & SDD",
    "module_description": "Master Docker through specification-first development,
    where AI agents generate Dockerfiles, compose configurations, and deployment
    scripts from your intent. Learn container patterns for AI-native applications,
    including multi-stage builds, layer optimization, and agent-friendly networking.
    This module bridges traditional DevOps with AI-driven infrastructure."
}
```

#### 2.6 Identify Prerequisites

Extract from:
- Constitution (general prerequisites)
- Course level (technical prerequisites)
- Tools/tech mentioned (required background)

Format as Python list (empty if none):
```python
"pre_requisite": []  # For foundational courses
"pre_requisite": ["Basic Python knowledge", "Familiarity with command line"]
"pre_requisite": ["AI-300 or equivalent", "Docker basics", "Cloud platforms experience"]
```

#### 2.7 Media Link

Default: `"https://i.postimg.cc/XYLz3tSB/course-2.webp"`

---

### Phase 3: Python File Generation

#### 3.1 Generate Filename

Format: `{course_code}_{sanitized_course_name}.py`

Sanitization rules:
- Lowercase
- Replace spaces with hyphens
- Remove special characters (keep alphanumeric and hyphens)
- Max 80 characters total

Examples:
- `AI-400_cloud-native-to-agent-native-infrastructure.py`
- `AI-300_ai-driven-development-python-agentic-ai.py`

#### 3.2 Write Python File

**File Structure:**

```python
"""
Course: {course_name}
Code: {course_code}
Generated: {ISO timestamp}

Database-ready course definition.
Constitution: v3.1.2
"""

from datetime import datetime
from typing import Dict, List, Any


# Course metadata
COURSE_CODE = "{course_code}"
COURSE_NAME = "{course_name}"
COURSE_INITIALS = "{course_initials}"


# Course definition (database-ready)
course: Dict[str, Any] = {
    "course_code": "{course_code}",
    "course_name": "{course_name}",
    "course_initials": "{course_initials}",
    "course_description": """{multi-line description}""",
    "created_by": "db_admin",
    "updated_by": "db_admin",
    "course_outcomes": [  # Native Python list
        "Outcome 1",
        "Outcome 2",
        # ... 5-7 outcomes
    ],
    "long_description": """{multi-paragraph compelling narrative}""",
    "learning_modules": [  # Native Python list of dicts
        {
            "module_id": 1,
            "module_name": "Module Name",
            "module_description": "Description"
        },
        # ... 4-6 modules
    ],
    "pre_requisite": [  # Native Python list (empty if none)
        "Prerequisite 1",
        "Prerequisite 2"
    ],
    "media_link": "https://i.postimg.cc/XYLz3tSB/course-2.webp"
}


def get_course_dict() -> Dict[str, Any]:
    """Return complete course dictionary for database insertion."""
    return course.copy()


def get_course_code() -> str:
    """Return course code."""
    return course["course_code"]


def get_course_name() -> str:
    """Return course name."""
    return course["course_name"]


def get_course_outcomes() -> List[str]:
    """Return list of course learning outcomes."""
    return course["course_outcomes"].copy()


def get_learning_modules() -> List[Dict[str, Any]]:
    """Return list of learning modules."""
    return [m.copy() for m in course["learning_modules"]]


def get_prerequisites() -> List[str]:
    """Return list of prerequisites."""
    return course["pre_requisite"].copy()


def validate_course() -> Dict[str, bool]:
    """
    Validate course data structure.

    Returns:
        Dictionary with validation results
    """
    return {
        "has_code": bool(course.get("course_code")),
        "has_name": bool(course.get("course_name")),
        "has_outcomes": len(course.get("course_outcomes", [])) >= 4,
        "has_modules": len(course.get("learning_modules", [])) >= 4,
        "outcomes_are_list": isinstance(course.get("course_outcomes"), list),
        "modules_are_list": isinstance(course.get("learning_modules"), list),
    }


if __name__ == "__main__":
    print(f"Course: {COURSE_NAME}")
    print(f"Code: {COURSE_CODE}")
    print(f"Outcomes: {len(get_course_outcomes())} learning outcomes")
    print(f"Modules: {len(get_learning_modules())} learning modules")
    print(f"Prerequisites: {len(get_prerequisites())} items")
    print("\nValidation:", validate_course())
    print("\n✅ Course data ready for database insertion")
```

**Critical Requirements:**
- ✅ All string fields properly escaped (triple quotes for multi-line)
- ✅ Native Python types (list, dict - NOT JSON strings)
- ✅ Type hints on all functions
- ✅ Proper indentation (4 spaces)
- ✅ ISO timestamp in header
- ✅ Constitution version reference

#### 3.3 Write to File System

Path: `docs/course_outlines/py_format/{filename}.py`

Validate:
- File is syntactically valid Python
- Can be imported/executed without errors
- All data types are correct (list, dict, not string)

#### 3.4 Output Summary

Print:
```
✅ Course Designed: {course_name}
   Code: {course_code}
   Level: {AI-100/200/300/400}
   Outcomes: {count} items
   Modules: {count} items
   Prerequisites: {count} items

📄 File: docs/course_outlines/py_format/{filename}.py

STATUS=WROTE PATH=docs/course_outlines/py_format/{filename}.py
```

---

## Copywriting Quality Checklist

Before finalizing, verify:

**Course Description:**
- ✅ Starts with transformation, not topic list
- ✅ Mentions specific tools/technologies
- ✅ Uses active, present-tense language
- ✅ 2-3 sentences max (concise and punchy)

**Long Description:**
- ✅ Paragraph 1: Bold paradigm shift statement
- ✅ Paragraph 2: Methodology/framework introduced
- ✅ Paragraph 3: Learning journey with 3-5 key points
- ✅ Paragraph 4: "This isn't just X; it's Y" transformation
- ✅ Uses power words (architect, orchestrate, autonomous, mastery)
- ✅ Avoids passive voice and generic language

**Course Outcomes:**
- ✅ 5-7 outcomes (not more, not less)
- ✅ Start with strong action verbs (Build, Design, Architect, Master)
- ✅ Specific and measurable (not "learn about")
- ✅ First outcome addresses AI-native mindset
- ✅ Final outcome(s) emphasize production/mastery
- ✅ Bloom's taxonomy progression

**Learning Modules:**
- ✅ 4-6 modules (cohesive narrative arc)
- ✅ Each module name follows "Theme: Specific Focus" format
- ✅ Descriptions are 2-3 sentences (purpose → skills → connection)
- ✅ Progressive complexity across modules
- ✅ Specific tool/tech names (not generic)

---

## Constitution Alignment Verification

Before writing file, confirm:
- ✅ Nine Pillars referenced or implied
- ✅ AI Development Spectrum stage clear (Assisted/Driven/Native)
- ✅ "Specs Are the New Syntax" philosophy present
- ✅ Co-learning partnership emphasized
- ✅ Transformation language (role shift: coder → architect)
- ✅ 10x-99x multiplier mindset mentioned or implied
- ✅ Validation/evals mindset (for advanced courses)
- ✅ Production-ready emphasis (for AI-300/400)

---

## Example Usage

### Example 1: Book-Based Course
```bash
/course-designer AI-300 "parts 9-13"
```

### Example 2: Raw Ideas Course
```bash
/course-designer AI-400 """
Docker with SDD & AIDD
Kubernetes orchestration
DAPR for agent coordination
DAPR Actors for autonomous systems
DAPR Workflow for agent orchestration
"""
```

### Example 3: Context Reference
```bash
/course-designer AI-400 """
@context/cloud/
Plus Claude Code from Chapter 5
Plus SDD from Part 5
"""
```

---

## Success Criteria

**Output File Must:**
- ✅ Be valid Python (no syntax errors)
- ✅ Load and execute without errors
- ✅ Contain native Python types (list/dict, not JSON strings)
- ✅ Include compelling, professional copywriting
- ✅ Align with constitution v3.1.2
- ✅ Be located at `docs/course_outlines/py_format/{filename}.py`

**Copywriting Must:**
- ✅ Hook reader in first sentence
- ✅ Paint transformation journey
- ✅ Use specific technical terms
- ✅ Avoid generic/passive language
- ✅ Emphasize AI-native thinking
- ✅ Connect to economic opportunity

**STATUS Output:**
```
STATUS=WROTE PATH=docs/course_outlines/py_format/{filename}.py
```
