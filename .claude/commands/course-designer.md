---
description: Intelligence-driven course designer using LoopFlow principles. Reads constitution + book content to generate compelling, database-ready courses automatically. Minimal questions, maximum intelligence.
argument-hint: [course-code] [book-parts OR topic-description]
---

# /course-designer: Intelligence-Driven Course Design

**Purpose**: Generate compelling, constitution-aligned course outlines from book content OR raw ideas using **vertical intelligence** (constitution + book structure + copywriting frameworks). Goal-oriented, context-adaptive, and database-ready.

**Intelligence Sources**:
- Constitution: Nine Pillars, AI Development Spectrum, target audience, 10x-99x multiplier
- Book Content: Chapter index, part READMEs, lesson structures
- Copywriting Frameworks: AIDA, PAS, transformation narratives
- Domain Context: Existing course patterns, market positioning

**Adaptive Workflow**: 0-3 targeted questions based on what intelligence can't derive.

## User Input

```text
$ARGUMENTS
```

---

## CORE PHILOSOPHY: INTELLIGENCE-DRIVEN COURSE DESIGN

**From**: Manual course planning with 20+ decisions
**To**: "Design AI-400 from parts 10-13" → Compelling, ready-to-use course

### The LoopFlow Approach

1. **User states GOAL**: "Design AI-400 from parts 10-13" OR "Create course on Docker/K8s/DAPR"
2. **AI reads AUTHORITATIVE SOURCES**: Constitution, book content, existing courses
3. **AI derives INTELLIGENCE**: Target audience, complexity level, tools covered, module structure
4. **AI asks TARGETED QUESTIONS**: Only what's genuinely ambiguous (0-3 questions max)
5. **AI generates OUTPUT**: Database-ready Python file with compelling copy

**Why This Works**:
- ✅ Intelligence-driven: Constitution eliminates guesswork
- ✅ Context-aware: Book structure informs modules automatically
- ✅ Compelling copy: Copywriting frameworks built-in
- ✅ Database-ready: Native Python types, no manual conversion

---

## WORKFLOW PHASES

### PHASE 0: Intelligent Context Discovery

**Automatic Intelligence Gathering** (NO user interaction):

1. **Read Constitution** (`.specify/memory/constitution.md`):
   - Nine Pillars framework
   - AI Development Spectrum (Assisted → Driven → Native)
   - "Specs Are the New Syntax" philosophy
   - Co-learning partnership principles
   - Target audience tiers
   - 10x-99x multiplier mindset

2. **Detect Content Source** (from user input):
   - **Pattern 1**: "parts X-Y" → Book content (Glob chapters in `apps/learn-app/docs/`)
   - **Pattern 2**: Multi-line description → Raw ideas (parse topics/tools)
   - **Pattern 3**: "@context/path" → Context files (read specified directory)

3. **Load Book Structure** (if applicable):
   - Chapter index (`specs/book/chapter-index.md`)
   - Part-level READMEs
   - Chapter titles and learning objectives
   - Tools/technologies covered

4. **Derive Course Intelligence** (automatic):
   ```python
   # From course code
   if course_code.startswith("AI-100"):
       level = "beginner"
       audience = "Aspiring (foundational)"
       parts_range = [1, 2, 3]
   elif course_code.startswith("AI-200"):
       level = "intermediate"
       audience = "Aspiring/Professional (intermediate)"
       parts_range = [4, 5, 6, 7, 8]
   elif course_code.startswith("AI-300"):
       level = "professional"
       audience = "Professional (production agentic AI)"
       parts_range = [9, 10, 11, 12, 13]
   elif course_code.startswith("AI-400"):
       level = "infrastructure"
       audience = "Professional (cloud-native to agent-native)"
       parts_range = [10, 11, 12, 13]  # Infrastructure-focused

   # From book content or topics
   tools_detected = ["Claude Code", "Docker", "Kubernetes", "DAPR", "Python", etc.]
   pillars_covered = [1, 2, 3, 6, 9]  # Which Nine Pillars
   spectrum_stage = "AI-Native" or "Driven" or "Assisted"
   ```

5. **Load Copywriting Intelligence**:
   - AIDA Framework (Attention → Interest → Desire → Action)
   - PAS Framework (Problem → Agitate → Solve)
   - Transformation language patterns
   - Power words library

**Adaptive Questions** (0-3 questions, ONLY if genuinely ambiguous):
- IF course name unclear → Ask: "Course title: [suggestion] or custom?"
- IF multiple valid module structures → Ask: "5 modules (depth) or 6 modules (breadth)?"
- IF existing similar course → Ask: "Differentiate from AI-300 or complement it?"

---

### PHASE 1: Course Metadata Generation

**Automatic Synthesis** (from intelligence):

**course_code**: User-provided (validated: AI-100/200/300/400 format)

**course_name**: Auto-generated formula:
```
{Primary Tech/Domain} + {Transformation Focus} + {Methodology}
```

Examples:
- "AI-Driven Development with Python and Agentic AI"
- "Cloud-Native AI — Learn Dapr, Docker & Kubernetes with AIDD"
- "Specification-Driven Development: Building Software with AI Partners"

**course_initials**: Extract meaningful acronym:
- AI-Driven Development with Python and Agentic AI → "AIDD-PAI"
- Cloud-Native AI → "CNAI"
- Specification-Driven Development → "SDD"

**course_description** (2-3 sentences):
- Formula: [Hook with transformation] + [Tools to master] + [Outcome/who you become]
- **Good**: "Build cloud-native infrastructure for intelligent agent systems. Using AI-Driven Development (AIDD), you'll learn Docker, Kubernetes, and Dapr to design and deploy production-ready AI agents with observability, scalability, and cloud-agnostic flexibility — laying the foundation for autonomous AI at scale."
- **Bad**: "Learn about cloud technologies and containers." (too generic)

---

### PHASE 2: Course Outcomes Generation (5-7 outcomes)

**Automatic Structure** (Bloom's taxonomy progression):

1. **Outcome 1**: Always AI-Native Mindset
   ```
   "Understand AI-native thinking: specs-first, validation-focused, co-learning partnership"
   ```

2. **Outcomes 2-3**: Tool/Tech Mastery (specific to course)
   ```
   "Master {primary tool} using {methodology} for {use case}"
   "Partner with {AI tool} to {generate|design|orchestrate} {artifact}"
   ```

3. **Outcomes 4-5**: Pattern Application
   ```
   "Apply {SDD|AIDD|Pattern} to {specific task} using {tool}"
   ```

4. **Outcomes 6-7**: Production Skills (for AI-300/400)
   ```
   "Build production-ready {systems} with {features}"
   "Architect autonomous {systems} that {self-organize|adapt|scale}"
   ```

**Copywriting Rules**:
- ✅ Start with strong action verbs (Master, Build, Design, Architect, Apply, Implement)
- ✅ Be specific about what they'll build
- ✅ Connect to real-world production
- ❌ Avoid "learn about", "understand basics", "explore"

**Example (AI-400)**:
```python
[
    "Apply Context Engineering to structure effective AI collaboration for infrastructure design",
    "Partner with Claude Code to generate production-ready cloud configurations from specifications",
    "Master Spec-Driven Development (SDD) to design infrastructure through clear intent, not manual YAML",
    "Containerize AI applications with Docker using AIDD and SDD for multi-stage builds and optimization",
    "Orchestrate agent systems on Kubernetes with AIDD and SDD using kubectl-ai and kagent",
    "Implement Dapr Core and Dapr Workflows for cloud-agnostic state, pub/sub, and long-running processes",
    "Build observable, scalable AI systems with OpenTelemetry, autoscaling, and automated CI/CD pipelines"
]
```

---

### PHASE 3: Long Description Generation (3-4 paragraphs)

**Automatic Narrative Structure**:

**Paragraph 1: The Paradigm Shift (Hook + Problem)**

Template:
```
The era of {OLD PARADIGM} is over. The future isn't about {OLD SKILL}; it's about {NEW CAPABILITY}.
{Course Focus} {ACTION} you {NEW SKILL}, {WHERE/WHEN/HOW}. {METHODOLOGY} introduces {FRAMEWORK},
moving you from {OLD ROLE} to {NEW ROLE}. This is how {DOMAIN} will be {DONE} for the next decade.
```

Apply copywriting:
- Bold opening ("era...is over")
- Contrast (old vs new)
- Urgency (why NOW)
- Connect to constitution principles

**Paragraph 2: The Methodology (Solution + Approach)**

Template:
```
In this course, you'll learn {PRIMARY SKILL} using {METHODOLOGY}. {APPROACH DESCRIPTION}.
Each step is {CHARACTERISTIC}. {AI PARTNERSHIP STATEMENT}. {CONSTITUTION ALIGNMENT}.
```

Apply copywriting:
- Name methodology (AIDD, SDD, AI-Native)
- Explain learning approach (not just content list)
- Emphasize AI partnership
- Connect to Nine Pillars

**Paragraph 3: The Journey (Module Walk-Through)**

Template:
```
You'll {MODULE 1 ACTION}, {MODULE 1 OUTCOME}. Then {MODULE 2 ACTION}, {MODULE 2 SPECIFIC}.
{MODULE 3 ACTION} to {MODULE 3 RESULT}. Finally, you'll {MODULE 4-5 ACTION}, {ADVANCED CAPABILITY}.
{TOOLS LIST}. {HANDS-ON EMPHASIS}.
```

Apply copywriting:
- Journey language ("start", "then", "finally")
- Name 3-5 key milestones
- Mention specific tools
- Emphasize hands-on building

**Paragraph 4: The Transformation (Outcome + Call to Action)**

Template:
```
This {isn't just|turns} {SURFACE LEVEL} {into|—it's} {DEEPER LEVEL}. By the end, you'll
{NEW CAPABILITY}, not just {OLD CAPABILITY}. {MULTIPLIER STATEMENT}. {WHO YOU BECOME}.
```

Apply copywriting:
- "Not just X; it's Y" contrast
- Paint future state
- Connect to 10x-99x multiplier
- End with empowerment

---

### PHASE 4: Learning Modules Generation (4-6 modules)

**Automatic Module Structure** (from book content or topics):

**Module Naming Convention**:
```
{Conceptual Theme}: {Specific Technology/Focus}
```

Examples:
- "Foundations: Cloud Native Infrastructure for AI"
- "Docker Fundamentals: Containerizing AI Applications"
- "Kubernetes Basics: Orchestrating Agent Systems"
- "DAPR Core: Cloud-Agnostic Abstractions"
- "Production Operations: Observability, Scaling & CI/CD"

**Module Description Formula** (2-3 sentences):
1. **Purpose**: What this module establishes
2. **Skills**: Specific tools/patterns covered
3. **Connection**: Link to next module or constitution

**Automatic Module Assignment** (from book chapters):
```python
# Group chapters into modules (4-6 chapters per module typically)
if parts == [1, 2, 3]:
    modules = [
        {"chapters": [1-4], "theme": "Foundations"},
        {"chapters": [5-8], "theme": "Tools Mastery"},
        {"chapters": [9-11], "theme": "Communication"},
    ]
elif parts == [9, 10, 11, 12, 13]:
    modules = [
        {"chapters": [30-33], "theme": "Agentic Foundations"},
        {"chapters": [34-38], "theme": "Agents SDK"},
        {"chapters": [39-43], "theme": "Multi-Agent Systems"},
        {"chapters": [44-48], "theme": "Production Deployment"},
    ]
```

**Copywriting for Modules**:
- ✅ Start with purpose (why this matters)
- ✅ Use specific tool/tech names
- ✅ Show progression
- ❌ Avoid generic language

---

### PHASE 5: Python File Generation

**Automatic File Creation**:

**Filename**: `{course_code}_{sanitized_course_name}.py`

Sanitization:
- Lowercase
- Spaces → hyphens
- Remove special chars
- Max 80 chars

**File Structure** (database-ready):

```python
"""
Course: {course_name}
Code: {course_code}
Generated: {ISO timestamp}
Constitution: v3.1.3
"""

from datetime import datetime
from typing import Dict, List, Any


# Course metadata
COURSE_CODE = "{course_code}"
COURSE_NAME = "{course_name}"
COURSE_INITIALS = "{course_initials}"


# Course definition (database-ready with native Python types)
course: Dict[str, Any] = {
    "course_code": COURSE_CODE,
    "course_name": COURSE_NAME,
    "course_initials": COURSE_INITIALS,
    "course_description": """{compelling 2-3 sentence description}""",
    "is_active": True,
    "is_offered_now": True,
    "created_by": "db_admin",
    "updated_by": "db_admin",
    "order": 1,  # Derived from course level
    "program_id": 1,

    # Native Python lists (NOT JSON strings)
    "course_outcomes": [
        "Outcome 1: AI-native mindset",
        "Outcome 2: Tool mastery",
        # ... 5-7 total
    ],

    "long_description": """{
        4-paragraph compelling narrative with:
        - Paragraph 1: Paradigm shift hook
        - Paragraph 2: Methodology introduction
        - Paragraph 3: Learning journey
        - Paragraph 4: Transformation outcome
    }""",

    "learning_modules": [  # Native Python list of dicts
        {
            "module_id": 1,
            "module_name": "Theme: Specific Focus",
            "module_description": "Purpose. Skills covered. Connection."
        },
        # ... 4-6 modules
    ],

    "pre_requisite": [  # Native Python list (empty if none)
        "Prerequisite 1",
        # or []
    ],

    "media_link": "https://i.postimg.cc/XYLz3tSB/course-2.webp"
}


def get_course_dict() -> Dict[str, Any]:
    """Return complete course dictionary for database insertion."""
    return course.copy()


def get_course_code() -> str:
    """Return course code."""
    return course["course_code"]


def get_outcomes() -> List[str]:
    """Return list of course learning outcomes."""
    return course["course_outcomes"].copy()


def get_modules() -> List[Dict[str, Any]]:
    """Return list of learning modules."""
    return [m.copy() for m in course["learning_modules"]]


def validate() -> Dict[str, bool]:
    """Validate course data structure."""
    return {
        "has_code": bool(course.get("course_code")),
        "has_name": bool(course.get("course_name")),
        "outcomes_count": len(course.get("course_outcomes", [])) >= 5,
        "modules_count": 4 <= len(course.get("learning_modules", [])) <= 6,
        "native_types": (
            isinstance(course.get("course_outcomes"), list) and
            isinstance(course.get("learning_modules"), list)
        ),
    }


if __name__ == "__main__":
    print(f"Course: {COURSE_NAME}")
    print(f"Code: {COURSE_CODE}")
    print(f"Outcomes: {len(get_outcomes())}")
    print(f"Modules: {len(get_modules())}")
    print(f"\nValidation: {validate()}")
    print("\n✅ Database-ready Python course definition")
```

**Critical Requirements**:
- ✅ Native Python types (list, dict — NOT JSON strings)
- ✅ Triple-quoted strings for multi-line
- ✅ Type hints on all functions
- ✅ ISO timestamp in header
- ✅ Constitution version reference

---

## COPYWRITING FRAMEWORKS (BUILT-IN)

### AIDA Framework
- **Attention**: "The era of X is over"
- **Interest**: Concrete tools and methodologies
- **Desire**: Transformation narrative (10x-99x multiplier)
- **Action**: "This is how software will be built"

### PAS Framework
- **Problem**: "The future isn't about X"
- **Agitate**: "It's about Y" (new capability)
- **Solve**: "This course teaches Z"

### Transformation Language
- From "coder" → to "architect"
- From "typing syntax" → to "specifying intent"
- From "debugging" → to "validating systems"
- From "tool user" → to "AI co-creator"

### Power Words (Auto-Applied)
- Paradigm shift, revolution, transformation
- Master, architect, orchestrate, build, design
- Production-ready, professional-grade
- Autonomous, intelligent, agentic
- 10x-99x multiplier, exponential

---

## CONSTITUTION ALIGNMENT (AUTOMATIC)

Before generating, verify:
- ✅ Nine Pillars referenced (explicitly or implicitly)
- ✅ AI Development Spectrum stage clear
- ✅ "Specs Are the New Syntax" emphasized
- ✅ Co-learning partnership present
- ✅ Transformation language (role shift)
- ✅ 10x-99x multiplier mentioned
- ✅ Validation-first for advanced courses
- ✅ Production-ready for AI-300/400

---

## USAGE EXAMPLES

### Example 1: Book-Based Course (Parts Reference)

```bash
/course-designer AI-300 "parts 9-13"
```

**Intelligence Derived**:
- Level: Professional (AI-300)
- Parts: 9-13 (Agentic AI, production deployment)
- Tools: Python, Agents SDK, multi-agent systems
- Audience: Professional developers
- Spectrum: AI-Native (50-99x multiplier)

**Adaptive Questions**: 0-1
- "Course title: 'AI-Driven Development with Python and Agentic AI' or custom?"

---

### Example 2: Topic-Based Course (Raw Description)

```bash
/course-designer AI-400 """
Docker containerization with SDD
Kubernetes orchestration with kubectl-ai
DAPR for cloud-agnostic agent systems
DAPR Workflows for long-running processes
Production observability with OpenTelemetry
"""
```

**Intelligence Derived**:
- Level: Infrastructure (AI-400)
- Tools: Docker, Kubernetes, DAPR, OpenTelemetry
- Methodology: SDD, AIDD
- Audience: Professional (infrastructure focus)
- Pillars: 1 (AI CLI), 6 (TDD), 7 (SDD), 9 (Cloud-Native)

**Adaptive Questions**: 1-2
- "Course title: 'Cloud-Native AI — Learn Dapr, Docker & Kubernetes with AIDD' or custom?"
- "5 modules (depth) or 6 modules (breadth)?"

---

### Example 3: Context-Based Course

```bash
/course-designer AI-400 """
@context/cloud-native/
Plus SDD from Part 5 (chapters 30-33)
Plus Claude Code from Chapter 5
"""
```

**Intelligence Derived**:
- Reads context files from `context/cloud-native/`
- Cross-references with chapters 30-33, chapter 5
- Synthesizes infrastructure + SDD methodology
- Detects tools: Claude Code, Docker, K8s, DAPR

---

## EXECUTION OUTPUT

```
🎯 Designing Course: AI-400

📖 Reading authoritative sources...
  ✓ Constitution v3.1.3 (Nine Pillars, AI Spectrum, target audience)
  ✓ Book content: Parts 10-13 (25 chapters on cloud-native infrastructure)
  ✓ Tools detected: Docker, Kubernetes, DAPR, OpenTelemetry, Claude Code
  ✓ Copywriting frameworks loaded (AIDA, PAS, transformation patterns)

🧠 Deriving course intelligence...
  ✓ Level: AI-400 (Infrastructure / Professional)
  ✓ Audience: Professional developers
  ✓ Spectrum: AI-Native (50-99x multiplier)
  ✓ Pillars: 1, 6, 7, 9 (AI CLI, TDD, SDD, Cloud-Native)
  ✓ Methodology: SDD + AIDD

🤔 Adaptive question (1):
  Q: Course title: "Cloud-Native AI — Learn Dapr, Docker & Kubernetes with AIDD" or custom?
  [User: Use suggested title]

✅ Course Generated: Cloud-Native AI
   Code: AI-400
   Initials: CNAI
   Outcomes: 7 learning outcomes
   Modules: 5 learning modules
   Prerequisites: 0 items

📄 File: docs/course_outlines/py_format/AI-400_cloud-native-ai.py

✅ Database-ready Python course definition created

STATUS=WROTE PATH=docs/course_outlines/py_format/AI-400_cloud-native-ai.py
```

---

## QUALITY CHECKLIST (AUTOMATIC)

**Course Description**:
- ✅ Starts with transformation (not topic list)
- ✅ Mentions specific tools
- ✅ Active, present-tense language
- ✅ 2-3 sentences (concise)

**Long Description**:
- ✅ P1: Paradigm shift hook
- ✅ P2: Methodology introduced
- ✅ P3: Learning journey (3-5 points)
- ✅ P4: "Not just X; it's Y" transformation
- ✅ Power words used
- ✅ No passive voice

**Outcomes**:
- ✅ 5-7 outcomes
- ✅ Strong action verbs
- ✅ Specific and measurable
- ✅ First = AI-native mindset
- ✅ Final = production/mastery
- ✅ Bloom's progression

**Modules**:
- ✅ 4-6 modules
- ✅ "Theme: Focus" format
- ✅ 2-3 sentence descriptions
- ✅ Progressive complexity
- ✅ Specific tool names

**Python File**:
- ✅ Valid syntax
- ✅ Native types (not JSON strings)
- ✅ Type hints
- ✅ Helper functions
- ✅ Validation function

---

## CRITICAL SUCCESS FACTORS

1. **Intelligence-Driven**: Constitution + book content read FIRST, questions SECOND
2. **Compelling Copy**: Copywriting frameworks applied automatically
3. **Database-Ready**: Native Python types, no conversion needed
4. **Constitutional Alignment**: Nine Pillars, spectrum, transformation language
5. **Minimal Questions**: 0-3 questions (only genuinely ambiguous items)
6. **Production Quality**: Professional copy, validated structure

---

## ONE COMMAND. MAXIMUM INTELLIGENCE. COMPELLING OUTPUT.

Run `/course-designer [code] [source]` and the system:

1. **Reads Constitution** → Nine Pillars, audience, spectrum, multiplier
2. **Loads Content** → Book chapters, context files, or raw topics
3. **Derives Intelligence** → Level, tools, modules, outcomes (automatic)
4. **Asks Targeted Questions** → 0-3 questions (only if ambiguous)
5. **Generates Compelling Copy** → Copywriting frameworks applied
6. **Outputs Python File** → Database-ready, native types, validated

**Result**: Professional, compelling course outline with constitutional alignment and zero manual copywriting.

---

**See also**:
- `/sp.loopflow.md` - Universal SDD Loop orchestrator (similar intelligence approach)
- `/sp.python-chapter.md` - Python chapter orchestrator (similar adaptive questions)
- `.specify/memory/constitution.md` - Project constitution (source of truth)
