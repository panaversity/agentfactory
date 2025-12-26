---
sidebar_position: 3
title: "Pydantic Field & Parameter Descriptions"
description: "Master input validation through Pydantic Fields, descriptions, and constraints. Build MCP tools that reject invalid inputs before they reach your code."
keywords: ["pydantic", "field", "validation", "constraints", "annotated", "description", "MCP", "input schema", "B1-B2"]
chapter: 38
lesson: 3
duration_minutes: 55

skills:
  - name: "Pydantic Field Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Programming and Algorithm Design"
    measurable_at_this_level: "Student can apply Field() with constraints to real domain parameters"
  - name: "Input Validation Patterns"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Data Management and Protection"
    measurable_at_this_level: "Student can distinguish when to use ge/le vs min_length/max_length"
  - name: "Description Metadata for AI"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Communication and Collaboration"
    measurable_at_this_level: "Student can write descriptions that guide AI agents toward correct parameter values"

learning_objectives:
  - objective: "Apply Pydantic Field() with validation constraints to create self-documenting MCP tool parameters"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student implements field validation for domain-specific constraints"
  - objective: "Distinguish between Field constraint types (ge/le, min_length/max_length, pattern) and apply correctly"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student selects appropriate constraints for parameter types"
  - objective: "Write descriptions that guide AI agents toward correct parameter values through constraint clarity"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Student evaluates whether descriptions make validation constraints discoverable to agents"
  - objective: "Understand how Field metadata translates to OpenAI Inspector schema that guides agent behavior"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Student reads generated schema and traces Field definitions to schema properties"

cognitive_load:
  new_concepts: 7
  assessment: "Moderate - Introduces Field constraint types with real-world examples. Builds on previous lesson's Pydantic basics."

differentiation:
  extension_for_advanced: "Implement custom validators with field_validator() decorator and compose Field constraints with Annotated types for complex domain rules"
  remedial_for_struggling: "Focus on three essential constraints: description, min_length/max_length, and ge/le. Skip pattern and custom validators in initial pass"
---

# Pydantic Field & Parameter Descriptions

When you design an MCP tool, you're not just writing code for your own use. You're creating an interface that AI agents will discover and use autonomously. The difference between an agent using your tool correctly and struggling with parameter selection is often just description clarity.

Consider this scenario: You've built an MCP server that manages project tasks. Your tool takes a `priority` parameter. Is it 0-10? 1-5? "low", "medium", "high"? Without clear constraints and descriptions, the agent will guess—and will likely guess wrong. But with Pydantic Field metadata, the agent reads your schema and knows exactly what's valid.

This lesson teaches you to make parameters self-documenting through Pydantic Fields. You'll learn not just how to validate inputs, but how to write descriptions that communicate intent to both humans reviewing your code and AI agents discovering your tools.

## Understanding Field: From Type Hints to Validation

In Lesson 2, you learned that Pydantic BaseModel validates inputs automatically. But basic type hints alone aren't enough:

```python
from pydantic import BaseModel

class CreateTaskInput(BaseModel):
    title: str
    priority: int
    tags: list[str]
```

This model validates that `priority` is an integer, but an agent could pass `priority=100` or `priority=-5` without triggering validation errors. The type hint doesn't express domain constraints.

Pydantic Field() changes this by attaching metadata to parameters:

```python
from pydantic import BaseModel, Field
from typing import Annotated

class CreateTaskInput(BaseModel):
    title: Annotated[str, Field(description="Task title", min_length=1, max_length=100)]
    priority: Annotated[int, Field(description="Priority 1-5", ge=1, le=5)]
    tags: Annotated[list[str], Field(description="Labels for organization", max_items=10)]
```

**What changed:**
- `title` now rejects empty strings and strings longer than 100 characters
- `priority` rejects values below 1 or above 5
- `tags` rejects lists with more than 10 items
- Each parameter includes a description that appears in the tool's OpenAI Inspector schema

This metadata serves three purposes:

1. **Validation**: Pydantic enforces constraints before your tool code runs
2. **Discovery**: AI agents read schema and learn what values are valid
3. **Documentation**: Future developers (including your future self) understand intent

## Seven Core Field Constraints

Different data types need different constraints. Here's the decision framework:

**For strings: Description + Length Constraints**

```python
username: Annotated[str, Field(
    description="Username for login (3-20 alphanumeric characters)",
    min_length=3,
    max_length=20,
    pattern=r"^[a-zA-Z0-9_]+$"  # Optional: enforce format
)]
```

**For integers and floats: Description + Range Constraints**

```python
priority: Annotated[int, Field(
    description="Priority level from 1 (lowest) to 5 (highest)",
    ge=1,  # greater than or equal
    le=5   # less than or equal
)]

discount_percent: Annotated[float, Field(
    description="Discount percentage: 0-100%",
    ge=0.0,
    le=100.0
)]
```

**For lists: Description + Item Limits**

```python
team_members: Annotated[list[str], Field(
    description="Email addresses of team members (2-50 people)",
    min_items=2,
    max_items=50
)]
```

**For enums: Description + Allowed Values**

```python
from enum import Enum

class Priority(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"

status: Annotated[Priority, Field(
    description="Priority: 'low', 'medium', or 'high'"
)]
```

**For optional fields: Description + Default Value**

```python
due_date: Annotated[Optional[str], Field(
    default=None,
    description="Due date in YYYY-MM-DD format (optional)"
)]

tags: Annotated[list[str], Field(
    default_factory=list,
    description="Labels for organization (optional, defaults to empty)"
)]
```

**Constraints by Type Summary:**

| Type | Constraint | Example |
|------|-----------|---------|
| `str` | `min_length`, `max_length`, `pattern` | `min_length=1, max_length=100` |
| `int`, `float` | `ge`, `le`, `gt`, `lt` | `ge=1, le=5` |
| `list` | `min_items`, `max_items` | `max_items=10` |
| Any type | `description` (always) | `description="Clear intent"` |
| Optional | `default`, `default_factory` | `default=None` |

## Three Roles Through Iteration: Building Better Validations

Let's walk through how discovery and refinement work together. You'll write initial validations, test with AI suggestions, and refine based on domain feedback.

### Initial Design: Basic Constraints

You start with a task creation tool:

```python
from pydantic import BaseModel, Field
from typing import Annotated

class CreateTaskInput(BaseModel):
    title: Annotated[str, Field(description="Task title")]
    priority: Annotated[int, Field(description="Priority level")]
    due_date: Annotated[str, Field(description="Due date")]
```

The constraints are there, but minimal. The descriptions don't guide agents toward valid values.

### Suggestion: AI Pattern for Task Priority

When you ask an AI agent to improve your schema, it might suggest:

"For task priority, consider constraints that match your domain:
- If priority represents a fixed scale (Jira style): `ge=1, le=5`
- If your team has specific definitions: Use Enum with explicit values
- Add example in description: 'Priority 1-5 (1=lowest, 5=urgent)'"

This teaches you a pattern: **Range constraints clarify when values form a spectrum.**

### Refinement: Your Domain Knowledge

But the AI doesn't know your business rule: "Tasks with priority 1-2 are automatically scheduled two weeks out. Priority 4-5 are urgent." So you refine:

```python
priority: Annotated[int, Field(
    description="Priority 1-5: 1-2=scheduled weeks out, 3=standard, 4-5=urgent",
    ge=1,
    le=5
)]

due_date: Annotated[str, Field(
    description="Due date YYYY-MM-DD. Required for priority 4-5 (urgent).",
    pattern=r"^\d{4}-\d{2}-\d{2}$"
)]
```

### Convergence: Validated and Documented

The final schema now:
- Validates inputs (Pydantic rejects invalid priority or date format)
- Guides agents ("Priority 4-5 are urgent → due_date is essential")
- Documents business logic ("1-2=scheduled weeks out" is now visible)

This is convergence: Neither AI nor your first instinct produced the final design. Iteration between suggested patterns and domain knowledge did.

## How Field Metadata Appears in MCP Schema

When you register an MCP tool, FastMCP generates an OpenAI Inspector schema that looks like this:

**Your Python code:**
```python
@mcp.tool()
async def create_task(params: CreateTaskInput) -> str:
    """Create a new task in the project."""
    pass
```

**Generated Schema (MCP Inspector sees this):**
```json
{
  "name": "create_task",
  "description": "Create a new task in the project.",
  "inputSchema": {
    "type": "object",
    "properties": {
      "title": {
        "type": "string",
        "description": "Task title",
        "minLength": 1,
        "maxLength": 100
      },
      "priority": {
        "type": "integer",
        "description": "Priority 1-5",
        "minimum": 1,
        "maximum": 5
      },
      "due_date": {
        "type": "string",
        "description": "Due date in YYYY-MM-DD format",
        "pattern": "^\\d{4}-\\d{2}-\\d{2}$"
      }
    },
    "required": ["title", "priority"]
  }
}
```

Agents read this schema. Clear descriptions + constraints = agents making valid requests.

## Cognitive Load: Distinguishing Field Constraint Types

This is where many developers struggle: Which constraint for which situation?

| Question | Constraint | Example |
|----------|-----------|---------|
| "Is this string limited by length?" | `min_length`, `max_length` | Username: `min_length=3, max_length=20` |
| "Does this number have a range?" | `ge`, `le` | Priority: `ge=1, le=5` |
| "What format must this string match?" | `pattern` | Date: `pattern=r"^\d{4}-\d{2}-\d{2}$"` |
| "Can this list have any number of items?" | `min_items`, `max_items` | Tags: `max_items=10` |
| "Can this field be empty/missing?" | `default`, `default_factory` | Tags: `default_factory=list` |

**Common mistake:** Using `ge`/`le` on strings or `min_length`/`max_length` on integers. They're type-specific.

## Try With AI: Designing Parameters for a Real Domain

You'll work through parameter design for a domain-specific tool, discovering constraints through iteration.

### Part 1: Initial Parameter Design

Imagine you're building an MCP server for a design tool (like Figma). You need a tool to create artboards (canvases) with size constraints.

**Your initial attempt:**

Ask AI: "I'm building a create_artboard tool for a design system. What parameters should this tool have? Consider that artboards have width/height in pixels, and they need names for organization."

**Expected response:**
AI suggests parameters like name, width, height, description. But constraints are vague.

### Part 2: Discovering Domain Constraints

Now tell AI your specific constraints:

"In our design system:
- Artboard names must be 1-50 characters and alphanumeric
- Width and height are in pixels, minimum 100px, maximum 4000px
- We support standard sizes (mobile: 375x812, tablet: 768x1024, desktop: 1920x1080)
- Description is optional, max 500 characters
- We track which design team owns the artboard"

### Part 3: Refining Parameter Validation

Based on your constraints, ask AI:

"Using Pydantic Field() with Annotated types, design the input model for this tool. Make the schema guide agents toward valid values."

### Part 4: Review Generated Schema

Ask AI to show the generated OpenAI Inspector schema:

"Show me the inputSchema that FastMCP would generate from this Pydantic model. Does the description make the constraints discoverable to agents?"

### Part 5: Iterate on Description Quality

Evaluate the descriptions. Are they clear? Do they guide agents?

Ask: "Improve these three descriptions so an agent reading them would understand the constraints without reading the code."

**What you're learning:**
- How constraint types vary by data type (strings, integers, lists)
- How descriptions make constraints discoverable to agents
- How to iterate between AI suggestions and domain knowledge
- How Field metadata translates to OpenAI Inspector schema

**Safety Note:** When designing parameters for tools that modify data (create, update, delete), comprehensive validation prevents both user errors and malicious input. Consider constraints as your first line of defense.

**Next Steps:**
After this lesson, you'll move to custom validators with `field_validator()` decorator, enabling complex validation logic that spans multiple fields or requires API calls.
