---
name: learning-objectives
description: |
  Generate measurable learning outcomes aligned with Bloom's taxonomy for educational content.
  Use this skill when educators need to define what students will achieve, create learning objectives
  for curriculum planning, or ensure objectives are specific and testable rather than vague.
  This skill helps break down complex topics into progressively building learning goals with clear
  assessment methods and success criteria.
allowed-tools: ["Read", "Bash", "Write"]
version: "1.0.0"
---

# Learning Objectives Skill

## Purpose

Enable educators to create **measurable, actionable learning objectives** aligned with Bloom's taxonomy. This skill helps:
- Define what students will achieve (not just what topics they'll cover)
- Ensure objectives are specific and testable (not vague)
- Identify prerequisites and scaffold learning progressively
- Plan appropriate assessment methods
- Sequence learning from basic recall to creative synthesis

## When to Activate

Use this skill when:
- Planning curriculum or lesson design and need to define learning outcomes
- Creating assessments and want to align them with clear objectives
- Designing a course and need measurable outcomes for accreditation
- Educators ask to "define objectives", "create learning goals", "set outcomes", or "what should students achieve?"
- Reviewing existing objectives and wondering if they're specific enough
- Designing a lesson and unsure what students should be able to do by the end

## Process

### Step 1: Understand the Context

When a request comes in to generate learning objectives, first understand:
- **What topic or concept?** (Python decorators, OOP, async/await, etc.)
- **Who are the learners?** (Beginners, intermediate, advanced)
- **How long to teach?** (30 minutes, 2 hours, full unit)
- **What's the end goal?** (Can they build something? Understand theory? Debug code?)

### Step 2: Review Bloom's Taxonomy (If Needed)

If you're not familiar with the specific topic's cognitive levels, read:
📖 [reference/blooms-taxonomy-programming.md](reference/blooms-taxonomy-programming.md)

This document maps Bloom's 6 levels to programming contexts with:
- Action verbs for each level (Remember, Understand, Apply, Analyze, Evaluate, Create)
- Programming examples
- Assessment methods for each level

### Step 3: Identify Prerequisites

Read the guidance on prerequisite analysis:
📖 [reference/prerequisite-analysis.md](reference/prerequisite-analysis.md)

For your objectives, determine:
- What must learners know BEFORE tackling the main concept?
- List prerequisites at Remember/Understand level (not deep mastery)
- Trace dependency chains to foundational knowledge

### Step 4: Choose Assessment Methods

Based on the Bloom's level of each objective, review appropriate assessment methods:
📖 [reference/assessment-methods.md](reference/assessment-methods.md)

This guides you to pair objectives with realistic assessment approaches (code exercises for Apply level, code reviews for Evaluate, etc.).

### Step 5: Generate Objectives

For each topic, create 3-5 objectives (typically):
- At least one from each level needed for the topic (Remember through Create)
- Progressively building in complexity
- Each with clear statement, context, prerequisites, assessment method, and success criteria

Use the template as guidance:
📄 [templates/learning-objective-template.yml](templates/learning-objective-template.yml)

**Key principle**: Each objective should answer:
- **What will learners DO?** (verb from Bloom's level)
- **In what context?** (the specific situation or problem)
- **How will we know they succeeded?** (measurable criteria)

### Step 6: Validate for Measurability

Once you've generated objectives, invoke the validation script to check they're measurable:

```bash
python .claude/skills/learning-objectives/scripts/validate-objectives.py objectives.yml
```

The script checks:
- ✅ Objectives use action verbs (not vague words like "understand" or "know")
- ✅ Objectives are specific and testable
- ✅ Verbs match the Bloom's level
- ✅ Required fields present (context, assessment, success criteria)

If validation fails, review the error messages and refine:
- Replace vague verbs with specific action verbs from Bloom's level
- Add missing context or success criteria
- Make statements more specific (not just topic names)

### Step 7: Refine and Finalize

Based on validation feedback:
- Revise any objectives with errors
- Ensure the objective sequence makes sense (build progressively)
- Verify prerequisites are necessary but not excessive
- Check that assessment methods are realistic for the time available

## Output Format

Present learning objectives in clear, structured format:

```yaml
topic: "Topic Name"
objectives:
  - id: "LO-001"
    statement: "Action verb + what learner will do"
    blooms_level: "Remember/Understand/Apply/Analyze/Evaluate/Create"
    context: "Specific situation or problem"
    prerequisites:
      - "Prerequisite 1"
      - "Prerequisite 2"
    assessment_method: "How this will be assessed"
    success_criteria:
      - "Criterion 1"
      - "Criterion 2"
```

## Acceptance Checks

- [ ] SpecRef present (path to chapter spec and objective IDs)
- [ ] ≤ 5 objectives; measurable verbs only
- [ ] Each objective has Bloom tag (e.g., L2-Understand, L3-Apply)
- [ ] Each objective includes an “Evidence” mapping to assessment items or tasks
- [ ] Objectives align with complexity tier constraints (see Constitution)

### Objective snippet
```
- [ID: LO-001 | Bloom: L3-Apply]
  Statement: Implement X in Y context
  Evidence: Quiz Q3, Task T2
```

## Examples

### Example 1: Python List Comprehensions (1-hour lesson)

**User Request**: "Create objectives for teaching Python list comprehensions to beginners"

**Generated Objectives**:

```yaml
topic: "Python List Comprehensions"
objectives:
  - id: "LO-C1-001"
    statement: "Define list comprehension and identify its basic syntax"
    blooms_level: "Remember"
    context: "Introduction to list comprehensions"
    prerequisites:
      - "Know Python list syntax and indexing"
      - "Can write basic for loops"
    assessment_method: "Quick quiz: match syntax to purpose"
    success_criteria:
      - "Can identify list comprehension in code"
      - "Can label parts of syntax (brackets, for, if)"

  - id: "LO-C1-002"
    statement: "Explain how list comprehension replaces for loop patterns"
    blooms_level: "Understand"
    context: "Comparing traditional loops with comprehensions"
    prerequisites:
      - "Understand for loop iteration"
    assessment_method: "Paraphrase: explain in own words"
    success_criteria:
      - "Can describe equivalent for loop for any list comprehension"
      - "Can explain what's happening step-by-step"

  - id: "LO-C1-003"
    statement: "Implement list comprehensions to filter and transform data"
    blooms_level: "Apply"
    context: "Given various data transformation tasks"
    prerequisites:
      - "Can write list comprehensions with filter conditions"
      - "Understand string/number methods"
    assessment_method: "Code exercise: write comprehensions for provided requirements"
    success_criteria:
      - "Code executes without errors"
      - "Output matches expected results"
      - "Uses list comprehension (not a for loop)"

  - id: "LO-C1-004"
    statement: "Compare list comprehension with traditional loops by examining efficiency and readability"
    blooms_level: "Analyze"
    context: "Choosing approaches for data processing"
    prerequisites:
      - "Implement list comprehensions fluently"
      - "Understand basic time complexity"
    assessment_method: "Code analysis: compare two implementations"
    success_criteria:
      - "Identifies efficiency differences (space/time)"
      - "Considers readability and maintainability"
      - "Justifies when to use each approach"
```

### Example 2: OOP Class Design (3-hour module)

**User Request**: "I'm teaching OOP fundamentals. What should students be able to do?"

**Generated Objectives**:

```yaml
topic: "Object-Oriented Programming Fundamentals"
objectives:
  - id: "LO-OOP-001"
    statement: "Define class, instance, method, and attribute and distinguish between them"
    blooms_level: "Remember"
    context: "Introduction to OOP vocabulary"
    prerequisites:
      - "Understand functions and variables in Python"
    assessment_method: "Matching exercise and short definitions"
    success_criteria:
      - "Can define each term correctly"
      - "Can match definitions to examples in code"

  - id: "LO-OOP-002"
    statement: "Explain how __init__ initializes instances and why self references the current object"
    blooms_level: "Understand"
    context: "Understanding object instantiation"
    prerequisites:
      - "Know class definition syntax"
      - "Understand function parameters and returns"
    assessment_method: "Walkthrough: explain code line-by-line"
    success_criteria:
      - "Can trace execution of __init__ method"
      - "Can explain what self represents"

  - id: "LO-OOP-003"
    statement: "Design and implement a class with attributes and methods for a specific domain"
    blooms_level: "Apply"
    context: "Real-world object requirements (e.g., Student, BankAccount, Car)"
    prerequisites:
      - "Can write class definitions with __init__"
      - "Understand instance vs class scope"
    assessment_method: "Code exercise: implement class from requirements"
    success_criteria:
      - "Class correctly models the domain"
      - "Attributes store state appropriately"
      - "Methods perform expected behaviors"
      - "Code follows PEP 8 naming conventions"

  - id: "LO-OOP-004"
    statement: "Analyze class hierarchies and identify inheritance relationships"
    blooms_level: "Analyze"
    context: "Understanding code organization and reuse patterns"
    prerequisites:
      - "Implement basic classes fluently"
      - "Understand inheritance syntax"
    assessment_method: "Code analysis exercise: identify class relationships"
    success_criteria:
      - "Identifies parent/child relationships correctly"
      - "Explains why inheritance is used"
      - "Identifies methods that are overridden"

  - id: "LO-OOP-005"
    statement: "Evaluate a class design and justify changes for maintainability and extensibility"
    blooms_level: "Evaluate"
    context: "Code review of existing class structures"
    prerequisites:
      - "Analyze inheritance patterns"
      - "Understand design principles (DRY, SOLID)"
    assessment_method: "Design critique: suggest improvements with justification"
    success_criteria:
      - "Identifies duplication that could be eliminated"
      - "Suggests appropriate inheritance changes"
      - "Justifies changes based on maintenance concerns"

  - id: "LO-OOP-006"
    statement: "Design an OOP solution for a complex domain problem"
    blooms_level: "Create"
    context: "Building a small system with multiple interacting classes"
    prerequisites:
      - "Master all previous OOP concepts"
      - "Can implement and evaluate class designs"
    assessment_method: "Create a mini-project with multiple classes"
    success_criteria:
      - "Solution correctly models the domain"
      - "Uses inheritance appropriately"
      - "Code is well-organized and reusable"
      - "Design decisions are documented and justified"
```

## Common Patterns

### Pattern 1: Short Tutorial (30 minutes)
- 1 Understand objective
- 1 Apply objective
- 1 Analyze objective
- ~Total: 3 objectives

### Pattern 2: Standard Lesson (1-2 hours)
- 1 Remember objective (if foundational topic)
- 1 Understand objective
- 2 Apply objectives (different contexts/complexities)
- 1 Analyze objective
- ~Total: 4-5 objectives

### Pattern 3: Full Unit (3-5 hours)
- 1 Remember (terminology)
- 1 Understand (concepts)
- 2-3 Apply (varied contexts)
- 1 Analyze (relationships)
- 1 Evaluate (quality)
- 1 Create (synthesis)
- ~Total: 7-8 objectives

### Pattern 4: Capstone/Project Course
- Lightweight foundational objectives (Remember/Understand)
- Multiple Apply objectives with increasing complexity
- Strong Analyze/Evaluate/Create focus
- Capstone project as final assessment

## Troubleshooting

### Objective Too Vague

**Problem**: "Understand decorators"

**Why it's bad**: Not measurable. What does "understand" mean? How will you know if they understand?

**Solution**: Use specific action verb and add context
- ✅ "Explain how a decorator wraps a function to modify its behavior"
- ✅ "Implement a custom decorator that logs function calls"

### Too Many Objectives

**Problem**: Created 15 objectives for 1-hour lesson

**Solution**: Reduce to 3-5 objectives. Ask:
- Which are essential? (Keep)
- Which are "nice to have"? (Remove or move to extension)
- Are some redundant? (Consolidate)

### Prerequisites Are Too Deep

**Problem**: Prerequisites include "Master Python" and "Understand all design patterns"

**Solution**: Be minimal. Ask: "What's the absolute minimum they need to know to start learning this?"
- Too deep: "Master all data structures"
- Just right: "Understand how lists work with indexing"

### Can't Think of Higher Bloom's Levels

**Problem**: Only created Remember and Apply objectives

**Solution**:
1. Read blooms-taxonomy-programming.md §4-6 for Analyze/Evaluate/Create examples
2. Ask: "What can learners do with this knowledge?" (Points toward Create)
3. Add analysis question: "Compare this approach with X..."
4. Add evaluation: "Would this solution work for Y? Why/why not?"

## Integration with Other Skills

This skill works well with:

**→ exercise-designer skill**: Once you have learning objectives, use that skill to create practice exercises aligned to each objective's Bloom's level

**→ technical-clarity skill**: Use to review the clarity of your objective statements

**→ code-example-generator skill**: Use to create examples that demonstrate Apply-level objectives

## Emerging Topics: Agentic AI & Model Context Protocol (MCP)

As books increasingly cover advanced AI topics, use this skill to define learning objectives for:

### Agentic AI Learning Objectives Example

**Topic**: "Building Autonomous AI Agents"

```yaml
topic: "Agentic AI Fundamentals"
objectives:
  - id: "LO-AGENT-001"
    statement: "Define autonomous agents, distinguish them from traditional chatbots, and explain key characteristics (goal-seeking, state management, tool use)"
    blooms_level: "Understand"
    context: "Introduction to agentic AI concepts"
    prerequisites:
      - "Understand AI basics and language models"
    assessment_method: "Short answer or concept mapping"
    success_criteria:
      - "Can list key differences between agents and chatbots"
      - "Can explain goal-directed behavior and autonomy"

  - id: "LO-AGENT-002"
    statement: "Implement a simple autonomous agent that uses tools to accomplish a specified goal"
    blooms_level: "Apply"
    context: "Building working agents with Python"
    prerequisites:
      - "Understand agent architecture and tool-use patterns"
      - "Can write Python functions and handle API calls"
    assessment_method: "Code exercise: implement agent from requirements"
    success_criteria:
      - "Agent autonomously reaches its goal"
      - "Correctly selects and uses available tools"
      - "Handles tool responses and iterates"

  - id: "LO-AGENT-003"
    statement: "Analyze agent behavior and identify when agents succeed, fail, or enter loops; propose fixes"
    blooms_level: "Analyze"
    context: "Debugging and improving agent performance"
    prerequisites:
      - "Can implement basic agents"
      - "Understand agent decision-making patterns"
    assessment_method: "Debug exercise: analyze agent logs and improve behavior"
    success_criteria:
      - "Identifies root causes of agent failures"
      - "Proposes specific improvements (better prompts, new tools, constraints)"
```

### MCP (Model Context Protocol) Learning Objectives Example

**Topic**: "Integrating MCP into Python Applications"

```yaml
topic: "Model Context Protocol (MCP) Integration"
objectives:
  - id: "LO-MCP-001"
    statement: "Explain MCP architecture, the role of servers and clients, and how MCP extends model capabilities"
    blooms_level: "Understand"
    context: "Introduction to MCP concepts"
    prerequisites:
      - "Understand Python modules and client-server patterns"
    assessment_method: "Explanation exercise: describe MCP client-server relationship"
    success_criteria:
      - "Can explain what MCP servers provide"
      - "Can describe the client-server protocol flow"

  - id: "LO-MCP-002"
    statement: "Install and configure existing MCP servers and integrate them into a Python application"
    blooms_level: "Apply"
    context: "Using MCP in real applications"
    prerequisites:
      - "Understand MCP architecture"
      - "Can write Python clients and manage dependencies"
    assessment_method: "Code exercise: integrate MCP server into working application"
    success_criteria:
      - "Correctly configures MCP client"
      - "Application successfully calls MCP server tools"
      - "Responses are properly handled and integrated"

  - id: "LO-MCP-003"
    statement: "Design and implement a custom MCP server that provides tools for a specific domain"
    blooms_level: "Create"
    context: "Building reusable MCP tools"
    prerequisites:
      - "Can integrate existing MCP servers"
      - "Understand server-side architecture and tool definitions"
    assessment_method: "Project: implement MCP server with documentation"
    success_criteria:
      - "Server correctly implements MCP protocol"
      - "Tools are well-defined with clear descriptions"
      - "Client can successfully call server tools"
      - "Code is documented and maintainable"
```

**Key Considerations for Advanced Topics**:
- These topics may require deeper prerequisites (solid Python, API design knowledge)
- Use spiral learning: revisit concepts at increasing depth (basic agent → advanced orchestration)
- Include both theory (understanding architecture) and practice (implementing tools)
- Assessment should emphasize real problem-solving, not just definitions

---

## Tips for Success

1. **Start with the end in mind**: What should learners be able to DO, not what will you teach?
2. **Make them testable**: Ask "How would I know if this objective was met?"
3. **Use consistent Bloom's level progression**: Build from simple to complex
4. **Include prerequisites**: Help learners self-assess readiness
5. **Validate often**: Use the validation script to catch vague language
6. **Test with real learners**: Objectives often need refinement based on actual usage

---

**Ready to generate objectives?** Provide:
- Topic/concept to teach
- Target learner level (beginner/intermediate/advanced)
- Available teaching time
- End goal (build project? understand theory? debug code?)

Or paste existing objectives and I'll help make them more measurable!
