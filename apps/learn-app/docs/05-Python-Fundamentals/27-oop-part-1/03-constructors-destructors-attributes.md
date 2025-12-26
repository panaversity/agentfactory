---
title: "Constructors, Destructors, and Attributes"
chapter: 24
lesson: 3
duration_minutes: 55

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Parameterized Constructors"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write constructors with multiple parameters and type hints"

  - name: "Default Parameters in Constructors"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can provide default values in __init__ methods and use them appropriately"

  - name: "Class vs Instance Attributes"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose appropriate attribute scope and explain memory implications"

  - name: "Destructor Usage (__del__)"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement __del__ for resource cleanup and understand limitations"

  - name: "Attribute Inspection (__dict__)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use __dict__ for debugging and understanding attribute storage"

learning_objectives:
  - objective: "Write constructors with multiple parameters and default values"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code implementation of parameterized constructors"

  - objective: "Distinguish between class attributes (shared) and instance attributes (unique)"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Explanation of attribute scoping and memory implications"

  - objective: "Implement destructors for proper resource cleanup"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code implementation of __del__ methods"

  - objective: "Use attribute inspection to debug object state"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Debugging exercises using __dict__"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (default parameters, class attributes, instance attributes, __dict__, destructors, resource management, attribute shadowing, attribute naming) - within B1-B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Design a DatabaseConnection class with connection pooling; implement multiple cleanup strategies; compare destructor vs context manager approaches"
  remedial_for_struggling: "Focus on simple BankAccount example first; use diagrams to show memory layout of class vs instance attributes; practice __dict__ inspection before moving to destructors"

# Generation metadata
generated_by: "content-implementer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-26.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Constructors, Destructors, and Attributes

Now that you can create basic classes, you'll dive deeper into sophisticated patterns: constructors with flexible parameters, the critical distinction between class and instance attributes, and object cleanup with destructors.

In this lesson, you'll discover why these patterns matter through hands-on experiments, learn from AI explanations, challenge the AI with subtle edge cases, and build a comprehensive reference guide.

---

## Part 1: Experience Default Parameters and Attribute Types

**Your Role**: Code experimenter discovering initialization patterns and attribute scoping

### Discovery Exercise 1: Why Default Parameters Matter

Create `task_initialization.py` and run this:

```python
from datetime import datetime

class Task:
    def __init__(self, title: str, description: str, priority: int, due_date):
        self.title = title
        self.description = description
        self.priority = priority
        self.due_date = due_date

# Works fine, but requires ALL parameters
task1 = Task("Buy groceries", "Fresh vegetables", 3, datetime(2025, 1, 15))

# What if you don't know the due date? Forced to use placeholder
task2 = Task("Review PR", "Code review", 1, None)  # Awkward!
```

**Output:**
```
Task created: Buy groceries
Task created: Review PR (no deadline)
```

Now add default parameters:

```python
from datetime import datetime

class Task:
    def __init__(
        self,
        title: str,
        description: str = "",
        priority: int = 5,
        due_date = None
    ):
        self.title = title
        self.description = description
        self.priority = priority
        self.due_date = due_date
        self.status = "pending"
        self.done = False
        self.created_at = datetime.now()

# Now flexible!
task1 = Task("Buy groceries", "Fresh vegetables", 3, datetime(2025, 1, 15))
task2 = Task("Review PR")  # Uses defaults: description="", priority=5, due_date=None
task3 = Task("Write docs", "API documentation")  # description provided, others default
```

**Output:**
```
task1.title = "Buy groceries"
task1.priority = 3
task2.title = "Review PR"
task2.priority = 5 (default)
task2.description = "" (default)
```

#### 💬 AI CoLearning Prompt

> "I added default parameters to my Task class. Required: title. Optional: description (default empty string), priority (default 5), due_date (default None). Now I can create tasks flexibly. But when should parameters be required vs have defaults? Show me 3 examples: 1) user registration (which fields MUST be provided?), 2) database connection (which have sensible defaults?), 3) API client config. Explain the design principle."

**Expected Understanding**: AI will explain that **critical data should be required** (no default), **convenience data can have defaults**. You'll see the tradeoff between flexibility and enforced completeness. Task.title is always required because every task must have a name; due_date is optional because you can create urgent immediate tasks.

---

### Discovery Exercise 2: Instance vs Class Attributes

Run this experiment:

```python
class Task:
    status_options = ["pending", "in_progress", "completed"]  # Class attribute - ALL tasks share this
    default_priority = 5  # Class attribute - shared default

    def __init__(self, title: str):
        self.title = title           # Instance attribute - each task's own
        self.priority = self.default_priority  # Instance attribute - each task's own
        self.done = False            # Instance attribute - each task's own

task1 = Task("Buy groceries")
task2 = Task("Review PR")

# Change class attribute (affects all Task instances)
Task.default_priority = 3
print(task1.priority)  # Still 5 (already assigned during __init__)
print(task2.priority)  # Still 5 (already assigned during __init__)

# But for new tasks:
task3 = Task("Write docs")
print(task3.priority)  # Still 5 (because default_priority is 5, not updated retroactively)

# Change instance attribute (affects only one task)
task1.priority = 1
print(task2.priority)  # Still 5 (independent)
```

**Output:**
```
task1.priority = 5
task2.priority = 5
task1.priority = 1 (changed only for this task)
task2.priority = 5 (unaffected)
```

#### 💬 AI CoLearning Prompt

> "I have a Task class with default_priority as a class attribute and priority as an instance attribute. When I change task1.priority = 1, only task1 is affected. But status_options is a class attribute shared by all tasks. Explain:
> 1. What's stored in memory differently for class vs instance attributes?
> 2. Give me 3 real-world examples where class attributes make sense (like API base_url, database config, valid status values)
> 3. When should I avoid class attributes? (Hint: think about mutable defaults like lists)"

**Expected Understanding**: AI will explain that class attributes are shared memory (one copy for all instances), instance attributes are per-object memory (each object gets its own copy). Use class attributes for configuration, constants, and shared validation rules. Avoid mutable class attributes like lists or dicts that should be independent.

---

### Discovery Exercise 3: The Shadowing Trap

Run this and observe strange behavior:

```python
class Task:
    priority_default = 5  # Class attribute

    def __init__(self, title: str):
        self.title = title

t1 = Task("Task 1")
t2 = Task("Task 2")

print(t1.priority_default)  # 5 (reading class attribute)
print(t2.priority_default)  # 5 (reading class attribute)

# Now the trap:
t1.priority_default = 1  # Creates INSTANCE attribute (shadowing the class attribute!)

print(t1.priority_default)       # 1 (instance attribute)
print(t2.priority_default)       # 5 (still class attribute)
print(Task.priority_default)     # 5 (class attribute unchanged)

# Check where the attribute lives:
print("priority_default" in t1.__dict__)    # True (it's in t1's instance dict)
print("priority_default" in t2.__dict__)    # False (t2 doesn't have it, reads from class)
print("priority_default" in Task.__dict__)  # True (it's in the class)
```

**Output:**
```
t1.priority_default = 1 (instance attribute shadows class attribute)
t2.priority_default = 5 (still reading from class)
Task.priority_default = 5 (class attribute unchanged)
```

#### 💬 AI CoLearning Prompt

> "I tried to update a class attribute through an instance (t1.priority_default = 1) but it created an INSTANCE attribute instead, shadowing the class attribute! Now t1 and t2 have different values!
> 1. Why does Python allow this? Is it a bug or a feature?
> 2. How do I update class attributes correctly? (Hint: use ClassName.attribute, not instance.attribute)
> 3. How can I detect if an attribute lives in an instance or the class using __dict__?"

**Expected Understanding**: AI will explain that `obj.attr = value` ALWAYS creates an instance attribute. To modify class attributes, use `ClassName.attr = value`. Shadowing is intentional but often confusing. Use `__dict__` to inspect which attributes belong to which object.

---

### Your Discovery Summary

Instead of manual files, **use AI to synthesize**:

#### 💬 AI CoLearning Prompt

> "Based on my experiments with default parameters, instance/class attributes, and shadowing, summarize these key insights:
> 1. When should constructor parameters be required vs have defaults?
> 2. When should data be instance attributes vs class attributes?
> 3. What's the shadowing trap and how do I avoid it?
>
> Give me 3 bullet points for my reference guide."

**Deliverable**: Save AI's synthesis in your notes. You've discovered constructor design patterns—now you're ready to learn advanced techniques.

---

## Part 2: Learn Advanced Constructor Patterns

**Your Role**: Student receiving instruction from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I'm building a Task management system and I've discovered two types of attributes:
> - Instance attributes like `self.title` and `self.priority` (each task has its own)
> - Class attributes like `status_options = ["pending", "in_progress", "completed"]` (shared across all tasks)
>
> Explain:
> 1. When should I use default parameters in Task.__init__?
> 2. What's the memory layout difference between instance and class attributes?
> 3. If task1.priority = 1 creates a new instance attribute, what happens to the class default?
> 4. Show me a Task example where class attributes are useful (configuration, validation rules, etc)."

### What You'll Learn from AI

**Expected AI Response** (summary):

- **Default parameters**: Make constructors flexible for optional task data (description, due_date)
- **Instance attributes**: Each task has independent values (title, priority, done)
- **Class attributes**: All tasks share configuration (status_options, default_priority)
- **Shadowing**: Creating instance attribute hides class attribute temporarily
- **Real use case**: Task.status_options (shared validation) vs task.priority (unique per task)

### Convergence Activity

After AI explains, ask:

> "Walk me through what happens in memory when I create `task = Task('Review PR')` in a class that has both `default_priority = 5` (class attribute) and `self.priority = default_priority` (instance attribute in __init__)."

**Deliverable**: Write a summary explaining when to use default parameters, class vs instance attributes in your Task class, and why shadowing is important.

---

## Part 3: Challenge AI with Edge Cases

**Your Role**: Student teaching AI by testing edge case understanding

#### Challenge 1: Default Parameters and Mutability

**Your prompt to AI**:

> "I'm designing a Task class and I want to add an optional tags parameter: `def __init__(self, title: str, tags: list = []):`. But I see warnings online that this is dangerous. Why is using a mutable default parameter (list, dict) problematic? Show me an example where this goes wrong with my Task class and explain what's happening."

**Expected learning**: AI will explain that mutable defaults are shared across all Task instances, causing hidden bugs where all tasks unexpectedly share the same tags list.

#### Challenge 2: Class Attributes in Task Configuration

**Your prompt to AI**:

> "I'm designing a Task class. Configuration like `status_options = ["pending", "in_progress", "completed"]` could be:
> - A class attribute (all tasks use same valid statuses)
> - An instance attribute in __init__ (each task could have different statuses)
>
> For these scenarios, which should it be and why:
> 1. All tasks in the system must use the same status values
> 2. Each task might have its own custom status workflow
> 3. The priority levels (1-10 scale) for all tasks
> 4. Timestamps when each task was created
>
> Explain your reasoning for each."

**Expected learning**: AI will explain design decisions about when to share data across tasks vs. keeping data independent per task.

#### Challenge 3: Attribute Shadowing with Tasks

**Your prompt to AI**:

> "If I create a TaskCategory subclass:
> ```python
> class Task:
>     status = 'pending'  # Class attribute
>
> class UrgentTask(Task):
>     pass
>
> ut = UrgentTask()
> ut.status = 'in_progress'
> ```
>
> Does this create an instance attribute on ut that shadows Task.status? Or does it modify the class attribute? Show me how to verify which happened using __dict__."

### Deliverable

Document your three challenges and AI's responses with your analysis of correctness.

---

## Part 4: Build Your Attributes and Constructors Reference

**Your Role**: Knowledge synthesizer creating design patterns

Create `task_constructors_guide.md` with patterns across multiple domains:

```markdown
# Constructors with Defaults and Attributes Guide

## Pattern 1: Task Constructor with Defaults (Primary Domain)

**When to use**: Optional parameters for flexible object creation

```python
from datetime import datetime

class Task:
    def __init__(
        self,
        title: str,
        description: str = "",
        priority: int = 5,
        due_date = None
    ):
        self.title = title
        self.description = description
        self.priority = priority
        self.due_date = due_date
        self.status = "pending"
        self.done = False
        self.created_at = datetime.now()

# All these work
Task("Buy groceries", "Fresh vegetables", 3, datetime(2025, 1, 15))
Task("Review PR")  # Uses defaults
Task("Write docs", "API documentation")  # Partial defaults
```

**Key principle**: Required parameters first (title), optional last (due_date)

---

## Pattern 2: Class Attributes for Task Configuration

**Class attributes** (shared across all tasks):
```python
class Task:
    status_options = ["pending", "in_progress", "completed"]
    default_priority = 5
    priority_range = (1, 10)  # Min, max valid values

    def __init__(self, title: str):
        self.title = title
        self.priority = self.default_priority
```

**Rule**: If data is configuration, validation rules, or constants → class attribute. If data is per-task → instance attribute.

---

## Pattern 3: Avoiding Mutable Default Parameters

```python
# ❌ WRONG - Mutable default shared between instances!
class Task:
    def __init__(self, title: str, tags: list = []):
        self.title = title
        self.tags = tags  # All tasks share same list!

task1 = Task("Task 1")
task2 = Task("Task 2")
task1.tags.append("urgent")
print(task2.tags)  # ["urgent"] - SHARED! ❌

# ✅ CORRECT - Use None, create new list inside
class Task:
    def __init__(self, title: str, tags: list = None):
        self.title = title
        self.tags = tags if tags is not None else []  # Each task gets own list
```

---

## Pattern 4: Updating Class Attributes Correctly

```python
class Task:
    default_priority = 5

    def __init__(self, title: str):
        self.title = title
        self.priority = self.default_priority

task1 = Task("Task 1")
task2 = Task("Task 2")

# ✅ CORRECT - Update class attribute through class name
Task.default_priority = 3  # New tasks will use priority 3

# ❌ WRONG - This creates instance attribute, doesn't modify class!
# task1.default_priority = 3  # This shadows the class attribute
```

---

## Pattern 5: Detecting Attribute Type with __dict__

```python
task = Task("Review PR")
task.priority = 1  # Instance attribute

# Check where attribute lives
print("priority" in task.__dict__)      # True = instance attribute
print("priority" in Task.__dict__)      # False = not a class attribute
print("status_options" in Task.__dict__)  # True = class attribute
```

---

## Pattern 6: Transfer to Auxiliary Domains

The same patterns work everywhere:

```python
# Legal: Case with defaults
class Case:
    case_statuses = ["open", "pending", "discovery", "trial", "closed"]

    def __init__(self, case_number: str, title: str, client_name: str, case_type: str = "civil"):
        self.case_number = case_number
        self.title = title
        self.client_name = client_name
        self.case_type = case_type
        self.status = "open"

# Finance: Invoice with defaults
class Invoice:
    status_options = ["draft", "pending", "paid", "overdue"]

    def __init__(self, invoice_id: str, client_name: str, amount: float, description: str = ""):
        self.invoice_id = invoice_id
        self.client_name = client_name
        self.amount = amount
        self.description = description
        self.status = "pending"
        self.is_paid = False

# Healthcare: Appointment with defaults
class Appointment:
    types = ["checkup", "followup", "specialist", "emergency"]

    def __init__(self, appointment_id: str, patient_name: str, doctor_name: str, appointment_type: str = "checkup"):
        self.appointment_id = appointment_id
        self.patient_name = patient_name
        self.doctor_name = doctor_name
        self.appointment_type = appointment_type
        self.status = "scheduled"
        self.is_confirmed = False
```

**Key insight**: Every domain follows the same constructor patterns. Task, Case, Invoice, Appointment all use:
- Required parameters (task title = case number = invoice id)
- Optional parameters with defaults (due_date, case_type, description)
- Class attributes for shared configuration (status_options)
- Instance attributes for per-object data (priority, status, done)

---

## Constructor Pattern Checklist

When designing ANY constructor, ask:

1. **What parameters are required?** (must provide)
2. **What parameters are optional?** (have sensible defaults)
3. **Should this be a class attribute?** (shared configuration/validation)
4. **Should this be an instance attribute?** (specific to each object)
5. **Am I using mutable defaults?** (use None instead)

### Validation with AI

> "Review my Task constructor and compare it to the Case and Invoice patterns. Are my decisions about required vs optional parameters consistent across domains? What patterns am I missing?"

**Deliverable**: Complete guide with Task pattern and at least 2 auxiliary domain patterns.
```

---

## Try With AI

Ready to master Task constructors with defaults, understand attribute scoping, and avoid mutable default traps?

**🔍 Explore Task Default Parameters:**
> "Show me a Task class with required title parameter and optional description (default=''), priority (default=5), due_date (default=None). Create tasks with: all params, only title, title+priority. Explain when defaults help vs when they hide required data. For a task management system, should all attributes be optional except title?"

**🎯 Practice Instance vs Class Attributes in Tasks:**
> "Build a Task class where: status_options is shared by all tasks (class attribute), but priority is per-task (instance attribute). Create multiple tasks, change Task.status_options on the class, show what happens. When should attributes be shared (like valid statuses) vs independent (like each task's priority)?"

**🧪 Test Mutable Default Danger with Tasks:**
> "Create a Task class with tags parameter defaulting to []. Add tags to task1. Create task2 without tags. What's in task2.tags? Why? Show me the fix (tags=None, then self.tags = tags or []). Explain why this Python gotcha matters for task management."

**🚀 Apply to Other Domains:**
> "Compare Task, Case, and Invoice constructors. They all need required ID-like fields and optional descriptive fields. Design Case.__init__(case_number, title, client_name, case_type='civil') and Invoice.__init__(invoice_id, client_name, amount, description=''). What patterns are the same? When does a field MUST be required vs can have defaults?"

---
