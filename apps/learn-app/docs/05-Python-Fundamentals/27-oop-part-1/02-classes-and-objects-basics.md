---
title: "Classes and Objects Basics"
chapter: 24
lesson: 2
duration_minutes: 50

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Class Definition Syntax"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write basic class definitions with proper PascalCase naming and docstrings"

  - name: "Object Instantiation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create multiple objects from a single class and access their independent attributes"

  - name: "Self Keyword Understanding"
    proficiency_level: "B1"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain that self represents the current instance and why Python requires it explicitly"

  - name: "Constructor Implementation"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write __init__ methods with parameters and type hints"

  - name: "Type Hint Usage in Classes"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can add type hints to constructor parameters and method returns"

  - name: "Method Definition in Classes"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can write instance methods with self parameter and appropriate return types"

  - name: "Task Management Domain Application"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can apply class design patterns to real-world domains like task management"

learning_objectives:
  - objective: "Create a Python class with a constructor that initializes attributes with type hints"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student writes a working class definition"

  - objective: "Instantiate multiple objects from a class and verify they maintain independent state"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates objects, modifies them, and shows they don't affect each other"

  - objective: "Explain the role of self in instance methods and why Python requires it explicitly"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Student explains what self means in their own words"

  - objective: "Add methods to a class and call them on objects to modify state"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates methods that modify object state and demonstrate domain-specific behavior"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (class syntax, __init__, self, instantiation, attributes, methods, type hints) at B1 level. At maximum for B1 proficiency ✓"

differentiation:
  extension_for_advanced: "Extend Task class with @property decorators for computed attributes (e.g., days_until_due). Create a Task subclass (UrgentTask) with stricter priority rules. Implement multiple domain examples (Case, Invoice, Appointment) to show pattern transferability."
  remedial_for_struggling: "Start with empty class (pass) before adding constructor. Use lots of print statements to trace what self is. Show how task1.title and task2.title are different memory locations. Provide partially completed Task class to fill in."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/046-part5-todo-integration/reference/task-entity.md"
created: "2025-12-26"
last_modified: "2025-12-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "2.0.0"
---

# Classes and Objects Basics

In Lesson 1, you discovered WHY OOP exists. Now you'll learn the syntax for HOW to create classes and objects through hands-on experimentation with real-world domain examples, starting with Task management that you'll build throughout Part 5.

---

## Part 1: Experience Classes by Experimentation

**Your Role**: Code explorer discovering how Python implements objects

Before reading syntax explanations, you'll experiment with creating classes to develop intuition.

### Discovery Exercise: Build Classes Step by Step

**What is a class?** A class is a blueprint (template) for creating objects. Think of it like a task template: the class defines what properties a task can have, and objects are the individual tasks you create from it. The `class` keyword in Python defines this blueprint.

**Stage 1: The Simplest Possible Class**

Let's create the simplest possible class - a blueprint with no details yet.

Create a file called `task_experiments.py` and run this code:

```python
# The absolute minimum: a class with no content
class Task:
    pass

# Now create an object from this class
my_task = Task()

# What is my_task? Inspect it
print(type(my_task))
print(my_task)
print(dir(my_task))  # What attributes/methods does it have?
```

**Your task 1**: Run this and document:
- What does `type(my_task)` tell you?
- What does `print(my_task)` show you? (Pay attention to the memory address)
- What's in `dir(my_task)`? What default methods does Python give every object?

**Output:**
```
<class 'task_experiments.Task'>
<task_experiments.Task object at 0x7f8b8c0d5f90>
['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subattr__', '__subclasshook__', '__weakref__', '__dict__']
```

#### 💬 AI CoLearning Prompt

> "I created an empty class with just `pass`. When I inspect `dir(my_task)`, I see lots of methods like `__init__`, `__str__`, `__dict__`. Where do these come from? What's the relationship between a class and an object? Explain using a cookie-cutter analogy, then connect it to how a Task template and individual tasks work together."

**Expected Understanding**: AI will explain that classes are blueprints/templates (cookie cutter or task template), objects are instances created from them (cookies or individual tasks). Python gives every object default methods automatically. You'll understand the class-object relationship before coding more.

---

![Anatomy diagram showing Python class structure with constructor, attributes, methods, self parameter, and type hints](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-5/chapter-27/python-class-structure-anatomy.png)

**Stage 2: Add Data to Task Objects**

**Pedagogical Note**: In this stage, we'll show you a way to add data to objects that WORKS but isn't the professional pattern. We're doing this deliberately so you understand WHY the proper pattern (constructors with `__init__`) exists. Discovery through contrast is powerful.

Now modify the code to represent real tasks:

```python
class Task:
    pass

# Create first task and add properties manually
my_task = Task()
my_task.title = "Review pull request"
my_task.done = False
my_task.priority = 2

print(f"Task: {my_task.title}")
print(f"Done: {my_task.done}")
print(f"Priority: {my_task.priority}")

# Create a second task
important_task = Task()
important_task.title = "Deploy to production"
important_task.done = False
important_task.priority = 1

print(f"Task: {important_task.title}")
print(f"Priority: {important_task.priority}")

# Question: If I change my_task properties, does important_task change?
my_task.priority = 5
print(f"\nAfter changing my_task priority to 5:")
print(f"my_task priority: {my_task.priority}")
print(f"important_task priority: {important_task.priority}")
```

**Your task 2**: Run this and answer:
- How do you add data to an object in Python?
- Are the objects independent? Prove it.
- What happens if you inspect `my_task.__dict__`? (This shows all attributes)

**Output:**
```
Task: Review pull request
Done: False
Priority: 2
Task: Deploy to production
Priority: 1

After changing my_task priority to 5:
my_task priority: 5
important_task priority: 1
```

#### 💬 AI CoLearning Prompt

> "I'm adding attributes to Task objects using `my_task.title = 'Review PR'`. This works, but I have to repeat this for EVERY task I create. For 100 tasks, that's 300+ lines of attribute assignment! How do professional developers avoid this repetition? What Python feature lets me initialize task attributes automatically when creating a Task object? Give me a preview of the solution before I learn the syntax."

**Expected Understanding**: AI will preview the `__init__` method (constructor) that automatically runs when you create an object. You'll see WHY constructors exist before learning HOW they work. This motivates the learning in Part 2.

---

**Stage 3: Notice the Problem with Manual Attributes**

You've seen that adding attributes manually works. Now let's see why it's problematic at scale.

```python
class Task:
    pass

# This works, but is repetitive and error-prone
task1 = Task()
task1.title = "Review PR"
task1.done = False
task1.priority = 2

task2 = Task()
task2.title = "Deploy to production"
task2.done = False
task2.priority = 1

task3 = Task()
task3.title = "Fix bug in auth module"
task3.done = True
task3.priority = 3

# What if you create 100 tasks? You'd repeat setup code 100 times
# What if you forget to set priority for a task? (Silent error - no type safety)
# How do you FORCE every Task object to have title: str, done: bool, priority: int?
```

**Your task 3**: Before reading further, predict:
- What would solve the repetition problem?
- How could you ensure every Task object MUST have title: str, done: bool, priority: int?
- What language feature would enforce this at object creation time?

**Answer Preview**: The solution is the constructor (`__init__` method) with type hints. You'll learn this in the next section. This discovery exercise showed you the PAIN that constructors solve.

---

### Your Discovery Summary

Instead of manual documentation, **use AI to synthesize** what you learned:

#### 💬 AI CoLearning Prompt

> "Based on my experiments with creating Task classes and adding attributes manually, summarize these key insights:
> 1. What's the relationship between classes and objects? (blueprint vs instance)
> 2. How are Task objects independent? (each has its own memory)
> 3. What's the pain point with manual attribute assignment? (repetition, no enforcement)
> 4. What's the solution I'm about to learn? (constructors with __init__)
>
> Give me 3 concise bullet points I can reference in Part 2."

**Deliverable**: Save AI's synthesis in your notes. You've discovered the problem—now you're ready to learn the solution (`__init__` method) in Part 2.

---

## Part 2: Learn Class Syntax and Constructors with Task Methods

**Your Role**: Student receiving instruction from AI Teacher

Now that you've experienced the limitations of manual attribute setting, it's time to learn the solution using real Task management patterns.

![Anatomical breakdown of Python class structure showing class definition, __init__ constructor, instance attributes, instance methods, and the role of self parameter in connecting methods to object instances](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-5/chapter-27/python-class-structure-anatomy.png)

### AI Teaching Prompt

Ask your AI companion:

> "I've been creating Task objects and manually adding title, done, priority attributes to each one. This is repetitive:
> ```python
> task1 = Task()
> task1.title = 'Review PR'
> task1.done = False
> task1.priority = 2
>
> task2 = Task()
> task2.title = 'Deploy to production'
> # ... repeat for every task ...
> ```
>
> How can I force every Task to automatically initialize with title, done, priority when created? Show me the syntax and explain:
> 1. What is the `__init__` method?
> 2. What is `self`?
> 3. Why does Python require explicit `self` parameter?
> 4. Show me the same task-creating code but using __init__"

### What You'll Learn from AI

**Expected AI Response** (summary):

- **`__init__` method**: A special constructor that Python automatically calls when you create an object
- **`self`**: Represents "the object being created." It's always the first parameter
- **`self.title = title`**: Creates an attribute on THIS object
- **Why explicit self?**: Python wants you to be clear: you're setting attributes on a specific object

**AI will show you**:

```python
class Task:
    def __init__(self, title: str, done: bool = False, priority: int = 5):
        self.title = title      # "Set THIS task's title"
        self.done = done        # "Set THIS task's done status"
        self.priority = priority  # "Set THIS task's priority"

# Now creation is much simpler!
task1 = Task("Review pull request", priority=2)
task2 = Task("Deploy to production", priority=1)

print(task1.title)  # Review pull request
print(task2.title)  # Deploy to production
print(task1.priority)  # 2
print(task2.priority)  # 1
```

**Output:**
```
Review pull request
Deploy to production
2
1
```

### Convergence Activity

After AI explains, **test your understanding**:

Ask AI: "Walk me through what happens in memory when I call `Task('Review PR', priority=2)`. Step by step: Python creates the object, then calls `__init__` with what self value? Show me how Python internally handles the 'self' parameter."

**Deliverable**: Write a summary explaining `__init__`, `self`, and why this solves the repetition problem compared to manual attribute setting.

---

## Part 3: Design Task Methods for State Changes

**Your Role**: Student designing behavior through instance methods

Now that you understand how to initialize Task objects, let's add methods that represent real task operations.

### Working with Task Methods

Real tasks need to DO things. Let's design methods that represent task operations:

```python
class Task:
    def __init__(self, title: str, done: bool = False, priority: int = 5):
        self.title = title
        self.done = done
        self.priority = priority

    def mark_complete(self) -> None:
        """Mark the task as done."""
        self.done = True
        print(f"Task '{self.title}' marked as complete")

    def update_priority(self, new_priority: int) -> None:
        """Change task priority (1=highest, 10=lowest)."""
        if 1 <= new_priority <= 10:
            self.priority = new_priority
            print(f"Priority updated to {new_priority}")
        else:
            print("Priority must be between 1 and 10")

    def get_status(self) -> str:
        """Return current task status."""
        status = "✓ Complete" if self.done else "○ Pending"
        return f"{self.title} [{status}] - Priority: {self.priority}"

# Use the Task class with methods
task1 = Task("Review pull request", priority=2)
print(task1.get_status())

task1.update_priority(1)
print(task1.get_status())

task1.mark_complete()
print(task1.get_status())
```

**Output:**
```
Review pull request [○ Pending] - Priority: 2
Priority updated to 1
Review pull request [✓ Complete] - Priority: 1
Task 'Review pull request' marked as complete
```

#### 💬 AI CoLearning Prompt

> "I've defined Task methods like `mark_complete()` and `update_priority()`. When I call `task1.mark_complete()`, how does Python know to modify task1's attributes, not task2's? Walk me through the method call: Python sees `task1.mark_complete()`, internally it must do something like `Task.mark_complete(task1)`. Explain how the self parameter makes this work automatically without me typing the object name."

---

### Challenge: Object Independence with Methods

**Your prompt to AI**:

> "I create two Task objects and modify one:
> ```python
> task1 = Task('Review PR', priority=2)
> task2 = Task('Deploy to production', priority=1)
> task1.mark_complete()
> task1.update_priority(5)
> ```
>
> After this, what are:
> - `task1.done`? `task1.priority`?
> - `task2.done`? `task2.priority`?
>
> Why didn't task2 change when I modified task1? Explain what's happening in memory that makes them separate."

**Expected learning**: AI will explain that each object has its own memory space for attributes. This is the fundamental power of objects. Methods operate on specific instances.

**Output:**
```
task1.done = True
task1.priority = 5
task2.done = False
task2.priority = 1
# task2 is independent!
```

---

## Part 4: Pattern Transfer Across Domains

**Your Role**: Knowledge synthesizer discovering pattern universality

You've learned classes using Task. Now you'll see that these SAME PATTERNS apply to completely different domains.

### Domain Transfer: Legal, Finance, Healthcare

The Task class pattern appears everywhere. Here's how it transfers:

#### Domain 1: Legal Case Management

```python
class Case:
    def __init__(self, case_number: str, title: str, client_name: str):
        self.case_number = case_number
        self.title = title
        self.client_name = client_name
        self.status = "open"

    def close_case(self) -> None:
        """Close the case."""
        self.status = "closed"
        print(f"Case {self.case_number} closed")

    def assign_attorney(self, attorney_name: str) -> None:
        """Assign attorney to case."""
        self.attorney = attorney_name
        print(f"Attorney {attorney_name} assigned to {self.case_number}")

# Usage
case1 = Case("2025-CV-1234", "Smith v. Jones", "Smith Corp")
print(f"Case: {case1.title} - Status: {case1.status}")
case1.close_case()
```

**Output:**
```
Case: Smith v. Jones - Status: open
Case 2025-CV-1234 closed
```

#### Domain 2: Finance Invoice Management

```python
class Invoice:
    def __init__(self, invoice_id: str, client_name: str, amount: float):
        self.invoice_id = invoice_id
        self.client_name = client_name
        self.amount = amount
        self.is_paid = False

    def mark_paid(self) -> None:
        """Mark invoice as paid."""
        self.is_paid = True
        print(f"Invoice {self.invoice_id} marked as paid")

    def apply_discount(self, discount_percent: float) -> None:
        """Apply discount to invoice amount."""
        discount = self.amount * (discount_percent / 100)
        self.amount -= discount
        print(f"Discount applied: ${discount:.2f}")

# Usage
invoice1 = Invoice("INV-2025-001", "Acme Corp", 5000.00)
print(f"Invoice {invoice1.invoice_id}: ${invoice1.amount}")
invoice1.apply_discount(10)
print(f"After discount: ${invoice1.amount}")
```

**Output:**
```
Invoice INV-2025-001: $5000.0
Discount applied: $500.00
After discount: $4500.0
```

#### Domain 3: Healthcare Appointment Scheduling

```python
class Appointment:
    def __init__(self, appointment_id: str, patient_name: str, doctor_name: str):
        self.appointment_id = appointment_id
        self.patient_name = patient_name
        self.doctor_name = doctor_name
        self.is_confirmed = False

    def confirm(self) -> None:
        """Confirm the appointment."""
        self.is_confirmed = True
        print(f"Appointment {self.appointment_id} confirmed")

    def reschedule(self, new_doctor: str) -> None:
        """Reschedule with different doctor."""
        self.doctor_name = new_doctor
        self.is_confirmed = False
        print(f"Rescheduled with Dr. {new_doctor}")

# Usage
apt1 = Appointment("APT-2025-0001", "John Doe", "Dr. Smith")
print(f"Appointment with Dr. {apt1.doctor_name}")
apt1.confirm()
apt1.reschedule("Dr. Johnson")
```

**Output:**
```
Appointment with Dr. Smith
Appointment APT-2025-0001 confirmed
Rescheduled with Dr. Johnson
```

### Pattern Comparison Table

| Pattern | Task | Case | Invoice | Appointment |
|---------|------|------|---------|-------------|
| Attributes | title, done, priority | case_number, status, client | invoice_id, amount, is_paid | appointment_id, is_confirmed |
| Constructor | `__init__(title, priority)` | `__init__(case_number, title)` | `__init__(invoice_id, amount)` | `__init__(appointment_id, patient)` |
| Key method | `mark_complete()` | `close_case()` | `mark_paid()` | `confirm()` |
| Status change | done → True | status → "closed" | is_paid → True | is_confirmed → True |

**Key insight**: The PATTERN is identical across all domains. Only the data and method names change. You're not learning Task-specific syntax—you're learning a universal Python pattern.

#### 💬 AI CoLearning Prompt

> "I've now seen Task, Case, Invoice, and Appointment classes. They all follow the same pattern:
> - Constructor initializes attributes
> - Methods modify that state
> - Each object maintains independent state
>
> What's the universal pattern here? If I encounter a completely new domain (e.g., Product inventory, Restaurant reservation, Library book), how would I design a class using what I've learned?"

**Deliverable**: Write a summary showing how the Task pattern applies universally.

---

## Part 5: Build Your Class Design Patterns Reference

**Your Role**: Knowledge synthesizer creating design templates

Now integrate everything into reusable patterns for designing classes in any domain.

### Your Class Design Template

Create a markdown file called `class_design_patterns.md` with this structure:

```markdown
# Class Design Patterns

## Pattern 1: Simple Data Container with State Methods

**When to use**: Class representing a real-world entity with state and operations

**Template**:
```python
class Entity:
    def __init__(self, required_attr: str, optional_attr: str = "default"):
        self.required_attr = required_attr
        self.optional_attr = optional_attr
        self.status = "initial_state"

    def perform_action(self) -> None:
        """Describe what this method does."""
        self.status = "new_state"
```

**Key points**:
- Constructor parameters match required attributes
- Type hints on parameters
- Status attributes track entity state
- Methods change state through self.attribute assignments

---

## Pattern 2: Methods with Parameters

**When to use**: Methods that modify state based on input

**Template**:
```python
class Task:
    def __init__(self, title: str, priority: int = 5):
        self.title = title
        self.priority = priority

    def update_priority(self, new_priority: int) -> None:
        """Update priority with validation."""
        if 1 <= new_priority <= 10:
            self.priority = new_priority
        else:
            raise ValueError("Priority must be 1-10")
```

**Key points**:
- Parameters enable method flexibility
- Validation happens in methods
- Methods can raise exceptions for invalid input
- Always use type hints

---

## Pattern 3: Methods with Return Values

**When to use**: Methods that compute or retrieve data

**Template**:
```python
class Invoice:
    def __init__(self, amount: float):
        self.amount = amount
        self.is_paid = False

    def remaining_balance(self) -> float:
        """Return amount still owed."""
        return self.amount if not self.is_paid else 0.0
```

**Key points**:
- Use return type hints (-> type)
- Methods should return data, not just print
- Callers can use returned values

---

## Pattern 4: Object Independence Verification

**How to verify objects are truly independent**:

```python
# Create two objects
task1 = Task("Review PR", priority=2)
task2 = Task("Deploy", priority=1)

# Modify one
task1.update_priority(5)

# Verify other is unchanged
assert task1.priority == 5
assert task2.priority == 1  # ✓ task2 is independent

print("Objects are independent!")
```

**Why this matters**: Confirms that each object has its own memory space.

---

## Pattern 5: The Self Parameter Explained

**Memory model**:
```python
class Task:
    def __init__(self, title: str):
        self.title = title  # "THIS task's title"

    def get_title(self) -> str:
        return self.title  # "THIS task's title"

task1 = Task("Review PR")
task2 = Task("Deploy")

# When you call task1.get_title():
# Python mentally does: Task.get_title(task1)
# "self" inside the method refers to task1
```

**Key insight**: `self` = "the specific object this method is operating on"

---

## Common Mistakes and Fixes

### Mistake 1: Forgetting `self` in methods
```python
# ❌ WRONG
def mark_complete():
    done = True

# ✅ CORRECT
def mark_complete(self):
    self.done = True
```

### Mistake 2: Not calling constructor
```python
# ❌ WRONG
task = Task  # Missing ()

# ✅ CORRECT
task = Task("Review PR")
```

### Mistake 3: Forgetting `self.` when accessing attributes
```python
# ❌ WRONG
def get_info(self):
    return title  # Where does 'title' come from?

# ✅ CORRECT
def get_info(self):
    return self.title  # THIS object's title
```

---

## Testing Pattern: Verify Independence

**Always test that objects are independent**:

```python
task1 = Task("Review PR", priority=2)
task2 = Task("Deploy", priority=1)

task1.update_priority(5)

assert task1.priority == 5
assert task2.priority == 1  # ← Unchanged!
```
```

### Validation with AI

Once your patterns are complete, validate them:

> "Review my class design patterns. Are my explanations of self and object independence accurate? What patterns am I missing? What common mistakes should I add?"

**Deliverable**: Complete `class_design_patterns.md` with patterns and quick reference guide.

---

## Try With AI

Ready to master class syntax and understand how objects manage independent state?

**🔍 Explore Task Methods:**
> "Show me a Task class with title, priority, and due_date attributes. Add methods: mark_complete(), delay_deadline(days), and get_status(). Create three task objects with different priorities. When I call task1.mark_complete(), show step-by-step what happens in memory. How does Python know to modify only task1 and not task2?"

**🎯 Practice Domain Transfer:**
> "I'm building [describe your system: legal case management, invoice processing, appointment scheduling, etc.]. Help me design one core entity class. Walk me through: what attributes describe it? What methods represent key operations? How would I initialize multiple objects and modify them independently? Show code with test examples."

**🧪 Test Object Independence:**
> "Create a Case class with case_number, status, and client_name attributes. Add methods: close_case() and assign_attorney(attorney). Make three case objects. Close the first case and change its attorney. Show that the other two cases are unaffected. Why are they independent?"

**🚀 Apply Pattern Across Domains:**
> "I've learned Task classes with methods. Show me how the same pattern applies to: Invoice (with mark_paid() and apply_discount()), Appointment (with confirm() and reschedule()), and Product (with adjust_price() and restock()). How is the pattern universal across domains?"

---
