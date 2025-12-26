---
sidebar_position: 4
title: "Encapsulation and Method Types"
chapter: 27
lesson: 4
duration_minutes: 60

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Access Control Patterns"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose appropriate access levels (public/protected/private) for attributes in design scenarios"

  - name: "Instance Method Design"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement methods operating on instance state (self) for various use cases"

  - name: "Class Method Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement @classmethod for factory patterns and class-level operations like Task.create_from_dict()"

  - name: "Static Method Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement @staticmethod for utility functions like Task.is_valid_priority() within class scope"

  - name: "Property Decorator Usage"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement getters, setters, and computed properties using @property decorator"

  - name: "Data Validation"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can validate attribute access through properties and setters"

  - name: "Method Type Selection"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose between instance methods, class methods, and static methods based on context"

  - name: "Encapsulation Through Properties"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design encapsulation strategies using properties and naming conventions"

  - name: "Getter/Setter Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can implement validation logic in property setters"

  - name: "AI-Assisted API Design"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Analyze"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can describe API design choices to AI and refine implementation together"

learning_objectives:
  - objective: "Evaluate access control patterns and choose appropriate visibility levels (public/protected/private) for attributes"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Design review exercise choosing access levels for real-world scenarios"

  - objective: "Implement all three method types (instance, class, static) using Task as primary example and explain when each is appropriate"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Implementation exercise with all three method types in Task and auxiliary classes"

  - objective: "Apply property decorators to implement Pythonic getters, setters, and computed attributes"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Property implementation with validation and computed properties"

  - objective: "Design encapsulation strategies using properties for data protection and validation across multiple domains"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Design exercise implementing validated Task, Case, Invoice, and Appointment classes"

cognitive_load:
  new_concepts: 10
  assessment: "10 concepts (public/protected/private, instance methods, @classmethod, @staticmethod, @property, @setter, computed properties, data validation, encapsulation strategy, inheritance hierarchy) at B2 maximum. At limit ✓"

differentiation:
  extension_for_advanced: "Research Python's @abstractmethod with @property. Design an abstract DataStore with property-based interface. Explore Protocol typing (PEP 544) as alternative to ABC for duck typing enforcement. Implement factory chains: Task.create_from_dict() that calls other class methods."
  remedial_for_struggling: "Focus on one method type at a time. Start with instance methods (mark_complete), then add class methods (create_from_dict), then static methods (is_valid_priority). Practice property decorator on simple cases (priority validation) before computed properties."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/046-part5-todo-integration/reference/task-entity.md"
created: "2025-12-26"
last_modified: "2025-12-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Encapsulation and Method Types

Now you'll master professional-grade class design through access control and multiple method types. In this lesson, you'll build a Task class with factory methods and utility functions, learn from AI how to protect attributes and design methods, challenge AI with real-world design decisions across multiple domains, and build a comprehensive guide for method selection that you'll use throughout Part 5.

---

## Part 1: Experience Access Control and Method Types

**Your Role**: Code explorer discovering why data protection and method organization matter

### Discovery Exercise: The Unprotected Task Problem

Create `method_discovery.py`:

```python
# Without encapsulation - dangerous!
class UnsafeTask:
    def __init__(self, title: str, priority: int = 5):
        self.title = title
        self.priority = priority
        self.done = False

task = UnsafeTask("Review pull request", 1)

# Oops! Bug in my code sets priority to impossible value
task.priority = 999  # No validation!
print(task.priority)  # 999

# What if multiple methods need to work with priority?
# If format changes, I have to update everywhere
```

**Output:**
```
999
```

**Your task 1**: Identify the problems:
- What prevents setting invalid priorities (should be 1-10)?
- If I have 100 methods using `self.priority`, and I need to add validation, where do I modify code?

### Discovering the Solution: Method Types

**Stage 2: Using Instance Methods**

```python
class Task:
    def __init__(self, title: str, priority: int = 5):
        self._priority = priority  # Protected
        self.title = title
        self.done = False

    def set_priority(self, new_priority: int) -> None:
        if 1 <= new_priority <= 10:
            self._priority = new_priority
        else:
            raise ValueError("Priority must be between 1 and 10")

    def get_priority(self) -> int:
        return self._priority

    def mark_complete(self) -> None:
        self.done = True

task = Task("Review pull request", 1)
task.set_priority(5)  # Uses method, validated
print(task.get_priority())  # 5
# task._priority = 999  # Possible, but naming says "don't do this"
```

**Output:**
```
5
```

**Your task 2**: Run this and observe:
- How does using methods instead of direct attribute access help?
- What happens if someone ignores the _ convention?

**Stage 3: Discovering Private Attributes**

```python
class SecureTask:
    def __init__(self, title: str, priority: int = 5):
        self.__priority = priority  # Double underscore - name mangled
        self.title = title
        self.done = False

    def set_priority(self, new_priority: int) -> None:
        if 1 <= new_priority <= 10:
            self.__priority = new_priority
        else:
            raise ValueError("Priority must be between 1 and 10")

    def get_priority(self) -> int:
        return self.__priority

task = SecureTask("Review pull request", 1)
print(task.get_priority())  # 1
# Try: task.__priority  # AttributeError!
# Try: task._SecureTask__priority  # Works, but ugly - don't do it
```

**Output:**
```
1
```

**Your task 3**: Run this and answer:
- Can you access __priority directly?
- Is this truly "private" or just annoying?

### Discovery Exercise 2: Three Method Types with Task Class

![Comparison grid showing Python method types: instance methods with self parameter operating on object state, class methods with cls parameter using @classmethod decorator, and static methods with @staticmethod decorator for utility functions](https://pub-80f166e40b854371ac7b05053b435162.r2.dev/books/ai-native-dev/static/images/part-5/chapter-27/python-method-types-comparison.png)

**Stage 4: Instance Methods vs Class Methods vs Static Methods (Task Focus)**

```python
class Task:
    task_count = 0
    priority_levels = {1: "Critical", 2: "High", 3: "Medium-High",
                       5: "Normal", 8: "Low", 10: "Someday"}

    def __init__(self, title: str, priority: int = 5):
        self.title = title
        self.priority = priority
        self.done = False
        Task.task_count += 1

    # Instance method - operates on self (this specific task)
    def mark_complete(self) -> str:
        self.done = True
        return f"Task '{self.title}' completed!"

    # Class method - factory pattern (no instance needed yet)
    @classmethod
    def create_from_dict(cls, data: dict) -> "Task":
        """Factory method to create task from dictionary"""
        return cls(data['title'], data.get('priority', 5))

    # Class method - another factory example
    @classmethod
    def create_urgent(cls, title: str) -> "Task":
        """Shortcut factory for urgent (priority 1) tasks"""
        return cls(title, priority=1)

    # Static method - utility function (no self or cls needed)
    @staticmethod
    def is_valid_priority(priority: int) -> bool:
        return isinstance(priority, int) and 1 <= priority <= 10

    # Static method - lookup table
    @staticmethod
    def priority_levels() -> dict[int, str]:
        return {1: "Critical", 2: "High", 3: "Medium-High",
                5: "Normal", 8: "Low", 10: "Someday"}

# Instance method needs object
task = Task("Review pull request", priority=1)
print(task.mark_complete())  # Works - operates on this task

# Class method - create from dict (no instance exists yet)
task2 = Task.create_from_dict({"title": "Write tests", "priority": 3})
print(f"Created: {task2.title}")  # Works - creates new instance

# Class method - create urgent
urgent = Task.create_urgent("Fix critical bug")
print(f"Urgent task priority: {urgent.priority}")

# Static method - validate (doesn't need task object)
print(Task.is_valid_priority(5))  # True
print(Task.is_valid_priority(999))  # False

# Static method - get levels
levels = Task.priority_levels()
print(f"Valid levels: {levels}")

print(f"Total tasks created: {Task.task_count}")
```

**Output:**
```
Task 'Review pull request' completed!
Created: Write tests
Urgent task priority: 1
True
False
Valid levels: {1: 'Critical', 2: 'High', 3: 'Medium-High', 5: 'Normal', 8: 'Low', 10: 'Someday'}
Total tasks created: 3
```

**Your task 4**: Run this and observe:
- Which methods need an object (instance) to work?
- Which can work with just the class (no instance)?
- Which are just utility functions grouped with the class for organization?

### Your Discoveries

Document in `method_type_analysis.md`:
1. Why do instance methods use `self`?
2. When would you use a class method (like `create_from_dict`) instead?
3. When would a static method (like `is_valid_priority`) be useful?
4. Why protect attributes with methods instead of direct access?

---

## Part 2: Learn Access Control and Method Design

**Your Role**: Student receiving instruction from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I've built a Task class with three method types (@classmethod for create_from_dict, @staticmethod for is_valid_priority, instance methods for mark_complete). Show me and explain:
> 1. What's the difference between @classmethod and @staticmethod?
> 2. When should I use each for a Task class?
> 3. For a Task class, which method type would you use for:
>    - mark_complete() which changes THIS task's done status
>    - create_from_dict(data) which should create a new Task from a dictionary
>    - is_valid_priority(num) which just validates a number
> 4. Show me how to enforce that priority can only be 1-10 using a property."

### Convergence Activity

After AI explains, ask: "Show me how the @property decorator works for protecting priority while allowing read access. Why is this better than a get_priority() method? Can I use this with create_from_dict()?"

**Deliverable**: Write summary explaining when to use each method type in the context of a Task class, with examples of when you'd use each.

---

## Part 3: Challenge AI with Method Design

**Your Role**: Student teaching AI by testing design understanding

#### Challenge 1: Task Factory Patterns

> "Show me a Task class where you have TWO class methods: create_from_dict() for creating from a dictionary, and create_urgent() as a shortcut for high-priority tasks. Why are these class methods instead of instance methods or static methods? When would each be called during development?"

#### Challenge 2: Properties vs Methods

> "In a Task class with a `_priority` attribute, would you expose it as:
> - A method: task.get_priority()
> - A property: task.priority (looks like attribute but uses @property)
>
> Which feels more Pythonic? Why? Show me both versions and explain the tradeoffs."

#### Challenge 3: Static Methods in Practice

> "When would a static method be useful in a Task class? Show me two examples: one for validation (is_valid_priority) and one for data lookup (priority_levels). What about for other domains like Case, Invoice, or Appointment management? When would you use static methods vs module-level functions?"

### Deliverable

Document challenges and AI responses with your analysis across multiple domains.

---

## Part 4: Reusable Patterns Across Domains

**Your Role**: Knowledge synthesizer creating design framework

You've mastered method types with Task. Now apply the same patterns across different domains:

### Task Pattern (Primary Example)

```python
class Task:
    def __init__(self, title: str, priority: int = 5):
        self._title = title
        self._priority = priority
        self._done = False

    @classmethod
    def create_from_dict(cls, data: dict) -> "Task":
        """Create Task from dictionary representation"""
        task = cls(data["title"], data.get("priority", 5))
        task._done = data.get("done", False)
        return task

    @classmethod
    def create_urgent(cls, title: str) -> "Task":
        """Factory method for urgent (priority 1) tasks"""
        return cls(title, priority=1)

    @staticmethod
    def is_valid_priority(value: int) -> bool:
        """Check if priority value is valid (1-10)"""
        return isinstance(value, int) and 1 <= value <= 10

    @staticmethod
    def priority_levels() -> dict[int, str]:
        """Return priority level descriptions"""
        return {1: "Critical", 2: "High", 3: "Medium-High",
                5: "Normal", 8: "Low", 10: "Someday"}

    def mark_complete(self) -> None:
        """Mark this task as completed"""
        self._done = True

    @property
    def priority(self) -> int:
        return self._priority

    @priority.setter
    def priority(self, value: int) -> None:
        if not self.is_valid_priority(value):
            raise ValueError(f"Priority must be 1-10, got {value}")
        self._priority = value
```

### Auxiliary Domain 1: Legal (Case Management)

```python
class Case:
    def __init__(self, case_number: str, title: str, client_name: str):
        self._case_number = case_number
        self._title = title
        self._client_name = client_name
        self._status = "open"

    @classmethod
    def create_from_database(cls, db_record: dict) -> "Case":
        """Factory method: create Case from database record"""
        case = cls(db_record["number"], db_record["title"], db_record["client"])
        case._status = db_record.get("status", "open")
        return case

    @staticmethod
    def valid_statuses() -> list[str]:
        """Return valid case statuses"""
        return ["open", "pending", "closed", "settled"]

    def close_case(self) -> None:
        """Instance method: close this specific case"""
        self._status = "closed"
```

### Auxiliary Domain 2: Finance (Invoice Management)

```python
class Invoice:
    def __init__(self, invoice_id: str, client_name: str, amount: float):
        self._invoice_id = invoice_id
        self._client_name = client_name
        self._amount = amount
        self._is_paid = False

    @classmethod
    def create_from_json(cls, json_data: dict) -> "Invoice":
        """Factory method: create from JSON"""
        return cls(json_data["id"], json_data["client"], json_data["amount"])

    @staticmethod
    def calculate_tax(amount: float, tax_rate: float = 0.1) -> float:
        """Static utility: calculate tax (doesn't need instance)"""
        return amount * tax_rate

    def mark_paid(self) -> None:
        """Instance method: mark this invoice as paid"""
        self._is_paid = True
```

### Method Types and Encapsulation Guide

Create `method_types_and_encapsulation_guide.md`:

**Instance Methods** - Use when: Method operates on specific object's data

```python
# Example: Task instance method
def mark_complete(self):
    self._done = True
```

**Class Methods (@classmethod)** - Use when: Creating instances (factories) or class-level operations

```python
# Example: Task class method
@classmethod
def create_from_dict(cls, data: dict):
    task = cls(data['title'], data['priority'])
    task._done = data.get('done', False)
    return task
```

**Static Methods (@staticmethod)** - Use when: Utility function grouped with class but doesn't need instance or class

```python
# Example: Task static method
@staticmethod
def is_valid_priority(num: int) -> bool:
    return 1 <= num <= 10
```

**Access Levels:**

- **Public**: `self.name` - Direct access encouraged
- **Protected**: `self._priority` - Convention, don't access directly
- **Private**: `self.__internal` - Name mangled, strongly discourage access

**Properties (@property/@setter)** - Make attributes look like data while using method validation

```python
@property
def priority(self) -> int:
    return self._priority

@priority.setter
def priority(self, value: int) -> None:
    if not 1 <= value <= 10:
        raise ValueError("Priority must be 1-10")
    self._priority = value
```

**Deliverable**: Complete guide with method type selection criteria and patterns from Task, Case, Invoice domains.

---

## Try With AI

Ready to master encapsulation, method types, and properties using the Task class as your anchor?

**Explore Access Control Levels:**

> "Build a Task class with three attributes: title (public), _priority (protected, convention), __completion_code (private, name-mangled). Show me how to access each from outside the class. Explain why priority should be protected and what Python's name mangling does with __completion_code. Then show the same pattern for a Case class in a legal domain."

**What you're learning:** The three access control levels give you flexibility. Public attributes are fast and simple. Protected attributes signal "don't touch this from outside" by convention. Private attributes use Python's name mangling to make accidental access harder. This same strategy applies to any class in any domain.

---

**Master Task Class Methods and Static Methods:**

> "Create a Task class with: 1) create_from_dict() class method for building from dictionaries, 2) create_urgent() class method as a factory for high-priority tasks, 3) is_valid_priority() static method to validate numbers, 4) priority_levels() static method to return the lookup table. Show me how each works. Why can't instance methods do what class methods do? Why are static methods better than just module-level functions?"

**What you're learning:** Class methods are factories that create instances without needing one first. Static methods are utilities grouped with the class. You'll use this exact pattern in Part 5: Task.create_from_dict(), Case.create_from_database(), Invoice.create_from_json(). The pattern is universal.

---

**Test Properties for Validation:**

> "Build a Task class with priority property. Use @property for getter, @priority.setter for setter that validates (1 to 10 range). Try: task.priority = -5, task.priority = 200, task.priority = 5. Show how properties enable validation without breaking the attribute access API. Why is this better than get_priority()/set_priority() methods? Show this same pattern for an Invoice class with amount validation (must be positive)."

**What you're learning:** Properties look like attributes from the outside but use methods internally. This is Pythonic—clean API, hidden validation. The pattern transfers across domains: validate Task priority, Invoice amount, Appointment time, Case status.

---

**Apply Class Methods Across Domains:**

> "I'm building systems for Task management, legal cases, and invoices. Show me how each would use @classmethod as a factory: Task.create_from_dict(), Case.create_from_database(), Invoice.create_from_json(). Why does each use a class method instead of just calling __init__? When would you need to do extra setup before returning the instance?"

**What you're learning:** Class methods are the pattern for constructing objects with complex setup. Task might need to generate IDs, Case might need to deserialize nested objects, Invoice might need to calculate totals. The pattern works everywhere once you understand why class methods have access to the class itself.

---
