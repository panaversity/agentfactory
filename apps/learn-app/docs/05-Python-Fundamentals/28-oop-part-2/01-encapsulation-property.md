---
sidebar_position: 1
title: "Encapsulation and Property Decorators"
description: "Master data protection through properties and priority validation. Learn how @property and @setter enforce business rules across Task, Invoice, and Appointment domains."
keywords: ["encapsulation", "property decorator", "validation", "protected attributes", "data protection", "python properties"]
chapter: 28
lesson: 1
duration_minutes: 65

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment, accreditation alignment, and differentiation
skills:
  - name: "Property Decorator Usage"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement getters, setters, and computed properties using @property decorator with validation"

  - name: "Data Validation Through Properties"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can enforce business rules through property setters across multiple domains"

  - name: "Protected Attribute Patterns"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can choose between public, protected, and private attributes based on encapsulation needs"

  - name: "Encapsulation Through Properties"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can design encapsulation strategies using properties and explain trade-offs"

  - name: "Domain-Driven Validation"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can transfer validation patterns across domains (Task, Invoice, Appointment)"

  - name: "Getter/Setter Validation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Safety & Security"
    measurable_at_this_level: "Student can implement validation logic in property setters with meaningful error messages"

  - name: "Computed Properties"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Create"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create read-only properties that compute values from protected attributes"

  - name: "Pythonic API Design"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain why properties are more Pythonic than get_/set_ methods"

learning_objectives:
  - objective: "Implement property decorators with validation to protect attributes and enforce business rules"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student creates validated Task priority property with multiple domains"

  - objective: "Design encapsulation strategies using protected attributes and properties for data integrity"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Design review comparing encapsulation approaches for Invoice and Appointment"

  - objective: "Create computed properties that derive values from protected attributes without exposing internals"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student implements is_overdue computed property for deadlined tasks"

  - objective: "Apply property validation patterns across multiple business domains consistently"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student implements consistent validation in Task, Invoice, and Appointment classes"

cognitive_load:
  new_concepts: 8
  assessment: "8 new concepts (@property decorator, @setter, validation in setters, computed properties, protected attributes, type validation, value range validation, cross-domain pattern) at B2 maximum. At limit ✓"

differentiation:
  extension_for_advanced: "Implement custom property descriptors using __get__ and __set__. Create a ValidatedProperty class that accepts validation functions. Explore property inheritance—how subclasses override parent properties. Build a property-based validation framework for multiple domains."
  remedial_for_struggling: "Focus on simple property first (getter only with @property). Practice single validation rule before complex rules. Use provided templates for setter structure. Start with Task.priority before tackling Invoice.amount. Trace through property calls step-by-step with print statements."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/046-part5-todo-integration/reference/task-entity.md"
created: "2025-12-26"
last_modified: "2025-12-26"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Encapsulation and Property Decorators

In Chapter 27, you learned the basics of classes and access control. Now you'll master encapsulation through properties—a technique that protects your data while keeping your API clean and Pythonic. Through task priority validation, you'll discover how properties enforce business rules across Todo apps, invoicing systems, and appointment scheduling. By the end, you'll understand why properties are central to professional OOP design and how the same validation pattern applies everywhere.

---

## Part 1: Experience the Problem and Solution

**Your Role**: Code explorer discovering why unvalidated attributes create bugs

### Discovery Exercise: The Dangerous Unprotected Task

Create `unprotected_task.py`:

```python
# Without encapsulation - dangerous!
class UnsafeTask:
    def __init__(self, title: str, priority: int = 5):
        self.title = title
        self.priority = priority  # No validation!
        self.done = False

# Create a valid task
task = UnsafeTask("Review pull request", priority=1)
print(f"Task: {task.title}, Priority: {task.priority}")

# Oops! Bug in my code sets priority to an impossible value
task.priority = 999
print(f"After bug - Priority: {task.priority}")  # 999 is invalid!

# What if I have 50 methods checking priority?
# If the rules change, I edit everywhere instead of one place
```

**Output:**
```
Task: Review pull request, Priority: 1
After bug - Priority: 999
```

**Your observation**:
- No validation exists—any integer is accepted
- If priority rules change (e.g., "must be 1-10"), where do you update the logic?
- Direct attribute access makes it impossible to enforce rules

### Discovery Exercise 2: Protected Attribute with Method

Now add validation through a method:

```python
class SaferTask:
    def __init__(self, title: str, priority: int = 5):
        self._priority = priority  # Protected - signals "don't access directly"
        self.title = title
        self.done = False

    def set_priority(self, value: int) -> None:
        """Set priority with validation (1-10)."""
        if not isinstance(value, int):
            raise TypeError("Priority must be an integer")
        if not 1 <= value <= 10:
            raise ValueError("Priority must be between 1 and 10")
        self._priority = value

    def get_priority(self) -> int:
        """Get priority."""
        return self._priority

task = SaferTask("Review pull request", priority=1)
print(f"Task: {task.title}, Priority: {task.get_priority()}")

# Try to set an invalid priority
try:
    task.set_priority(999)
except ValueError as e:
    print(f"Error: {e}")

# Valid priority works
task.set_priority(3)
print(f"After update - Priority: {task.get_priority()}")
```

**Output:**
```
Task: Review pull request, Priority: 1
Error: Priority must be between 1 and 10
After update - Priority: 3
```

**Your observation**:
- Protected attribute (_priority) signals "don't touch this directly"
- get_priority() and set_priority() enforce validation in ONE place
- If rules change, update the setter once
- BUT: It feels verbose compared to direct attribute access

### Discovery Exercise 3: The Pythonic Solution - Properties

Now use @property and @setter decorators:

```python
class Task:
    def __init__(self, title: str, priority: int = 5):
        self._priority = priority  # Protected - hidden from outside
        self.title = title
        self.done = False

    @property
    def priority(self) -> int:
        """Get task priority."""
        return self._priority

    @priority.setter
    def priority(self, value: int) -> None:
        """Set priority with validation (1-10)."""
        if not isinstance(value, int):
            raise TypeError("Priority must be an integer")
        if not 1 <= value <= 10:
            raise ValueError("Priority must be between 1 and 10")
        self._priority = value

# Usage - looks like accessing an attribute!
task = Task("Review pull request", priority=1)
print(f"Task: {task.title}, Priority: {task.priority}")

# Setting looks like assignment, but uses validation!
try:
    task.priority = 999
except ValueError as e:
    print(f"Error: {e}")

# Valid priority works
task.priority = 3
print(f"After update - Priority: {task.priority}")
```

**Output:**
```
Task: Review pull request, Priority: 1
Error: Priority must be between 1 and 10
After update - Priority: 3
```

**Your observation**:
- Looks like attribute access (task.priority = 3)
- Actually runs validation code in the setter
- This is the Pythonic way—clean API + hidden validation
- Compare: `task.set_priority(3)` vs `task.priority = 3` — which feels more natural?

---

## Part 2: Learn Properties From AI Teacher

**Your Role**: Student receiving explanation from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I've discovered three ways to handle priority in a Task class:
> 1. Direct attribute: `self.priority = value` (no validation)
> 2. Methods: `task.set_priority(3)` and `task.get_priority()` (validated)
> 3. Properties: `task.priority = 3` (validated, looks like attributes)
>
> Explain:
> - Why is the property approach better than method approach?
> - What does @property decorator do? What does @priority.setter do?
> - Why is `task.priority = 5` more Pythonic than `task.set_priority(5)`?
> - Show me how the priority property stops invalid values"

### Convergence Activity

After AI explains, ask this follow-up question:

> "Now I understand properties hide validation behind attribute-like syntax. Show me a property that's read-only (getter only, no setter). When would I want a property you can't change after creation?"

**What you're learning**: Properties bridge two worlds—they look like simple attributes from the outside but enforce rules inside. This is the Pythonic philosophy: "make the simple case simple, but allow complexity when needed." The same pattern works for task priority, invoice amounts, appointment times, anywhere you need validation.

---

## Part 3: Challenge AI with Design Decisions

**Your Role**: Student teaching AI by testing its understanding

### Challenge 1: Priority Validation Across Domains

> "Build Task, Invoice, and Appointment classes where:
> - Task has priority (1-10)
> - Invoice has discount_percent (0-100)
> - Appointment has duration_minutes (15-480 in 15-min increments)
>
> Use @property/@setter for validation in all three. Show the pattern is identical—validation rules change, but the @property/@setter structure stays the same. Why is this pattern called 'domain-driven validation'?"

### Challenge 2: Computed Properties (Read-Only)

> "In a Task class with _priority attribute, create a read-only property `urgency_label` that returns:
> - 'Critical' if priority <= 2
> - 'High' if priority <= 4
> - 'Medium' if priority <= 7
> - 'Low' if priority > 7
>
> Why is this a property instead of a method? When do you use @property without @setter (read-only computed properties)?"

### Challenge 3: Type and Value Validation

> "Create an Invoice class with amount property. Validate:
> - Must be float or int (convert int to float)
> - Must be positive (> 0)
> - Cannot be less than 0.01 (minimum charge)
>
> Then add a discount property that:
> - Accepts 0-100 percent
> - Won't set if amount hasn't been set yet
> - Raises meaningful error messages
>
> Show how property validation prevents impossible states."

### Deliverable

Document the three challenges and AI responses in `property_validation_patterns.md`. Include your analysis of which patterns transfer across domains.

---

## Part 4: Master Encapsulation Across Domains

**Your Role**: Knowledge synthesizer building reusable validation patterns

### Pattern 1: Simple Range Validation

The most common pattern—enforce minimum and maximum values:

```python
class Task:
    def __init__(self, title: str, priority: int = 5):
        self._priority = priority
        self.title = title

    @property
    def priority(self) -> int:
        """Get task priority (1=highest, 10=lowest)."""
        return self._priority

    @priority.setter
    def priority(self, value: int) -> None:
        """Set priority with validation."""
        if not isinstance(value, int):
            raise TypeError("Priority must be an integer")
        if not 1 <= value <= 10:
            raise ValueError("Priority must be between 1 and 10")
        self._priority = value
```

**Applied to multiple domains:**

#### Invoice Amount Validation

```python
class Invoice:
    def __init__(self, invoice_id: str, client_name: str, amount: float):
        self._amount = amount
        self.invoice_id = invoice_id
        self.client_name = client_name

    @property
    def amount(self) -> float:
        """Get invoice amount."""
        return self._amount

    @amount.setter
    def amount(self, value: float) -> None:
        """Set amount with validation (must be positive)."""
        if not isinstance(value, (int, float)):
            raise TypeError("Amount must be a number")
        if value <= 0:
            raise ValueError("Amount must be positive")
        if value < 0.01:
            raise ValueError("Minimum invoice amount is 0.01")
        self._amount = float(value)

# Usage
invoice = Invoice("INV-001", "ACME Corp", 1500.00)
print(f"Invoice: {invoice.amount}")

try:
    invoice.amount = -100  # Invalid!
except ValueError as e:
    print(f"Error: {e}")

invoice.amount = 2000.50  # Valid
print(f"Updated: {invoice.amount}")
```

**Output:**
```
Invoice: 1500.0
Error: Amount must be positive
Updated: 2000.5
```

#### Appointment Status Validation

```python
class Appointment:
    VALID_STATUSES = {"scheduled", "in_progress", "completed", "cancelled"}

    def __init__(self, appointment_id: str, patient_name: str, doctor_name: str):
        self._status = "scheduled"
        self.appointment_id = appointment_id
        self.patient_name = patient_name
        self.doctor_name = doctor_name

    @property
    def status(self) -> str:
        """Get appointment status."""
        return self._status

    @status.setter
    def status(self, value: str) -> None:
        """Set status with validation (must be in allowed values)."""
        if not isinstance(value, str):
            raise TypeError("Status must be a string")
        if value not in self.VALID_STATUSES:
            raise ValueError(
                f"Status must be one of {self.VALID_STATUSES}. Got '{value}'"
            )
        self._status = value

# Usage
apt = Appointment("APT-001", "John Doe", "Dr. Smith")
print(f"Status: {apt.status}")

try:
    apt.status = "pending"  # Invalid!
except ValueError as e:
    print(f"Error: {e}")

apt.status = "completed"  # Valid
print(f"Updated: {apt.status}")
```

**Output:**
```
Status: scheduled
Error: Status must be one of {'scheduled', 'in_progress', 'completed', 'cancelled'}. Got 'pending'
Updated: completed
```

### Pattern 2: Computed Properties (Read-Only)

Properties that calculate values from protected attributes without setter:

```python
from datetime import datetime, timedelta

class Task:
    def __init__(self, title: str, priority: int = 5, due_date: datetime | None = None):
        self._title = title
        self._priority = priority
        self._due_date = due_date
        self.done = False

    @property
    def title(self) -> str:
        return self._title

    @property
    def priority(self) -> int:
        return self._priority

    @priority.setter
    def priority(self, value: int) -> None:
        if not 1 <= value <= 10:
            raise ValueError("Priority must be between 1 and 10")
        self._priority = value

    @property
    def due_date(self) -> datetime | None:
        return self._due_date

    @property
    def is_overdue(self) -> bool:
        """Computed property - no setter. Read-only."""
        if self._due_date is None or self.done:
            return False
        return datetime.now() > self._due_date

    @property
    def days_until_due(self) -> int | None:
        """Computed property - returns days remaining."""
        if self._due_date is None or self.done:
            return None
        delta = self._due_date - datetime.now()
        return delta.days

# Usage
task = Task("Ship release", priority=1, due_date=datetime(2025, 01, 15))
print(f"Task: {task.title}")
print(f"Days until due: {task.days_until_due}")
print(f"Is overdue? {task.is_overdue}")

task.done = True
print(f"After marking complete - Is overdue? {task.is_overdue}")
```

**Output:**
```
Task: Ship release
Days until due: 19
Is overdue? False
After marking complete - Is overdue? False
```

### Pattern 3: Dependent Property Validation

Some properties depend on other properties being set:

```python
class Invoice:
    def __init__(self, invoice_id: str, client_name: str, amount: float):
        self._amount = amount
        self._discount_percent = 0.0
        self.invoice_id = invoice_id
        self.client_name = client_name

    @property
    def amount(self) -> float:
        return self._amount

    @amount.setter
    def amount(self, value: float) -> None:
        if value <= 0:
            raise ValueError("Amount must be positive")
        self._amount = float(value)

    @property
    def discount_percent(self) -> float:
        return self._discount_percent

    @discount_percent.setter
    def discount_percent(self, value: float) -> None:
        """Discount depends on amount being set."""
        if not 0 <= value <= 100:
            raise ValueError("Discount must be between 0-100%")
        if self._amount == 0:
            raise ValueError("Cannot set discount before amount is set")
        self._discount_percent = float(value)

    @property
    def final_amount(self) -> float:
        """Computed amount after discount."""
        return self._amount * (1 - self._discount_percent / 100)

# Usage
invoice = Invoice("INV-001", "ACME", 1000.0)
print(f"Original: {invoice.amount}")

# Discount requires amount to be set first
try:
    invoice2 = Invoice("INV-002", "ACME", 0)
    invoice2.discount_percent = 10  # Fails - amount is 0
except ValueError as e:
    print(f"Error: {e}")

# Valid: amount set, then discount
invoice.discount_percent = 10
print(f"Discount: {invoice.discount_percent}%")
print(f"Final: {invoice.final_amount}")
```

**Output:**
```
Original: 1000.0
Error: Cannot set discount before amount is set
Discount: 10.0
Final: 900.0
```

### Pattern 4: Type Coercion and Validation

Validate and convert types automatically:

```python
class Task:
    def __init__(self, title: str, priority: int = 5):
        self._priority = priority
        self.title = title

    @property
    def priority(self) -> int:
        return self._priority

    @priority.setter
    def priority(self, value: int | str) -> None:
        """Accept int or string representation of priority."""
        # Try to convert string to int
        if isinstance(value, str):
            try:
                value = int(value)
            except ValueError:
                raise ValueError(f"Cannot convert '{value}' to priority number")

        if not isinstance(value, int):
            raise TypeError("Priority must be int or string")

        if not 1 <= value <= 10:
            raise ValueError("Priority must be between 1 and 10")

        self._priority = value

# Usage
task = Task("Review PR", priority=1)
print(f"Priority: {task.priority}")

# Accept string and convert automatically
task.priority = "5"
print(f"After string assignment: {task.priority}")

# Reject invalid strings
try:
    task.priority = "urgent"  # Can't convert
except ValueError as e:
    print(f"Error: {e}")
```

**Output:**
```
Priority: 1
After string assignment: 5
Error: Cannot convert 'urgent' to priority number
```

### Encapsulation and Property Design Guide

Create `property_pattern_guide.md`:

| Pattern | Use When | Example |
|---------|----------|---------|
| **@property getter only** | Expose protected attribute read-only | `task.priority` - read but protect from modification |
| **@property + @setter** | Need validation on both read and write | Task.priority, Invoice.amount must validate on set |
| **Computed property (no setter)** | Value derives from other attributes | `task.is_overdue`, `invoice.final_amount` |
| **Type coercion in setter** | Accept multiple types, convert automatically | Accept `priority = "5"` or `priority = 5` |
| **Dependent validation** | One property depends on another | Can't set discount until amount is set |

**Pythonic Principles**:
- Use `@property` instead of `get_priority()` methods
- Make simple cases simple: `task.priority = 5`
- Hide complexity: validation runs automatically
- Protect internals: `_priority` signals "don't touch"
- Meaningful errors: "Priority must be 1-10", not "Invalid"

**Deliverable**: Complete guide with decision criteria for each pattern and domain examples.

---

## Try With AI

Ready to master properties, encapsulation, and validation across real-world domains?

### Prompt 1: Task Priority Validation with Properties

> "Build a Task class where priority is protected with a @property decorator. Validate that priority:
> - Must be an integer
> - Must be between 1 and 10
> - Raises TypeError if not integer
> - Raises ValueError if out of range
>
> Show the implementation, then demonstrate:
> ```python
> task = Task('Review PR', priority=1)
> print(task.priority)  # Works: 1
>
> task.priority = 5  # Works - looks like assignment
> task.priority = 15  # Should raise ValueError
> task.priority = 'high'  # Should raise TypeError
> ```
>
> Explain why using @property is better than task.set_priority(5) or task.get_priority()."

**What you're learning**: The @property decorator creates a clean API. From the outside, task.priority looks like a simple attribute. Inside, validation runs automatically. This is Pythonic—the language should adapt to your domain, not the reverse. Priority validation applies everywhere: Task, Invoice, Appointment, Case. You'll use this pattern constantly.

---

### Prompt 2: Invoice Amount Validation with Type Coercion

> "Create an Invoice class with amount property that:
> - Accepts int or float
> - Converts int to float
> - Rejects negative, zero, or amounts < 0.01
> - Raises TypeError for non-numeric types
> - Raises ValueError with the specific problem ('Amount must be positive', 'Minimum is 0.01')
>
> Test with:
> ```python
> inv = Invoice('INV-001', 'ACME', 1500)  # int - converts to float
> inv.amount = 2000.50  # float
> inv.amount = -100  # ValueError
> inv.amount = 'expensive'  # TypeError
> ```
>
> Apply the same pattern to an Appointment class with duration_minutes property (must be 15-480 in 15-min increments). Show how the validation structure is identical."

**What you're learning**: Properties enforce business rules consistently. The pattern is always the same: (1) check type, (2) check value constraints, (3) raise specific errors. When you apply this to 5 different classes, you recognize the pattern and code faster. This is how expert developers work—they transfer patterns across domains.

---

### Prompt 3: Computed Properties and Read-Only Access

> "Create a Task class with:
> - title, priority, due_date attributes (protected with _)
> - priority property with @setter (validate 1-10)
> - is_overdue computed property (True if due_date < now() and not done)
> - days_until_due computed property (returns int days remaining)
> - urgency_label computed property ('Critical', 'High', 'Medium', 'Low' based on priority)
>
> Show that these computed properties have @property but no @setter (read-only). Why would you try to do `task.is_overdue = True`? What should happen? Demonstrate the difference between a property you CAN change (priority) vs properties you CANNOT change (is_overdue, urgency_label)."

**What you're learning**: Properties can be read-only (no setter). Some things should never be assigned directly—overdue status should be calculated, not set by hand. When you see a property without a setter, you know "this value is computed, don't set it." This is another Pythonic principle—the language communicates intent through code structure.

---

### Prompt 4: Dependent Property Validation (Advanced)

> "In an Invoice class:
> - amount property (must be positive, min 0.01)
> - discount_percent property (0-100%, but CANNOT be set unless amount > 0)
> - final_amount computed property (returns amount * (1 - discount/100))
>
> Show that:
> ```python
> inv = Invoice('INV-001', 'Client', 0)  # amount = 0
> inv.discount_percent = 10  # Should raise: 'Cannot set discount before amount is set'
>
> inv.amount = 1000  # Now amount is valid
> inv.discount_percent = 10  # Now this works!
> print(inv.final_amount)  # 900
> ```
>
> Why would you create dependent validation? What business scenario requires 'cannot discount a zero-dollar invoice'? Apply this pattern to a Task class where status can't be 'completed' unless priority was set."

**What you're learning**: Real systems have complex rules. Properties let you enforce them cleanly. Dependent validation prevents invalid states from ever existing in your code. This is called "making impossible states impossible"—if your code logic makes it impossible to create an overdue non-deadline task or a discounted zero-dollar invoice, bugs can't happen.

---

## Try With AI: Mastery Through Application

You've now discovered all the patterns. Time to integrate them with AI and master the complete property validation toolkit.

---

### Synthesis: Multi-Domain Property Validation

> "I've learned @property decorators with validation across Task, Invoice, and Appointment classes. Now show me one comprehensive example that combines everything:
>
> Build a Case class (legal domain) with:
> 1. case_number property (string, must match format 'CASE-XXXX-YYYY')
> 2. status property (enum: 'open', 'discovery', 'trial', 'closed' with validation)
> 3. amount_at_stake property (must be positive, cannot be changed once status is 'trial' or 'closed')
> 4. is_critical computed property (True if amount > $1M)
> 5. days_in_discovery computed property (read-only)
>
> Show how the same patterns from Task, Invoice, and Appointment apply to Case. Why is this pattern called 'domain-driven property design'? Apply this architecture to a Patient class (healthcare domain)."

**What you're learning**: You're now a professional-grade OOP architect. The property validation pattern is universal—Task priority, Invoice amount, Case status, Patient ID. When you see a new class, you know: (1) Which attributes need validation? (2) Which are protected (_)? (3) Which are computed (no setter)? (4) What are the business rules? Properties let you enforce all of this while keeping the API clean. This is what separates amateur code from professional systems.

