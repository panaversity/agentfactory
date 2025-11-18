---
title: "Encapsulation and Method Types"
chapter: 24
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
    measurable_at_this_level: "Student can implement @classmethod for factory patterns and class-level operations"

  - name: "Static Method Implementation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement @staticmethod for utility functions within class scope"

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

  - objective: "Implement all three method types (instance, class, static) and explain when each is appropriate"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Implementation exercise with all three method types in one class"

  - objective: "Apply property decorators to implement Pythonic getters, setters, and computed attributes"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Property implementation with validation and computed properties"

  - objective: "Design encapsulation strategies using properties for data protection and validation"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Design exercise implementing validated BankAccount class"

cognitive_load:
  new_concepts: 10
  assessment: "10 concepts (public/protected/private, instance methods, @classmethod, @staticmethod, @property, @setter, computed properties, data validation, encapsulation strategy, ABC contracts) at B2 maximum. At limit ✓"

differentiation:
  extension_for_advanced: "Research Python's @abstractmethod with @property. Design an abstract DataStore with property-based interface. Explore Protocol typing (PEP 544) as alternative to ABC for duck typing enforcement."
  remedial_for_struggling: "Focus on one method type at a time. Start with instance methods, then add class methods, then static methods. Practice property decorator on simple cases (age validation) before computed properties."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-24.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Encapsulation and Method Types

Now you'll master professional-grade class design through access control and multiple method types. In this lesson, you'll discover why encapsulation matters through experimentation, learn from AI how to protect attributes and design methods, challenge AI with real-world design decisions, and build a comprehensive guide for method selection.

---

## Part 1: Student Discovers Access Control and Method Types

**Your Role**: Code explorer discovering why data protection and method organization matter

### Discovery Exercise: The Unprotected Account Problem

Create `method_discovery.py`:

```python
# Without encapsulation - dangerous!
class UnsafeBankAccount:
    def __init__(self, holder: str, balance: float):
        self.balance = balance
        self.holder = holder

account = UnsafeBankAccount("Alice", 1000)

# Oops! Bug in my code sets balance to impossible value
account.balance = -999999  # No validation!
print(account.balance)  # -999999

# What if multiple methods need to work with balance?
# If format changes, I have to update everywhere
```

**Your task 1**: Identify the problems:
- What prevents setting invalid balances?
- If I have 100 methods using `self.balance`, and I need to add a fee calculation, where do I modify code?

### Discovering the Solution: Method Types

**Stage 2: Using Instance Methods**

```python
class SaferBankAccount:
    def __init__(self, holder: str, balance: float):
        self._balance = balance  # Protected
        self.holder = holder

    def deposit(self, amount: float) -> float:
        if amount <= 0:
            raise ValueError("Deposit must be positive")
        self._balance += amount
        return self._balance

    def get_balance(self) -> float:
        return self._balance

account = SaferBankAccount("Alice", 1000)
account.deposit(100)  # Uses method, validated
# account._balance = -999  # Possible, but naming says "don't do this"
```

**Your task 2**: Run this and observe:
- How does using methods instead of direct attribute access help?
- What happens if someone ignores the _ convention?

**Stage 3: Discovering Private Attributes**

```python
class SecureBankAccount:
    def __init__(self, holder: str, balance: float):
        self.__balance = balance  # Double underscore - name mangled
        self.holder = holder

    def deposit(self, amount: float) -> float:
        if amount <= 0:
            raise ValueError("Deposit must be positive")
        self.__balance += amount
        return self.__balance

    def get_balance(self) -> float:
        return self.__balance

account = SecureBankAccount("Alice", 1000)
print(account.get_balance())  # 1000
# Try: account.__balance  # AttributeError!
# Try: account._SecureBankAccount__balance  # Works, but ugly - don't do it
```

**Your task 3**: Run this and answer:
- Can you access __balance directly?
- Is this truly "private" or just annoying?

### Discovery Exercise 2: Multiple Method Types

**Stage 4: Instance Methods vs Class Methods vs Static Methods**

```python
class Dog:
    species = "Canis familiaris"
    total_dogs_created = 0

    def __init__(self, name: str):
        self.name = name
        Dog.total_dogs_created += 1

    # Instance method - operates on self
    def bark(self) -> str:
        return f"{self.name} says: Woof!"

    # Class method - operates on class, not instance
    @classmethod
    def from_breed_name(cls, name: str, breed: str):
        """Factory method to create dogs with breed"""
        return cls(f"{name} ({breed})")

    # Static method - doesn't need self or cls
    @staticmethod
    def is_good_name(name: str) -> bool:
        return len(name) >= 2

# Instance method needs object
dog = Dog("Max")
print(dog.bark())  # Works - operates on Max

# Class method uses class, can be used without instance
dog2 = Dog.from_breed_name("Buddy", "Labrador")

# Static method is just a function in class namespace
print(Dog.is_good_name("M"))  # False
```

**Your task 4**: Run this and observe:
- Which methods need an object to work?
- Which can work with just the class?
- Which are just utility functions grouped with the class?

### Your Discoveries

Document in `method_type_analysis.md`:
1. Why do instance methods use `self`?
2. When would you use a class method instead?
3. When would a static method be useful?
4. Why protect attributes with methods instead of direct access?

---

## Part 2: AI Explains Access Control and Method Design

**Your Role**: Student receiving instruction from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I've discovered three method types and different access control levels. Show me and explain:
> 1. What's the difference between @classmethod and @staticmethod?
> 2. When should I use each?
> 3. For a BankAccount class, which method type would you use for:
>    - deposit() which changes THIS account's balance
>    - get_interest_rate() which all accounts share
>    - is_valid_account_number() which just validates a string
> 4. Show me how to enforce that balance can't be negative using a property."

### Convergence Activity

After AI explains, ask: "Show me how the @property decorator works for protecting an attribute while allowing read access. Why is this better than a get_balance() method?"

**Deliverable**: Write summary explaining when to use each method type.

---

## Part 3: Student Challenges AI with Method Design

**Your Role**: Student teaching AI by testing design understanding

#### Challenge 1: Factory Pattern

> "Show me a Dog class where you have a class method that creates a Dog from a string like 'Max_Labrador_5'. Why is this better than having users call __init__ directly?"

#### Challenge 2: Properties vs Methods

> "In a class with a `_balance` attribute, would you expose it as:
> - A method: account.get_balance()
> - A property: account.balance (looks like attribute but uses @property)
>
> Which feels more Pythonic? Why?"

#### Challenge 3: Static Methods in Practice

> "When would a static method be useful? Show me a real example where a static method makes sense vs a module-level function."

### Deliverable

Document challenges and AI responses with your analysis.

---

## Part 4: Build Your Method Design Guide

**Your Role**: Knowledge synthesizer creating design framework

Create `method_types_and_encapsulation_guide.md`:

```markdown
# Method Types and Encapsulation Guide

## Instance Methods
Use when: Method operates on specific object's data

```python
def deposit(self, amount: float):
    self._balance += amount
```

## Class Methods (@classmethod)
Use when: Method operates on class data or creates instances

```python
@classmethod
def from_dict(cls, data: dict):
    return cls(data['name'], data['balance'])
```

## Static Methods (@staticmethod)
Use when: Utility function grouped with class but doesn't need instance or class

```python
@staticmethod
def is_valid_account_number(num: str) -> bool:
    return len(num) == 10
```

## Access Levels

- **Public**: `self.name` - Direct access encouraged
- **Protected**: `self._balance` - Convention, don't access directly
- **Private**: `self.__ssn` - Name mangled, discourage access

## Properties (@property/@setter)
Make attributes look like data while using method validation

```python
@property
def balance(self) -> float:
    return self._balance

@balance.setter
def balance(self, value: float):
    if value < 0:
        raise ValueError("Balance cannot be negative")
    self._balance = value
```
```

**Deliverable**: Complete guide with method type selection criteria.

---

## Summary: Bidirectional Learning Pattern

**Part 1 (Student discovers)**: You explored why encapsulation matters and discovered three method types
**Part 2 (AI teaches)**: AI explained when to use each method type and properties
**Part 3 (Student teaches)**: You challenged AI with design decisions
**Part 4 (Knowledge synthesis)**: You built a method design guide

### What You've Built

1. `method_type_analysis.md` — Your discoveries about method types
2. Summary — Understanding access control and method selection
3. Challenge documentation — Real-world design decisions
4. `method_types_and_encapsulation_guide.md` — Design reference

### Next Steps

Lesson 5 is the capstone where you'll design and build the Game Character System, using all lessons 1-4 concepts together to create a professional multi-class system.

