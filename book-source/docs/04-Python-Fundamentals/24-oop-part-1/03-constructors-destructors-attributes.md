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
source_spec: "specs/020-oop-part-1-2/spec-chapter-24.md"
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

## Part 1: Student Discovers Default Parameters and Attribute Types

**Your Role**: Code experimenter discovering initialization patterns and attribute scoping

### Discovery Exercise 1: Why Default Parameters Matter

**Stage 1: Required parameters only**

Create a file called `attribute_experiments.py`:

```python
class Dog:
    def __init__(self, name: str, breed: str, age: int):
        self.name = name
        self.breed = breed
        self.age = age

# This works, but requires every parameter
dog1 = Dog("Max", "Labrador", 5)
dog2 = Dog("Buddy", "Golden", 3)

# But what if you don't know the age?
# You're forced to guess or use a placeholder
dog3 = Dog("Unknown", "Mixed", 0)  # Awkward!
```

**Your task 1**: Run this and document:
- Do you have to provide all parameters?
- What if some info is optional?
- What design pattern solves this problem?

**Stage 2: Add Default Parameters**

Modify the class:

```python
class Dog:
    def __init__(self, name: str, breed: str, age: int = 0):
        self.name = name
        self.breed = breed
        self.age = age

# Much more flexible!
dog1 = Dog("Max", "Labrador", 5)
dog2 = Dog("Buddy", "Golden")  # Uses default age
dog3 = Dog("Unknown", "Mixed")  # No guessing needed
```

**Your task 2**: Run this and answer:
- How many ways can you now call Dog()?
- Why is this better than forcing all parameters?

### Discovery Exercise 2: Class vs Instance Attributes

**Stage 3: Observe Instance Attributes (we know these)**

```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name      # Each dog has its own name
        self.breed = breed    # Each dog has its own breed

dog1 = Dog("Max", "Labrador")
dog2 = Dog("Buddy", "Golden Retriever")

# Modify dog1
dog1.breed = "Lab Mix"

# Is dog2 affected?
print(dog2.breed)  # Still "Golden Retriever"
```

**Your task 3**: Run this and confirm:
- Each object has its own attribute values (instance attributes)
- Modifying one doesn't affect the other

**Stage 4: Discover Class Attributes (something new)**

```python
class Dog:
    species = "Canis familiaris"  # Class attribute (defined outside __init__)

    def __init__(self, name: str, breed: str):
        self.name = name      # Instance attribute
        self.breed = breed    # Instance attribute

dog1 = Dog("Max", "Labrador")
dog2 = Dog("Buddy", "Golden Retriever")

# Both dogs share the same species
print(dog1.species)  # Canis familiaris
print(dog2.species)  # Canis familiaris (same!)

# Change the CLASS attribute
Dog.species = "Canis lupus familiaris"
print(dog1.species)  # Now shows the new value!
print(dog2.species)  # Both are affected!
```

**Your task 4**: Run this and document:
- What's a class attribute?
- Is it shared between objects or independent?
- If you change it on one object, what happens?

**Stage 5: The Shadowing Problem**

```python
class Dog:
    tricks_known = 0  # Class attribute tracking total tricks

    def __init__(self, name: str):
        self.name = name  # Instance attribute

dog1 = Dog("Max")
dog2 = Dog("Buddy")

print(dog1.tricks_known)  # 0
print(dog2.tricks_known)  # 0

# Now something interesting happens
dog1.tricks_known = 5  # Create an instance attribute that SHADOWS the class attribute

print(dog1.tricks_known)  # 5 (instance attribute)
print(dog2.tricks_known)  # 0 (still using class attribute)
print(Dog.tricks_known)   # 0 (class attribute unchanged)
```

**Your task 5**: Run this and answer:
- Why did dog2.tricks_known stay 0?
- What happened in memory?
- Is this a bug or a feature?

### Your Discoveries

Document your findings in `attribute_types_analysis.md`:
1. What are instance attributes? When do you use them?
2. What are class attributes? When do you use them?
3. Why is the shadowing behavior important to understand?
4. How would you design a class that uses both types appropriately?

---

## Part 2: AI Explains Advanced Constructor Patterns

**Your Role**: Student receiving instruction from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I'm designing classes with optional parameters and I've discovered two types of attributes:
> - Instance attributes like `self.name` (each object has its own)
> - Class attributes like `tricks_known = 0` (shared across all objects)
>
> Explain:
> 1. When should I use default parameters in `__init__`?
> 2. What's the memory layout difference between instance and class attributes?
> 3. If dog1.tricks_known = 5 creates a new instance attribute, what happens to the class attribute?
> 4. Show me a real example where class attributes are useful (not artificial)."

### What You'll Learn from AI

**Expected AI Response** (summary):

- **Default parameters**: Make constructors flexible for optional data
- **Instance attributes**: Each object has independent copies
- **Class attributes**: All objects share one copy
- **Shadowing**: Creating instance attribute hides class attribute temporarily
- **Real use case**: Configuration values (class) vs instance data (instance)

### Convergence Activity

After AI explains, ask:

> "Walk me through what happens in memory when I create `dog = Dog('Max')` in a class that has both `species = 'Dog'` (class attribute) and `self.name = name` (instance attribute)."

**Deliverable**: Write a summary explaining when to use default parameters, class vs instance attributes, and why shadowing is important.

---

## Part 3: Student Challenges AI with Edge Cases

**Your Role**: Student teaching AI by testing edge case understanding

#### Challenge 1: Default Parameters and Mutability

**Your prompt to AI**:

> "I see a lot of code that uses `def __init__(self, items: list = []):`. But online I see warnings that this is dangerous. Why is using a mutable default parameter (list, dict) problematic? Show me an example where this goes wrong and explain what's happening."

**Expected learning**: AI will explain that mutable defaults are shared across all instances, causing hidden bugs.

#### Challenge 2: Class Attributes in AI Agents

**Your prompt to AI**:

> "I'm designing a ChatAgent class. Configuration like `model_name = 'gpt-4'` could be:
> - A class attribute (all agents use same model)
> - An instance attribute in __init__ (each agent can use different model)
>
> For these scenarios, which should it be:
> 1. All agents in production use gpt-4, test agents use gpt-3.5
> 2. Each conversation with a user uses a potentially different model
> 3. The API key for authentication
>
> Explain your reasoning."

**Expected learning**: AI will explain design decisions about when to share data.

#### Challenge 3: Attribute Inheritance Behavior

**Your prompt to AI**:

> "If I create a subclass of Dog:
> ```python
> class Dog:
>     species = 'Canis familiaris'
>
> class Husky(Dog):
>     pass
>
> h = Husky()
> h.species = 'Husky Mix'
> ```
>
> Does this create an instance attribute on h that shadows Dog.species? Or does it modify the class attribute? Show me how to verify which happened."

### Deliverable

Document your three challenges and AI's responses with your analysis of correctness.

---

## Part 4: Build Your Attributes and Constructors Reference

**Your Role**: Knowledge synthesizer creating design patterns

Create `constructor_and_attributes_guide.md`:

```markdown
# Constructors with Defaults and Attributes Guide

## Pattern 1: Basic Constructor with Defaults

**When to use**: Optional parameters that don't need validation

```python
class Person:
    def __init__(self, name: str, age: int = 0, city: str = "Unknown"):
        self.name = name
        self.age = age
        self.city = city

# All these work
Person("Alice", 30, "NYC")      # All parameters
Person("Bob", 25)                # Uses default city
Person("Charlie")                # Uses default age and city
```

**Key principle**: Required parameters first, optional last

---

## Pattern 2: Default Parameters with Validation

```python
class BankAccount:
    def __init__(self, holder: str, initial_balance: float = 0.0):
        if initial_balance < 0:
            raise ValueError("Balance cannot be negative")
        self.holder = holder
        self.balance = initial_balance
```

**Key principle**: Validate before storing

---

## Pattern 3: Distinguishing Instance vs Class Attributes

**Instance attributes** (use in `__init__`):
```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name        # Each dog has its own name
        self.breed = breed      # Each dog has its own breed
```

**Class attributes** (define in class body):
```python
class Dog:
    species = "Canis familiaris"  # All dogs are this species
    total_dogs = 0                # Track total dogs created

    def __init__(self, name: str):
        self.name = name
```

**Rule**: If data is specific to one object → instance attribute. If data is shared → class attribute.

---

## Pattern 4: Updating Class Attributes

```python
class Game:
    high_score = 0  # Class attribute

    def __init__(self, player_name: str):
        self.player_name = player_name  # Instance
        self.score = 0                  # Instance

g1 = Game("Alice")
g1.score = 100

# To update class attribute, use ClassName.attribute
Game.high_score = 100  # All Game objects see this

# NOT through instance (creates shadowing):
# g1.high_score = 100  # This creates instance attribute, doesn't modify class attribute
```

---

## Pattern 5: Detecting Attribute Type

```python
obj = Dog("Max")
obj.name = "Max Jr."

# Is name instance or class attribute?
print("name" in obj.__dict__)  # True = instance, False = class

print(Dog.__dict__["species"])  # Access class attribute via class
```

---

## Pattern 6: Avoiding Mutable Default Parameters

```python
# ❌ WRONG - Mutable default shared between instances!
class Classroom:
    def __init__(self, name: str, students: list = []):
        self.name = name
        self.students = students

class1 = Classroom("A")
class2 = Classroom("B")
class1.students.append("Alice")
print(class2.students)  # ["Alice"] - SHARED!

# ✅ CORRECT - Use None, create new list inside
class Classroom:
    def __init__(self, name: str, students: list = None):
        self.name = name
        self.students = students if students is not None else []
```

---

## Pattern 7: Class Attributes for Configuration

```python
class APIClient:
    # Configuration shared by all clients
    base_url = "https://api.example.com"
    timeout = 30

    def __init__(self, api_key: str):
        # Each client has unique credentials
        self.api_key = api_key

# All clients use same base URL and timeout
# But each has different API key
```

---

## Constructor Pattern Checklist

When designing a constructor, ask:

1. **What parameters are required?** (must provide)
2. **What parameters are optional?** (have sensible defaults)
3. **Should this be a class attribute?** (shared across instances)
4. **Should this be an instance attribute?** (specific to each object)
5. **Do I need validation?** (check constraints)
```

### Validation with AI

> "Review my constructor and attribute design patterns. Are my recommendations about when to use defaults vs required parameters sound? What patterns am I missing?"

**Deliverable**: Complete guide with all patterns and checklist.

---

## Summary: Bidirectional Learning Pattern

**Part 1 (Student discovers)**: You experimented with default parameters and discovered instance vs class attributes
**Part 2 (AI teaches)**: AI explained advanced constructor patterns and attribute design
**Part 3 (Student teaches)**: You challenged AI with edge cases about mutability and inheritance
**Part 4 (Knowledge synthesis)**: You built patterns for advanced constructors and attributes

### What You've Built

1. `attribute_types_analysis.md` — Your findings on attribute scoping
2. Summary — Understanding defaults and instance/class distinctions
3. Challenge documentation — Edge cases you posed to AI
4. `constructor_and_attributes_guide.md` — Comprehensive reference

### Next Steps

Lesson 4 builds on these foundations with encapsulation, access control (public/protected/private), and three types of methods (instance, class, static). You now understand the data layer; next you'll master method design.
