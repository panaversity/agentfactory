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

  - name: "AI-Assisted Class Design"
    proficiency_level: "B1"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Communication & Collaboration"
    measurable_at_this_level: "Student can describe class requirements to AI and validate generated code for correctness"

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

  - objective: "Add methods to a class and call them on objects"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Student creates a method and calls it on an instance"

cognitive_load:
  new_concepts: 7
  assessment: "7 new concepts (class syntax, __init__, self, instantiation, attributes, methods, type hints) at B1 level. At maximum for B1 proficiency ✓"

differentiation:
  extension_for_advanced: "Extend Dog class with @property decorators for computed attributes (e.g., age_in_human_years). Create a Dog class hierarchy (breed-specific subclasses with unique methods)."
  remedial_for_struggling: "Start with empty class (pass) before adding constructor. Use lots of print statements to trace what self is. Show how dog1.name and dog2.name are different memory locations."

# Generation metadata
generated_by: "content-implementer v1.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-26.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Classes and Objects Basics

In Lesson 1, you discovered WHY OOP exists. Now you'll learn the syntax for HOW to create classes and objects through hands-on experimentation, AI-guided instruction, challenging the AI, and synthesizing your understanding into reusable patterns.

---

## Part 1: Experience Classes by Experimentation

**Your Role**: Code explorer discovering how Python implements objects

Before reading syntax explanations, you'll experiment with creating classes to develop intuition.

### Discovery Exercise: Build Classes Step by Step

**What is a class?** A class is a blueprint (template) for creating objects. Think of it like a cookie cutter: the class is the cutter shape, and objects are the individual cookies you make from it. The `class` keyword in Python defines this blueprint.

**Stage 1: The Simplest Possible Class**

Let's create the simplest possible class - a blueprint with no details yet.

Create a file called `class_experiments.py` and run this code:

```python
# The absolute minimum: a class with no content
class Dog:
    pass

# Now create an object from this class
my_dog = Dog()

# What is my_dog? Inspect it
print(type(my_dog))
print(my_dog)
print(dir(my_dog))  # What attributes/methods does it have?
```

**Your task 1**: Run this and document:
- What does `type(my_dog)` tell you?
- What does `print(my_dog)` show you? (Pay attention to the memory address)
- What's in `dir(my_dog)`? What default methods does Python give every object?

#### 💬 AI CoLearning Prompt

> "I created an empty class with just `pass`. When I inspect `dir(my_dog)`, I see lots of methods like `__init__`, `__str__`, `__dict__`. Where do these come from? What's the relationship between a class and an object? Explain using a cookie-cutter analogy."

**Expected Understanding**: AI will explain that classes are blueprints/templates (cookie cutter), objects are instances created from them (cookies). Python gives every object default methods automatically. You'll understand the class-object relationship before coding more.

---

![Anatomy diagram showing Python class structure with constructor, attributes, methods, self parameter, and type hints](/img/part-5/chapter-28/python-class-structure-anatomy.png)

**Stage 2: Add Data to Objects**

**Pedagogical Note**: In this stage, we'll show you a way to add data to objects that WORKS but isn't the professional pattern. We're doing this deliberately so you understand WHY the proper pattern (constructors with `__init__`) exists. Discovery through contrast is powerful.

Now modify the code:

```python
class Dog:
    pass

my_dog = Dog()
# Add attributes directly to the object (unusual, but possible in Python!)
my_dog.name = "Max"
my_dog.breed = "Labrador"

print(f"Name: {my_dog.name}")
print(f"Breed: {my_dog.breed}")

# Create a second object
other_dog = Dog()
other_dog.name = "Buddy"
other_dog.breed = "Golden Retriever"

print(f"First dog: {my_dog.name} ({my_dog.breed})")
print(f"Second dog: {other_dog.name} ({other_dog.breed})")

# Question: If I change my_dog.name, does other_dog.name change?
my_dog.name = "Max Jr."
print(f"After changing my_dog:")
print(f"First dog name: {my_dog.name}")
print(f"Second dog name: {other_dog.name}")
```

**Your task 2**: Run this and answer:
- How do you add data to an object in Python?
- Are the objects independent? Prove it.
- What happens if you inspect `my_dog.__dict__`? (This shows all attributes)

#### 💬 AI CoLearning Prompt

> "I'm adding attributes to objects using `my_dog.name = 'Max'`. This works, but I have to repeat this for EVERY dog I create. For 100 dogs, that's 300+ lines of attribute assignment! How do professional developers avoid this repetition? What Python feature lets me initialize attributes automatically when creating an object? Give me a preview of the solution before I learn the syntax."

**Expected Understanding**: AI will preview the `__init__` method (constructor) that automatically runs when you create an object. You'll see WHY constructors exist before learning HOW they work. This motivates the learning in Part 2.

---

**Stage 3: Notice the Problem with Manual Attributes**

You've seen that adding attributes manually works. Now let's see why it's problematic at scale.

```python
class Dog:
    pass

# This works, but is repetitive and error-prone
dog1 = Dog()
dog1.name = "Max"
dog1.breed = "Labrador"
dog1.age = 5

dog2 = Dog()
dog2.name = "Buddy"
dog2.breed = "Golden Retriever"
dog2.age = 3

# What if you create 100 dogs? You'd repeat setup code 100 times
# What if you forget to set breed for a dog? (Silent error - no type safety)
# How do you FORCE every Dog object to have name, breed, age with correct types?
```

**Your task 3**: Before reading further, predict:
- What would solve the repetition problem?
- How could you ensure every Dog object MUST have name: str, breed: str, age: int?
- What language feature would enforce this at object creation time?

**Answer Preview**: The solution is the constructor (`__init__` method) with type hints. You'll learn this in the next section. This discovery exercise showed you the PAIN that constructors solve.

---

### Your Discovery Summary

Instead of manual documentation, **use AI to synthesize** what you learned:

#### 💬 AI CoLearning Prompt

> "Based on my experiments with creating classes and adding attributes manually, summarize these key insights:
> 1. What's the relationship between classes and objects? (blueprint vs instance)
> 2. How are objects independent? (each has its own memory)
> 3. What's the pain point with manual attribute assignment? (repetition, no enforcement)
> 4. What's the solution I'm about to learn? (constructors with __init__)
>
> Give me 3 concise bullet points I can reference in Part 2."

**Deliverable**: Save AI's synthesis in your notes. You've discovered the problem—now you're ready to learn the solution (`__init__` method) in Part 2.

---

## Part 2: Learn Class Syntax and Constructors

**Your Role**: Student receiving instruction from AI Teacher

Now that you've experienced the limitations of manual attribute setting, it's time to learn the solution.

![Anatomical breakdown of Python class structure showing class definition, __init__ constructor, instance attributes, instance methods, and the role of self parameter in connecting methods to object instances](/img/part-5/chapter-28/python-class-structure-anatomy.png)

### AI Teaching Prompt

Ask your AI companion:

> "I've been creating Dog objects and manually adding name, breed, age attributes to each one. This is repetitive:
> ```python
> dog1 = Dog()
> dog1.name = 'Max'
> dog1.breed = 'Labrador'
> dog1.age = 5
>
> dog2 = Dog()
> dog2.name = 'Buddy'
> # ... repeat for every dog ...
> ```
>
> How can I force every Dog to automatically initialize with name, breed, age when created? Show me the syntax and explain:
> 1. What is the `__init__` method?
> 2. What is `self`?
> 3. Why does Python require explicit `self` parameter?
> 4. Show me the same dog-creating code but using __init__"

### What You'll Learn from AI

**Expected AI Response** (summary):

- **`__init__` method**: A special constructor that Python automatically calls when you create an object
- **`self`**: Represents "the object being created." It's always the first parameter
- **`self.name = name`**: Creates an attribute on THIS object
- **Why explicit self?**: Python wants you to be clear: you're setting attributes on a specific object

**AI will show you**:

```python
class Dog:
    def __init__(self, name: str, breed: str, age: int):
        self.name = name      # "Set THIS dog's name"
        self.breed = breed    # "Set THIS dog's breed"
        self.age = age        # "Set THIS dog's age"

# Now creation is much simpler!
dog1 = Dog("Max", "Labrador", 5)
dog2 = Dog("Buddy", "Golden Retriever", 3)

print(dog1.name)  # Max
print(dog2.name)  # Buddy
```

### Convergence Activity

After AI explains, **test your understanding**:

Ask AI: "Show me what happens in memory when I call `Dog('Max', 'Labrador', 5)`. Walk through step-by-step: Python creates the object, then calls `__init__` with what self value?"

**Deliverable**: Write a summary explaining `__init__`, `self`, and why this solves the repetition problem compared to manual attribute setting.

---

## Part 3: Challenge AI with Edge Cases

**Your Role**: Student teaching AI by testing its understanding

Now you'll design scenarios that test whether AI understands subtle aspects of classes and objects.

### Challenge Design Pattern

#### Challenge 1: Object Independence

**Your prompt to AI**:

> "I create two Dog objects:
> ```python
> dog1 = Dog("Max", "Labrador", 5)
> dog2 = Dog("Buddy", "Golden Retriever", 3)
> dog1.age = 10
> ```
>
> After this, what are:
> - `dog1.age`?
> - `dog2.age`?
>
> Why didn't dog2.age change? Explain what's happening in memory that makes them separate."

**Expected learning**: AI will explain that each object has its own memory space for attributes. This is the fundamental power of objects.

#### Challenge 2: Shared Blueprint, Different Instances

**Your prompt to AI**:

> "I have:
> ```python
> dogs = []
> for i in range(1000):
>     dogs.append(Dog(f"Dog{i}", "Mixed", i % 20))
> ```
>
> How many times did Python execute the Dog class definition code? How many times did it execute `__init__`? Why is this efficient?"

**Expected learning**: AI will explain that the class is defined once, but `__init__` runs for each object creation.

#### Challenge 3: Default Parameters in Constructors

**Your prompt to AI**:

> "Show me a Dog class where age is optional with a default value of 0. So:
> - `Dog('Max', 'Labrador', 5)` creates a 5-year-old dog
> - `Dog('Buddy', 'Golden')` creates a dog with age 0
>
> What's the syntax for default parameters in `__init__`?"

### Deliverable

Document your three challenges, AI's responses, and your analysis of whether AI's explanation was correct and complete.

---

## Part 4: Build Your Class Design Pattern

**Your Role**: Knowledge synthesizer creating design templates

Now integrate everything into reusable patterns for designing classes.

### Your Class Design Template

Create a markdown file called `class_design_patterns.md` with this structure:

```markdown
# Class Design Patterns

## Pattern 1: Simple Data Container (Basic Constructor)

**When to use**: Class whose main job is storing related data

**Template**:
```python
class Person:
    def __init__(self, name: str, age: int, email: str):
        self.name = name
        self.age = age
        self.email = email
```

**Key points**:
- Constructor parameters match attributes
- Type hints on parameters
- Simple, focused initialization

---

## Pattern 2: Constructor with Defaults

**When to use**: Some attributes are optional or have sensible defaults

**Template**:
```python
class Dog:
    def __init__(self, name: str, breed: str, age: int = 0):
        self.name = name
        self.breed = breed
        self.age = age
```

**Key points**:
- Required parameters first
- Optional parameters last (with defaults)
- Defaults are usually for less critical data

---

## Pattern 3: Constructor with Validation

**When to use**: Data must meet certain constraints

**Template**:
```python
class BankAccount:
    def __init__(self, holder: str, initial_balance: float):
        if initial_balance < 0:
            raise ValueError("Balance cannot be negative")
        self.holder = holder
        self.balance = initial_balance
```

**Key points**:
- Check constraints in `__init__`
- Raise exceptions for invalid data
- Prevent invalid object creation

---

## Pattern 4: Adding Methods (Behavior)

**When to use**: Objects need to perform actions, not just store data

**Template**:
```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name
        self.breed = breed

    def bark(self) -> str:
        """Return a bark string"""
        return f"{self.name} says: Woof!"

    def get_info(self) -> str:
        """Return info about this dog"""
        return f"{self.name} is a {self.breed}"
```

**Key points**:
- Methods always have `self` as first parameter
- Methods use `self.attribute` to access object's data
- Include type hints on return types (`-> str`)
- Use docstrings to explain method purpose

---

## Pattern 5: Object Independence Verification

**How to verify objects are truly independent**:

```python
# Create two objects
dog1 = Dog("Max", "Labrador")
dog2 = Dog("Buddy", "Golden Retriever")

# Modify one
dog1.name = "Max Jr."

# Verify other is unchanged
assert dog2.name == "Buddy"  # ✓ dog2 is independent
assert dog1.name == "Max Jr."  # ✓ dog1 changed correctly
```

**Why this matters**: Confirms that each object has its own memory space.

---

## Pattern 6: The Self Parameter Explained

**Memory model**:
```python
class Dog:
    def __init__(self, name: str):
        self.name = name  # "THIS dog's name"

dog1 = Dog("Max")
dog2 = Dog("Buddy")

# When you call dog1.bark():
# Python mentally does: Dog.bark(dog1)
# "self" inside the method refers to dog1
```

**Key insight**: `self` = "the specific object this method is operating on"

---

## Pattern 7: Constructor Pattern with Multiple Attributes

**Full example with all patterns**:

```python
class BankAccount:
    """A bank account with holder, balance, and transaction tracking"""

    def __init__(self, holder: str, initial_balance: float = 0.0):
        """
        Initialize a bank account

        Args:
            holder: Name of account holder
            initial_balance: Starting balance (default 0.0)
        """
        # Validate inputs
        if initial_balance < 0:
            raise ValueError("Balance cannot be negative")

        # Initialize attributes
        self.holder = holder
        self.balance = initial_balance
        self.transactions = []

    def deposit(self, amount: float) -> float:
        """Add money and return new balance"""
        self.balance += amount
        self.transactions.append(f"Deposit: +${amount}")
        return self.balance

    def withdraw(self, amount: float) -> bool:
        """Remove money if funds available, return success"""
        if amount > self.balance:
            return False
        self.balance -= amount
        self.transactions.append(f"Withdraw: -${amount}")
        return True

    def get_balance(self) -> float:
        """Return current balance"""
        return self.balance
```

---

## Quick Reference: Class vs Instance

**Class**: Blueprint, created once
```python
class Dog:           # ← This is the class (blueprint)
    def __init__(self, name):
        self.name = name
```

**Instance**: Actual object, created many times
```python
dog1 = Dog("Max")    # ← dog1 is an instance (object)
dog2 = Dog("Buddy")  # ← dog2 is a different instance
```

---

## Common Mistakes and Fixes

### Mistake 1: Forgetting `self` in methods
```python
# ❌ WRONG
def bark():
    return "Woof"

# ✅ CORRECT
def bark(self):
    return "Woof"
```

### Mistake 2: Not calling constructor
```python
# ❌ WRONG - Dog without ()
dog = Dog

# ✅ CORRECT - Dog with ()
dog = Dog("Max", "Labrador")
```

### Mistake 3: Forgetting `self.` when accessing attributes
```python
# ❌ WRONG
def describe(self):
    return name  # Where does 'name' come from?

# ✅ CORRECT
def describe(self):
    return self.name  # THIS object's name
```

---

## Testing Pattern: Verify Independence

**Always test that objects are independent**:

```python
account1 = BankAccount("Alice", 1000)
account2 = BankAccount("Bob", 500)

account1.deposit(200)  # Alice: 1200, Bob: still 500

assert account1.balance == 1200
assert account2.balance == 500  # ← Unchanged!
```
```

### Validation with AI

Once your patterns are complete, validate them:

> "Review my class design patterns. Are my explanations of self and object independence accurate? What patterns am I missing? What common mistakes should I add?"

**Deliverable**: Complete `class_design_patterns.md` with patterns and quick reference guide.

---

## Try With AI

Ready to master class syntax and understand how objects manage independent state?

**🔍 Explore `__init__` and `self`:**
> "Explain __init__ and self like I'm 10 years old. Then show me a Car class with brand, model, year attributes. Create three car objects. When I call car1.display_info(), how does Python know to show car1's data, not car2's? Trace through what 'self' means in memory."

**🎯 Practice Class Design:**
> "Help me build a Product class for an e-commerce system. It needs: name, price, stock_count attributes; and methods: apply_discount(percentage), restock(quantity), is_available(). Guide me through: what goes in __init__, what are instance methods, how to validate inputs (negative price/stock)."

**🧪 Test Object Independence:**
> "Create a Counter class with value attribute (starts at 0) and increment() method. Make three counters. Increment the first twice, the second once, the third zero times. Predict final values. Then explain: why don't all counters share the same value? What makes each object independent?"

**🚀 Apply to Data Modeling:**
> "I'm building [describe your system: library, blog, game, etc.]. Help me identify one core entity and design its class. Walk me through: what attributes describe it? What actions can it perform? What should __init__ accept as parameters vs set as defaults?"

---
