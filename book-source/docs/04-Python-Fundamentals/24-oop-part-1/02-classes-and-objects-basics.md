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
generated_by: "lesson-writer v1.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-24.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Classes and Objects Basics

## From Blueprints to Real Buildings

In Lesson 1, you learned that a **class is a blueprint** and an **object is something built from that blueprint**. Now you're going to build your first class from scratch and create actual objects.

Think of it like architecture. An architect draws a blueprint for a house. The blueprint (class) specifies: "This house has 3 bedrooms, 2 bathrooms, a kitchen." But the blueprint itself isn't a house—it's the instructions for building houses. When a contractor builds the house (instantiation), they create a real, physical house (object) with actual bedrooms, bathrooms, and kitchen.

In this lesson, you'll learn the syntax for creating blueprints (classes) and building from them (objects). You'll also understand the mysterious `self` keyword that appears in every class.

---

## Defining Your First Class

The simplest class is just a name and the keyword `class`:

```python
class Dog:
    pass  # Empty class for now
```

That's it! You've created a class named `Dog`. Let's break this down:

- **`class` keyword**: Tells Python you're defining a class
- **`Dog`**: The class name (always PascalCase - capital first letter, no underscores)
- **`pass`**: Python placeholder meaning "there's nothing here yet, but the class exists"

This empty class isn't very useful, but it's valid Python. You can already create objects from it:

```python
my_dog = Dog()  # Create an object from the Dog class
print(my_dog)   # <__main__.Dog object at 0x...>
print(type(my_dog))  # <class '__main__.Dog'>
```

The object exists, but it has no data (attributes) or behavior (methods). Let's fix that.

---

## The `__init__` Constructor Method

When you create an object from a class, Python automatically calls a special method named `__init__` (pronounced "dunder init"). This is your **constructor** — it initializes the object when it's born.

```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name
        self.breed = breed
```

Let's understand each part:

- **`def __init__(self, ...):`**: Defines the constructor. It's a method (function inside a class)
- **`self`**: Represents the object being created. It's always the first parameter in instance methods
- **`name: str, breed: str`**: Parameters the constructor accepts (with type hints!)
- **`self.name = name`**: Creates an attribute called `name` on this object and stores the parameter value
- **`self.breed = breed`**: Creates an attribute called `breed` on this object

#### 💬 AI Colearning Prompt

> Ask your AI Co-Teacher: "What happens in memory when we call `Dog('Max', 'Labrador')`? Walk me through step-by-step, starting with Python creating the object, then executing `__init__`."

This helps you visualize how objects are created before you see them in action.

---

## Creating Objects (Instantiation)

Now that the `Dog` class has a constructor, we can create objects with actual data:

```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name
        self.breed = breed

# Create first object
dog1 = Dog("Max", "Labrador")
print(dog1.name)    # Max
print(dog1.breed)   # Labrador

# Create second object
dog2 = Dog("Buddy", "Golden Retriever")
print(dog2.name)    # Buddy
print(dog2.breed)   # Golden Retriever

# Critical: dog1 and dog2 are separate objects with separate data
print(dog1.name)    # Max (still!)
print(dog2.name)    # Buddy (different)
```

Notice something important: Even though both objects came from the same class (blueprint), they have **different data**. `dog1` and `dog2` each have their own `name` and `breed` attributes.

This is the power of classes: One blueprint, many objects, each with independent state.

#### 🎓 Instructor Commentary

> In AI-native development, you don't memorize `self` syntax—you understand WHAT it means: "this specific object." When an AI chatbot agent processes messages, `self.conversation_history` means "this agent's conversation," not all agents'. Different agents need different conversation histories. Classes let you model that.

---

## Understanding `self`: The Instance Reference

The word `self` confuses many beginners. Here's the truth: **`self` is just a variable name that refers to the current object being operated on.**

When you call `dog1.name`, Python is actually doing this:

```python
# What you write:
dog1.name

# What Python does internally (conceptually):
Dog.name(dog1)  # Call the name attribute on the dog1 object
```

More importantly, inside a method:

```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name  # "Set MY name to this value"
        self.breed = breed

dog1 = Dog("Max", "Labrador")
dog2 = Dog("Buddy", "Golden Retriever")

# When we created dog1, Python:
# 1. Created a new Dog object
# 2. Called __init__ with self=dog1
# 3. self.name = "Max" means dog1.name = "Max"

# When we created dog2, Python:
# 1. Created a new Dog object
# 2. Called __init__ with self=dog2
# 3. self.name = "Buddy" means dog2.name = "Buddy"
```

**Why does Python require `self` explicitly?** Because Python wants you to be explicit about which object you're talking to. Other languages (like Java or C++) hide this behind the scenes, but Python says "be clear: this method operates on this specific object."

---

## Adding Simple Methods

Methods are just functions that live inside a class. They operate on the object's data:

```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name
        self.breed = breed

    def bark(self) -> str:
        """Make the dog bark"""
        return f"{self.name} says: Woof!"

    def describe(self) -> str:
        """Describe the dog"""
        return f"{self.name} is a {self.breed}"

# Use the methods
dog1 = Dog("Max", "Labrador")
print(dog1.bark())        # Max says: Woof!
print(dog1.describe())    # Max is a Labrador

dog2 = Dog("Buddy", "Golden Retriever")
print(dog2.bark())        # Buddy says: Woof!
print(dog2.describe())    # Buddy is a Golden Retriever
```

Notice:

- **Methods always have `self` as the first parameter** (even though you don't pass it when calling)
- **Methods use `return` to give back values**, just like functions
- **Type hints work on methods too** (`-> str` means it returns a string)
- When you call `dog1.bark()`, Python automatically passes `dog1` as the `self` parameter

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Generate a Vehicle class with brand, model, and year attributes. Add a display_info() method that returns a formatted string like 'Toyota Camry (2024)'. Include type hints on everything. Then explain: why are type hints important in classes that will be used by other developers or AI tools?"

**Expected Outcome**: You'll practice both writing classes and understanding why modern Python requires type hints for clarity.

#### ✨ Teaching Tip

> Use Claude Code to explore how Python's built-in `list` class works: "Show me the source code for Python's list class and explain how methods like append() work internally. How does append() use self?"

This helps you see that professional Python code uses these same patterns.

---

## Built-in Functions vs Methods

Here's a distinction you need to understand: **built-in functions** work on any object, but **methods** belong to specific classes.

```python
# Built-in functions: work on many types
name = "Max"
print(len(name))           # 3 - len() works on strings, lists, dicts
print(type(dog1))          # <class '__main__.Dog'> - type() works on everything

# Methods: belong to specific objects
dog = Dog("Max", "Labrador")
print(dog.bark())          # "Max says: Woof!" - only Dog objects have bark()
print(dog.describe())      # "Max is a Labrador" - only Dog objects have describe()

# Calling a method
dog.bark()                 # Method call: object.method()

# If you tried this, it would fail:
# len(dog1)                # TypeError! len() doesn't work on Dog objects
```

The pattern is:

- **Functions**: `function(object)` — like `len(string)`, `type(dog)`, `print(value)`
- **Methods**: `object.method()` — like `dog.bark()`, `string.upper()`, `list.append(item)`

---

## Putting It All Together: A More Realistic Class

Let's build a practical `BankAccount` class that combines everything you've learned:

```python
class BankAccount:
    """A simple bank account with deposit and withdrawal"""

    def __init__(self, account_holder: str, initial_balance: float = 0.0):
        """Initialize account with holder name and starting balance"""
        self.account_holder = account_holder
        self.balance = initial_balance

    def deposit(self, amount: float) -> None:
        """Add money to the account"""
        self.balance += amount
        print(f"{self.account_holder} deposited ${amount}. New balance: ${self.balance:.2f}")

    def withdraw(self, amount: float) -> bool:
        """Remove money if sufficient funds exist"""
        if amount <= self.balance:
            self.balance -= amount
            print(f"{self.account_holder} withdrew ${amount}. New balance: ${self.balance:.2f}")
            return True
        else:
            print(f"Insufficient funds for {self.account_holder}. Balance: ${self.balance:.2f}")
            return False

    def get_balance(self) -> float:
        """Return current balance"""
        return self.balance

# Create accounts
alice = BankAccount("Alice", 1000)
bob = BankAccount("Bob", 500)

# Use the accounts
alice.deposit(200)                    # Alice deposited $200. New balance: $1200.00
success = bob.withdraw(100)           # Bob withdrew $100. New balance: $400.00
print(f"Withdrawal successful: {success}")  # Withdrawal successful: True

failed = bob.withdraw(500)            # Insufficient funds for Bob. Balance: $400.00
print(f"Withdrawal successful: {failed}")   # Withdrawal successful: False

# Check independence of objects
print(f"Alice's balance: ${alice.get_balance()}")  # Alice's balance: $1200.00
print(f"Bob's balance: ${bob.get_balance()}")      # Bob's balance: $400.00
```

This class demonstrates:

- **Constructor with defaults**: `initial_balance = 0.0` is optional
- **Multiple methods**: `deposit()`, `withdraw()`, `get_balance()`
- **Type hints throughout**: on parameters and return values
- **docstrings**: Explain what each method does
- **Object independence**: Alice's account and Bob's account are separate

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "I want to extend the BankAccount class with a transaction history. Add a list attribute to store transaction records. Modify deposit() and withdraw() to add entries like 'Deposited $200 on 2025-11-09 14:35:22'. Show me how to implement this."

**Expected Outcome**: You'll see how to work with more complex attributes (lists of data) inside classes.

#### ✨ Teaching Tip

> When debugging classes, use `print()` to inspect objects. Create a BankAccount and ask your AI: "Show me how to inspect what attributes this object has. What's the __dict__ attribute and how can I use it to debug class instances?"

---

## Common Mistakes to Avoid

**Mistake 1: Forgetting `self` in methods**

```python
# ❌ WRONG
class Dog:
    def bark():  # Missing self!
        return "Woof!"

# ✅ CORRECT
class Dog:
    def bark(self) -> str:  # Always include self
        return "Woof!"
```

**Mistake 2: Not calling the constructor**

```python
# ❌ WRONG - this doesn't call __init__
dog = Dog  # This is the class, not an object

# ✅ CORRECT - use parentheses to call __init__
dog = Dog("Max", "Labrador")  # This creates an object
```

**Mistake 3: Confusing class names with variable names**

```python
# ❌ Confusing
class dog:  # Lowercase (wrong!)
    pass

# ✅ CORRECT
class Dog:  # PascalCase (right!)
    pass

dog1 = Dog()  # Variable name can be lowercase
```

**Mistake 4: Accessing attributes without an object**

```python
class Dog:
    def __init__(self, name: str):
        self.name = name

# ❌ WRONG - Dog.name doesn't exist without an object
print(Dog.name)  # AttributeError!

# ✅ CORRECT - use an object
my_dog = Dog("Max")
print(my_dog.name)  # Max
```

---

## Try With AI

Use your AI companion (Claude Code or Gemini CLI). You're reinforcing class creation and object independence through hands-on practice.

### Prompt 1: Recall - Class Syntax

```
Write a simple Person class with name and age attributes. Include type hints.
Then create two person objects and print their attributes to show they're independent.
```

**Expected outcome**: You'll write a working class with a constructor and verify object independence.

### Prompt 2: Understand - The Self Keyword

```
Explain what 'self' means in Python classes. Why do we write 'self.name = name' in __init__?
What happens if we forget self? Show me an example that fails without self and works with it.
```

**Expected outcome**: You'll understand the self parameter deeply and see how it connects to object identity.

### Prompt 3: Apply - Building Your First Functional Class

```
Create a BankAccount class with account_holder (str) and balance (float) attributes.
Add a deposit(amount) method that adds to balance and returns the new balance.
Test it: create two accounts, deposit different amounts, verify they have separate balances.
```

**Expected outcome**: You'll build a functional class with data and behavior, and verify that objects are truly independent.

### Prompt 4: Analyze - Object Independence

```
If I create two BankAccount objects (account1 and account2), and I set account1.balance = 500,
does account2.balance change? Why or why not? Explain with a diagram or description of what's
happening in memory.
```

**Expected outcome**: You'll understand that each object has its own attribute storage and that modifications to one don't affect others.
