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
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-24.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Constructors, Destructors, and Attributes

Now that you can create basic classes with simple constructors, we'll move into more sophisticated initialization patterns. You'll learn how to give your objects multiple parameters, provide sensible defaults, understand the critical difference between class and instance attributes, and properly manage object cleanup when objects are destroyed.

These concepts are foundational for building robust, maintainable Python classes. Whether you're building a simple data class or a complex AI agent, understanding how objects initialize and store data will transform the quality of your code.

## Parameterized Constructors with Defaults

In Lesson 2, you saw simple constructors with required parameters. Real-world classes often need **optional parameters** with sensible defaults.

### Constructors with Required Parameters

Let's start with a constructor that requires all arguments:

```python
class Car:
    def __init__(self, make: str, model: str, year: int):
        self.make = make
        self.model = model
        self.year = year

# Must provide all three arguments
car1 = Car("Toyota", "Camry", 2024)
car2 = Car("Honda", "Civic", 2023)

print(car1.make)  # Toyota
```

### Adding Default Parameters

Now add defaults for flexibility:

```python
class Car:
    def __init__(self, make: str = "Unknown", model: str = "Unknown", year: int = 2024):
        self.make = make
        self.model = model
        self.year = year

# Can provide all arguments
car1 = Car("Toyota", "Camry", 2024)

# Or use defaults
car2 = Car()  # Uses all defaults: Unknown, Unknown, 2024

# Or provide some arguments
car3 = Car("Honda", "Civic")  # Uses default year: 2024
```

The pattern is **required parameters first, then optional parameters with defaults**.

#### 💬 AI Colearning Prompt
> "When should we use default parameters in constructors? Give me 3 scenarios where defaults are helpful and 1 where they're dangerous."

---

## Class Attributes vs Instance Attributes

This is a **critical concept** that confuses many beginners. Let's clarify:

### Understanding the Difference

**Instance attributes** are unique to each object. Every instance of a class has its own copy:

```python
class Dog:
    def __init__(self, name: str, breed: str):
        self.name = name      # Instance attribute
        self.breed = breed    # Instance attribute

dog1 = Dog("Max", "Labrador")
dog2 = Dog("Buddy", "Golden Retriever")

print(dog1.name)  # Max
print(dog2.name)  # Buddy - different value!
```

Each dog has its own `name`. They don't share.

**Class attributes** are shared across ALL instances of a class:

```python
class BankAccount:
    interest_rate = 0.03  # Class attribute (shared by all accounts)

    def __init__(self, holder: str, balance: float = 0.0):
        self.holder = holder    # Instance attribute
        self.balance = balance  # Instance attribute

account1 = BankAccount("Alice", 1000)
account2 = BankAccount("Bob", 2000)

# All accounts share the same interest rate
print(account1.interest_rate)  # 0.03
print(account2.interest_rate)  # 0.03 (same value)

# Changing the class attribute affects all instances
BankAccount.interest_rate = 0.04
print(account1.interest_rate)  # 0.04 - changed!
print(account2.interest_rate)  # 0.04 - changed!
```

### The Shadowing Problem (Critical!)

Here's where it gets tricky. If you assign to an attribute through an instance, you create an **instance attribute** that shadows (hides) the class attribute:

```python
class BankAccount:
    interest_rate = 0.03  # Class attribute

    def __init__(self, holder: str):
        self.holder = holder

account1 = BankAccount("Alice")
account2 = BankAccount("Bob")

print(account1.interest_rate)  # 0.03 (using class attribute)

# Now assign through instance - creates NEW instance attribute!
account1.interest_rate = 0.05

print(account1.interest_rate)  # 0.05 (instance attribute)
print(account2.interest_rate)  # 0.03 (still using class attribute)
print(BankAccount.interest_rate)  # 0.03 (class attribute unchanged)
```

This is **not a bug—it's by design**. Python first looks for instance attributes, then class attributes.

#### 🎓 Instructor Commentary
> "The class vs instance attribute distinction is CRITICAL for AI agents. Configuration settings (API keys, model names, timeouts) are often class attributes shared across all agents of that type. But conversation history is an instance attribute—each agent needs its own separate history."

#### 💬 AI Colearning Prompt
> "Explain why modifying a class attribute through an instance (like `account1.interest_rate = 0.05`) creates a new instance attribute instead of modifying the class attribute. What's happening in memory?"

---

## Inspecting Attributes with `__dict__`

When debugging, you need to see what attributes an object actually has. Python provides `__dict__` for this:

```python
class Person:
    species = "Human"  # Class attribute

    def __init__(self, name: str, age: int):
        self.name = name  # Instance attribute
        self.age = age    # Instance attribute

p = Person("Alice", 30)

# See instance attributes only
print(p.__dict__)
# Output: {'name': 'Alice', 'age': 30}

# Class attributes aren't in instance __dict__
print("species" in p.__dict__)  # False

# But you can still access via instance
print(p.species)  # Human
```

`__dict__` shows only instance attributes, not class attributes. This is useful for debugging—if an attribute isn't in `__dict__`, it's a class attribute.

#### ✨ Teaching Tip
> "Use `__dict__` to debug attribute issues. When an attribute isn't showing up where you expect, check `__dict__`. Ask Claude: 'Why isn't my attribute showing up in __dict__?'"

---

## Destructors: The `__del__` Method

Just as `__init__` is called when an object is **created**, `__del__` (destructor) is called when an object is **destroyed** (garbage collected).

### When Do You Need Destructors?

Destructors are useful for **cleanup**: closing files, disconnecting from databases, releasing network connections, freeing memory.

### A FileHandler Example

```python
class FileHandler:
    def __init__(self, filename: str):
        self.filename = filename
        self.file = open(filename, 'w')
        print(f"Opened {filename}")

    def write(self, data: str):
        self.file.write(data)

    def __del__(self):
        if hasattr(self, 'file'):  # Make sure file exists
            self.file.close()
            print(f"Closed {self.filename}")

# Create handler
handler = FileHandler("test.txt")
handler.write("Hello, World!")

# When handler goes out of scope, __del__ is called automatically
del handler  # Explicit deletion
# Output: "Closed test.txt"
```

### Important: Destructors Are Unreliable!

While `__del__` seems convenient, **it's not guaranteed to be called immediately**:
- Python's garbage collector decides when to clean up
- Exceptions might prevent `__del__` from running
- Circular references can delay cleanup indefinitely

**Better approach**: Use **context managers** (covered in Part 4 later):

```python
# Better: Use 'with' statement for guaranteed cleanup
with open("test.txt", 'w') as file:
    file.write("Hello")
# File GUARANTEED to close here, no matter what
```

#### 🚀 CoLearning Challenge
> "Ask your AI: Create a DatabaseConnection class that connects in `__init__` and disconnects in `__del__`. Then explain why `__del__` is NOT reliable for critical cleanup (hint: what happens if an exception occurs?). What's the better approach?"

---

## Putting It Together: A Real Example

Let's build a `Product` class that uses all these concepts:

```python
class Product:
    # Class attributes (shared by all products)
    tax_rate = 0.1  # 10% tax
    product_count = 0  # Track total products created

    def __init__(self, name: str, price: float, quantity: int = 1):
        # Instance attributes (unique to each product)
        self.name = name
        self.price = price  # Must be positive
        self.quantity = quantity

        # Increment class attribute
        Product.product_count += 1

    @property
    def total_price(self) -> float:
        """Computed property: price including tax"""
        return self.price * self.quantity * (1 + Product.tax_rate)

    def __str__(self) -> str:
        return f"{self.name}: ${self.price} x {self.quantity}"

    def __del__(self):
        """Cleanup when product is destroyed"""
        Product.product_count -= 1
        print(f"Removed {self.name} from inventory")

# Create products
p1 = Product("Laptop", 1000.0, quantity=2)
p2 = Product("Mouse", 25.0)

print(Product.product_count)  # 2 - class attribute tracks total
print(p1.total_price)  # 2200.0 (1000 * 2 * 1.1)

print(p1.__dict__)  # Shows instance attributes only
# Output: {'name': 'Laptop', 'price': 1000.0, 'quantity': 2}
```

**Notice**:
- `tax_rate` and `product_count` are class attributes (shared)
- `name`, `price`, `quantity` are instance attributes (unique per product)
- We access `Product.product_count` (class-level) to increment a counter
- `__del__` decrements the counter when products are removed
- `__dict__` shows only instance attributes

---

## Common Mistakes to Avoid

### Mistake 1: Forgetting Default Parameters Come Last

```python
# ❌ WRONG - required parameter after default
def __init__(self, name: str = "Unknown", age: int):
    pass
# SyntaxError: non-default argument follows default argument

# ✅ CORRECT - defaults come after required
def __init__(self, name: str, age: int = 0):
    pass
```

### Mistake 2: Modifying Mutable Class Attributes

```python
# ❌ DANGEROUS
class Student:
    courses: list[str] = []  # Class attribute - MUTABLE!

    def __init__(self, name: str):
        self.name = name

s1 = Student("Alice")
s1.courses.append("Python")

s2 = Student("Bob")
print(s2.courses)  # ["Python"] - BOTH students share the list!

# ✅ SAFE - Mutable data belongs in instances
class Student:
    def __init__(self, name: str):
        self.name = name
        self.courses: list[str] = []  # Instance attribute
```

### Mistake 3: Relying on `__del__` for Critical Cleanup

```python
# ❌ RISKY
class DatabaseConnection:
    def __init__(self, host: str):
        self.conn = connect_to_db(host)

    def __del__(self):
        self.conn.close()  # Might never be called!

# ✅ SAFE - Use context managers
class DatabaseConnection:
    def __enter__(self):
        self.conn = connect_to_db(self.host)
        return self

    def __exit__(self, *args):
        self.conn.close()  # Guaranteed to be called
```

---

## Try With AI

Use your AI companion (Claude Code or Gemini CLI). In these prompts, you'll practice managing object initialization and attribute scoping.

**Prompt 1: Recall - Constructor Types**

```
Create a Product class with name, price, and quantity.
Provide default values: price=0.0, quantity=0.
Show how to create a product with all values specified vs using defaults.
```

**Expected outcome**: You'll master default parameters in constructors and understand when to use them.

---

**Prompt 2: Understand - Class vs Instance Attributes**

```
I have a VideoGame class where all games share the same 'platform' (e.g., "PC").
Should 'platform' be a class attribute or instance attribute?
What if each game can be on multiple platforms?
```

**Expected outcome**: You'll learn to choose the right attribute scope based on data semantics (shared vs unique).

---

**Prompt 3: Apply - Debugging with `__dict__`**

```
Write a Student class with name, grade, and courses (list).
Create a student object.
Use __dict__ to inspect its attributes.
Then add a class attribute 'school' and check if it appears in the instance's __dict__.
```

**Expected outcome**: You'll use `__dict__` for debugging and understanding how Python stores attributes.

---

**Prompt 4: Analyze - Destructor Use Cases**

```
When is __del__ useful for resource cleanup?
When is it dangerous or unreliable?
Give me a better alternative for guaranteed cleanup in Python.
```

**Expected outcome**: You'll understand destructors and their limitations, and learn about context managers as a safer alternative.
