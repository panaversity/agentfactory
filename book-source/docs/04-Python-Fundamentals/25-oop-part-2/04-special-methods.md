---
title: "Special Methods (Magic Methods)"
chapter: 25
lesson: 4
duration_minutes: 80

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "String Representation (__str__, __repr__)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can customize how objects display in print() and interactive shell for user-friendly and debug purposes"

  - name: "Operator Overloading (__add__, __sub__, __mul__, etc.)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement arithmetic operators for custom classes, making objects behave like built-in types"

  - name: "Container Protocol (__len__, __getitem__, __setitem__, __delitem__)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can implement container methods so custom objects support len(), indexing, and item assignment like lists/dicts"

  - name: "Iteration Protocol (__iter__, __next__)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement iteration protocol to make custom objects work with for loops"

  - name: "Comparison and Hashing (__eq__, __lt__, __hash__)"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can implement comparison methods and ensure hash consistency for use in sets/dicts"

  - name: "Callable Objects (__call__)"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can understand how objects can be made callable like functions, enabling advanced patterns"

learning_objectives:
  - objective: "Implement string representation methods (__str__, __repr__) to customize object display"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create a class with both __str__ and __repr__, demonstrate user-friendly and debug output"

  - objective: "Implement operator overloading (__add__, __sub__, __mul__) to enable arithmetic with custom objects"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create Vector or Money class supporting multiple operators with type checking"

  - objective: "Implement container protocol (__len__, __getitem__, __setitem__) to make objects behave like sequences"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create custom collection class supporting indexing and length operations"

  - objective: "Implement iteration protocol (__iter__, __next__) to enable for loop iteration"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create iterable class and verify it works in for loops and comprehensions"

  - objective: "Implement comparison methods (__eq__, __lt__) and understand hash consistency requirements"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Create comparable class usable in sorted() and as dict key/set member"

cognitive_load:
  new_concepts: 10
  assessment: "10 new special method categories (at B2 limit): __str__/__repr__, __add__/__sub__/__mul__, __len__/__getitem__, __iter__/__next__, __eq__/__lt__/__hash__, __call__. Organized into 6 logical groups with progressive complexity ✓"

differentiation:
  extension_for_advanced: "Implement __radd__, __rmul__ for reverse operators; explore context managers (__enter__/__exit__); implement descriptor protocol (__get__/__set__/__delete__)"
  remedial_for_struggling: "Start with just __str__ and __repr__ to build confidence; practice simple operators (__add__ only); use templates for __getitem__ implementation"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-25.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Lesson 4: Special Methods (Magic Methods)

## Introduction: The Secret Behind Python Objects

When you write `len([1, 2, 3])` or `vector1 + vector2` or `my_dict["key"]`, you're calling **special methods** (also called "magic methods" or "dunder methods" because they have double underscores). These methods are Python's secret sauce—they let you make your custom objects behave like built-in types.

In Lessons 1-3, you learned to organize code with inheritance, polymorphism, and composition. In this lesson, you'll discover how to make objects truly Pythonic by implementing the special method protocols that Python looks for when you use operators, indexing, iteration, and more.

**Why this matters**: Professional Python code doesn't just work—it feels natural. A Vector class that supports `+` and `*`, a custom collection that supports `len()` and `for` loops—these make your APIs intuitive and readable. Special methods are how you bridge the gap between "custom class" and "feels like a built-in."

---

## Understanding Special Methods: The Protocol Perspective

Before diving into specific methods, understand a fundamental principle: **special methods define protocols**. A protocol is a contract—if your class implements certain methods, Python knows your object supports certain operations.

**Key insight**: Python doesn't check object types (`isinstance()`). It checks for behavior. This is duck typing at its core. If your object has `__len__()`, Python treats it as something with length, regardless of its class.

#### 💬 AI Colearning Prompt

> "Explain how Python's built-in list implements special methods. What happens when I do `len([1,2,3])`? When I do `[1,2,3][0]`? When I iterate with `for x in [1,2,3]`? Trace through the actual special methods that get called."

This exploration with your AI partner will deepen your understanding of the special method contracts that make Python objects consistent.

---

## String Representations: __str__ and __repr__

When you print an object or look at it in the Python interactive shell, Python calls special methods to decide how to display it.

### __str__: User-Friendly Display

`__str__()` returns a user-friendly string representation. Python calls it when you use `print()` or `str()`:

```python
class Person:
    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

    def __str__(self) -> str:
        """User-friendly string for print()"""
        return f"{self.name}, {self.age} years old"

p = Person("Alice", 30)
print(p)      # Output: Alice, 30 years old
print(str(p)) # Output: Alice, 30 years old
```

### __repr__: Developer-Friendly Display

`__repr__()` returns a developer-friendly string for debugging. Python calls it in the interactive shell or when you call `repr()`:

```python
class Person:
    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

    def __str__(self) -> str:
        return f"{self.name}, {self.age} years old"

    def __repr__(self) -> str:
        """Developer-friendly string"""
        return f"Person(name='{self.name}', age={self.age})"

p = Person("Alice", 30)
print(p)      # Output: Alice, 30 years old (calls __str__)
repr(p)       # Output: Person(name='Alice', age=30) (calls __repr__)
p             # In shell: Person(name='Alice', age=30) (calls __repr__)
```

**The convention**: `__str__()` is for end users. `__repr__()` is for developers debugging code. Ideally, `repr()` output should be valid Python code that recreates the object.

#### 🎓 Instructor Commentary

> In AI-native development, these methods become critical when you log agent state or debug multi-agent systems. A well-implemented `__repr__()` tells you exactly what you're looking at. A readable `__str__()` makes agent output feel natural to users.

---

## Operator Overloading: Making Objects Arithmetic

Special methods let you define how `+`, `-`, `*`, `/`, and other operators behave with your objects.

### Basic Operator Overloading

```python
class Vector:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __add__(self, other: 'Vector') -> 'Vector':
        """Define vector + vector"""
        if not isinstance(other, Vector):
            return NotImplemented  # Let Python handle type error
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector') -> 'Vector':
        """Define vector - vector"""
        if not isinstance(other, Vector):
            return NotImplemented
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector':
        """Define vector * scalar"""
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return Vector(self.x * scalar, self.y * scalar)

    def __str__(self) -> str:
        return f"Vector({self.x}, {self.y})"

# Test it
v1 = Vector(1, 2)
v2 = Vector(3, 4)

print(v1 + v2)   # Vector(4, 6)
print(v2 - v1)   # Vector(2, 2)
print(v1 * 3)    # Vector(3, 6)
```

**Key detail**: Return `NotImplemented` (not `None`) when an operation doesn't apply. This tells Python to try the reverse operation (`__radd__`, `__rmul__`, etc.) on the other operand.

#### 💬 AI Colearning Prompt

> "Show me all the operator overload special methods Python supports: __add__, __sub__, __mul__, __truediv__, __floordiv__, __mod__, __pow__. Create a Money class and explain when I'd use __radd__ instead of __add__. What's the difference?"

This deeper exploration helps you understand the full operator landscape and when reverse operators matter.

---

## Container Protocol: Indexing and Length

If you want your object to behave like a list or dictionary, implement these methods:

### __len__ and __getitem__

```python
class Playlist:
    def __init__(self):
        self._songs: list[str] = []

    def add_song(self, song: str) -> None:
        """Add a song to the playlist"""
        self._songs.append(song)

    def __len__(self) -> int:
        """Support len(playlist)"""
        return len(self._songs)

    def __getitem__(self, index: int) -> str:
        """Support playlist[0] for reading"""
        if not isinstance(index, int):
            raise TypeError(f"list indices must be integers, not {type(index).__name__}")
        return self._songs[index]

    def __setitem__(self, index: int, song: str) -> None:
        """Support playlist[0] = 'new song' for assignment"""
        if not isinstance(index, int):
            raise TypeError(f"list indices must be integers, not {type(index).__name__}")
        self._songs[index] = song

    def __delitem__(self, index: int) -> None:
        """Support del playlist[0]"""
        if not isinstance(index, int):
            raise TypeError(f"list indices must be integers, not {type(index).__name__}")
        del self._songs[index]

# Test it
playlist = Playlist()
playlist.add_song("Song 1")
playlist.add_song("Song 2")
playlist.add_song("Song 3")

print(len(playlist))         # 3
print(playlist[0])           # Song 1
playlist[1] = "Updated Song 2"
print(playlist[1])           # Updated Song 2
del playlist[2]
print(len(playlist))         # 2
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Create a Range class that mimics Python's built-in range() function. Implement __iter__ and __next__ for iteration. Then add __len__ and __getitem__ to support len(my_range) and my_range[0]. Explain how all these methods work together."

**Expected Outcome**: You'll understand how container protocols layer on top of each other—iteration, length, and indexing are separate contracts.

---

## Iteration Protocol: Making Objects Loop-Able

The iteration protocol lets objects work in `for` loops and list comprehensions.

### __iter__ and __next__

```python
class Countdown:
    def __init__(self, start: int):
        self.current = start

    def __iter__(self):
        """Return an iterator (often self)"""
        return self

    def __next__(self) -> int:
        """Return the next value in iteration"""
        if self.current <= 0:
            raise StopIteration  # Signal end of iteration
        self.current -= 1
        return self.current + 1

# Test it
for num in Countdown(5):
    print(num)  # Prints: 5, 4, 3, 2, 1
```

**Critical pattern**: `__iter__()` returns an iterator object (often `self`). `__next__()` returns the next value and raises `StopIteration` when done. This is the protocol Python's `for` loop expects.

#### 🎓 Instructor Commentary

> In AI-native development, iteration protocols enable elegant APIs. Imagine an AgentQueue that yields agents in priority order, or a DataStream that yields batches of training data. Proper iteration protocols make your systems read naturally: `for agent in queue:`.

---

## Comparison and Equality: __eq__, __lt__, __hash__

These methods define how objects compare and how they behave in sets and dictionaries.

### Equality and Ordering

```python
from functools import total_ordering

@total_ordering  # Fills in missing comparison methods
class Person:
    def __init__(self, name: str, age: int):
        self.name = name
        self.age = age

    def __eq__(self, other) -> bool:
        """Define equality (==)"""
        if not isinstance(other, Person):
            return False
        return self.age == other.age

    def __lt__(self, other) -> bool:
        """Define less-than (<)"""
        if not isinstance(other, Person):
            return NotImplemented
        return self.age < other.age

    def __hash__(self) -> int:
        """Define hash for use in sets/dicts"""
        return hash((self.name, self.age))

    def __repr__(self) -> str:
        return f"Person(name='{self.name}', age={self.age})"

# Test it
people = [Person("Alice", 30), Person("Bob", 25), Person("Charlie", 35)]

# Sorting uses __lt__
sorted_people = sorted(people)
print(sorted_people)  # [Person(name='Bob', age=25), Person(name='Alice', age=30), Person(name='Charlie', age=35)]

# Using in sets and dicts uses __hash__ and __eq__
unique_people = set(people)
print(len(unique_people))  # 3

person_ages = {p: p.age for p in people}
print(person_ages)
```

**Critical rule**: If you implement `__eq__()`, you MUST implement `__hash__()`. Objects that compare equal must have the same hash. Otherwise, they can't be used in sets or as dictionary keys.

#### ✨ Teaching Tip

> Use Claude Code to explore hash consistency: "Create a class where __eq__ compares by name but __hash__ includes age. Show me why this breaks sets/dicts. Then fix it by making both compare the same way."

---

## Callable Objects: __call__

The `__call__()` method makes an object callable like a function. This enables advanced patterns where objects store state and behavior together.

```python
class Multiplier:
    def __init__(self, factor: int):
        self.factor = factor

    def __call__(self, x: int) -> int:
        """Make the instance callable"""
        return x * self.factor

# Create callable objects
double = Multiplier(2)
triple = Multiplier(3)

# Call them like functions
print(double(5))    # 10
print(triple(5))    # 15
print(double(100))  # 200
```

**Use case**: Create decorator-like objects that maintain state. Or create factory functions that remember configuration.

Another example—a decorator that counts calls:

```python
class CallCounter:
    def __init__(self, func):
        self.func = func
        self.call_count = 0

    def __call__(self, *args, **kwargs):
        self.call_count += 1
        print(f"Call #{self.call_count}")
        return self.func(*args, **kwargs)

def greet(name: str) -> str:
    return f"Hello, {name}!"

tracked_greet = CallCounter(greet)
tracked_greet("Alice")    # Call #1 → Hello, Alice!
tracked_greet("Bob")      # Call #2 → Hello, Bob!
print(tracked_greet.call_count)  # 2
```

#### 💬 AI Colearning Prompt

> "Show me 3 real-world use cases for __call__. How would callable classes help in a multi-agent system? Could agents themselves be callable objects that process messages?"

Explore with your AI partner how callable objects enable sophisticated design patterns.

---

## Context Managers: __enter__ and __exit__ (Brief Introduction)

Context managers use `__enter__()` and `__exit__()` to manage resources (files, database connections, locks). You've likely used them with `with` statements:

```python
with open('file.txt') as f:
    content = f.read()  # File is automatically closed after this block
```

Here's a minimal example:

```python
class DatabaseConnection:
    def __init__(self, connection_string: str):
        self.connection_string = connection_string
        self.is_connected = False

    def __enter__(self):
        """Called when entering 'with' block"""
        print(f"Connecting to {self.connection_string}")
        self.is_connected = True
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Called when exiting 'with' block (even if exception)"""
        print("Closing connection")
        self.is_connected = False
        return False  # Don't suppress exceptions

# Use it
with DatabaseConnection("postgres://localhost/db") as db:
    print(f"Connected: {db.is_connected}")
    # Connection automatically closes here
```

**Key idea**: `__exit__()` is guaranteed to run, even if an exception occurs. This makes it perfect for cleanup operations.

We'll dive deeper into context managers in a later chapter. For now, recognize the pattern: special methods let Python manage object lifecycles elegantly.

---

## Putting It All Together: Building a Complete Custom Type

Let's combine multiple special methods to create a practical, Pythonic class:

```python
class Money:
    """A Money class supporting arithmetic and comparison"""

    def __init__(self, amount: float, currency: str = "USD"):
        if amount < 0:
            raise ValueError("Amount cannot be negative")
        self.amount = amount
        self.currency = currency

    def __str__(self) -> str:
        """User-friendly display"""
        return f"${self.amount:.2f} {self.currency}"

    def __repr__(self) -> str:
        """Debug display"""
        return f"Money({self.amount}, '{self.currency}')"

    def __add__(self, other: 'Money') -> 'Money':
        """Add two Money objects"""
        if not isinstance(other, Money):
            return NotImplemented
        if self.currency != other.currency:
            raise ValueError(f"Cannot add {self.currency} and {other.currency}")
        return Money(self.amount + other.amount, self.currency)

    def __eq__(self, other) -> bool:
        """Check equality"""
        if not isinstance(other, Money):
            return False
        return self.amount == other.amount and self.currency == other.currency

    def __lt__(self, other) -> bool:
        """Compare amounts (assumes same currency)"""
        if not isinstance(other, Money):
            return NotImplemented
        if self.currency != other.currency:
            raise ValueError(f"Cannot compare {self.currency} and {other.currency}")
        return self.amount < other.amount

    def __hash__(self) -> int:
        """Hash for sets/dicts"""
        return hash((self.amount, self.currency))

# Test it
wallet = Money(100.00)
purchase = Money(25.50)

total = wallet + purchase
print(total)                           # $125.50 USD
print(repr(total))                     # Money(125.5, 'USD')
print(wallet == Money(100.00))         # True
print(wallet < Money(200.00))          # True

# Use in sets
unique_amounts = {Money(100), Money(100), Money(50)}
print(len(unique_amounts))             # 2 (duplicates removed by hash/eq)
```

#### 🎓 Instructor Commentary

> This Money class demonstrates why special methods matter. Without them, arithmetic on currency would be clunky: `Money.add(wallet, purchase)`. With special methods, it's natural: `wallet + purchase`. Natural syntax is professional code.

---

## Common Patterns and Best Practices

### Pattern 1: Type Checking in Special Methods

Always check types and return `NotImplemented` for unsupported operations:

```python
def __add__(self, other):
    if not isinstance(other, Vector):
        return NotImplemented
    # ... proceed with addition
```

### Pattern 2: Implementing __repr__ Correctly

Strive to make `repr()` output valid Python:

```python
def __repr__(self) -> str:
    return f"Money({self.amount}, '{self.currency}')"
    # This string can be eval()'d to recreate the object
```

### Pattern 3: Hash Consistency

If two objects are equal, they MUST have the same hash:

```python
def __eq__(self, other):
    return self.id == other.id

def __hash__(self):
    return hash(self.id)  # Must match __eq__
```

### Pattern 4: Raise StopIteration in __next__

Signal the end of iteration by raising `StopIteration`, not by returning `None`:

```python
def __next__(self):
    if self.is_done:
        raise StopIteration  # Correct
    return self.next_value
```

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "I created a custom class with __eq__ but forgot to implement __hash__. Show me what breaks when I try to put instances in a set. Then explain why Python requires hash consistency. Design a fix."

**Expected Outcome**: You'll understand the subtle contract between `__eq__` and `__hash__` and why breaking it causes mysterious bugs.

---

## Try With AI

**Tool**: Claude Code, Gemini CLI, or your preferred AI companion tool

Use this 4-prompt set to practice special methods with your AI partner. Start with simple cases and progress toward complex integration.

### Prompt 1: Recall - Basic Special Methods

```
Create a Book class with:
- __init__(title, author, isbn)
- __str__() returning "Title by Author"
- __repr__() returning "Book('Title', 'Author', 'isbn-123')"
- __eq__() comparing by ISBN

Test all three methods and show the output.
```

**Expected Outcome**: You understand the distinction between `__str__` (readable) and `__repr__` (debuggable), and how equality works for domain objects.

---

### Prompt 2: Understand - Operator Overloading

```
Create a Temperature class that represents temperatures in Celsius:
- __init__(celsius: float)
- __str__() returning "X°C"
- __add__() adding two temperatures (return new Temperature)
- __sub__() subtracting two temperatures
- __lt__() comparing temperatures
- __eq__() checking equality

Test: temp1 = Temperature(20), temp2 = Temperature(30)
- temp1 + temp2 should return Temperature(50)
- temp1 < temp2 should return True
- temp1 == Temperature(20) should return True

Show your code and test output.
```

**Expected Outcome**: You can implement multiple operators and understand operator protocol, including type checking and `NotImplemented`.

---

### Prompt 3: Apply - Container Protocol

```
Create a Stack class (Last In, First Out) with:
- __init__() initializing empty stack
- push(item) adding to stack
- pop() removing and returning from stack
- __len__() supporting len(stack)
- __getitem__(index) supporting stack[0] to peek (without removing)
- __iter__() supporting for loops

Test:
- Create stack, push 3 items
- Use len(stack) to check size
- Use stack[0] to peek at top
- Iterate: for item in stack
- Pop an item and verify

Show complete code and test output.
```

**Expected Outcome**: You understand how container protocols compose—iteration, length, and indexing are distinct contracts, and a class can implement multiple simultaneously.

---

### Prompt 4: Synthesize - Building a Custom Collection

```
Create a SortedList class that automatically keeps items sorted:
- __init__(items=None) initializing with optional list
- add(item) adding item while maintaining sort order
- __len__() returning number of items
- __getitem__(index) supporting indexing
- __contains__(item) supporting 'in' operator (check if item exists)
- __iter__() supporting for loops (iterate in sorted order)
- __repr__() for debugging

Implement this WITHOUT using Python's sorted() in __init__ or add()—build it from scratch.

Test with:
- Create SortedList([3, 1, 4, 1, 5])
- Add 2 (should go in correct sorted position)
- Check len(), access items by index, iterate, check membership
- Print repr()

Show complete code and detailed test output explaining how sorting is maintained.
```

**Expected Outcome**: You understand how to build production-ready custom types that feel like built-in Python objects, implementing multiple protocols together with careful state management.

---

**Safety and Ethics Note**: When implementing special methods, remember that users of your class rely on standard Python behavior. A Vector `+` operation should behave like mathematical vector addition, not something unexpected. Breaking conventions confuses other developers (and your future self). Honor the principle: "Explicit is better than implicit."
