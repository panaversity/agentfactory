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
generated_by: "content-implementer v3.0.0"
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

#### 🎓 Expert Insight

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

#### 🎓 Expert Insight

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

#### 🎓 Expert Insight

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

## Challenge: Making Objects Behave Like Built-In Types

In this challenge, you'll discover why custom objects need special methods, learn how to implement them, test your understanding against AI, and build production-ready custom types.

---

## Part 1: Student Discovers the Limitation of Objects Without Special Methods

**Your Role**: Developer discovering protocol limitations

### Discovery Exercise: Try Using Custom Objects Like Built-In Types

Imagine you're building a Vector class for mathematical computations. You want to use it like built-in objects.

**Stage 1: Objects That Don't Support Operators**

Create a basic Vector class without special methods:

```python
# vector.py - NO SPECIAL METHODS
class Vector:
    """Basic 2D vector without special methods"""

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def magnitude(self) -> float:
        return (self.x ** 2 + self.y ** 2) ** 0.5


# Try to use it like built-in types
v1 = Vector(3, 4)
v2 = Vector(1, 2)

# This fails - objects don't support +
# result = v1 + v2  # TypeError: unsupported operand type(s)

# This is confusing output
print(v1)  # <__main__.Vector object at 0x7f...>

# Can't use in sets or as dict keys without issues
vectors = {v1: "magnitude_5"}  # Works but confusing

# Can't iterate
# for component in v1:  # TypeError: 'Vector' object is not iterable
```

**Your task 1**: Copy this code and document in `objects_without_protocols_analysis.md`:
- What operations fail on custom Vector? (addition, printing, iteration, etc.)
- Which built-in functions don't work (len(), iter(), etc.)?
- How would you currently add two vectors manually?
- What would make Vector "feel" like a built-in type?

**Stage 2: The Hacky Workarounds**

**Your task 2**: Try working around the limitations and document:
- How would you add two vectors without the `+` operator?
- How would you print a vector nicely?
- What operations are awkward without protocol support?
- What pattern emerges?

### Your Discovery Document

Create `special_methods_problem_statement.md` with:

1. **The Limitation Problem**: Objects don't support standard Python operations
2. **The Awkwardness Problem**: Code to work with custom objects feels unnatural
3. **The Integration Problem**: Objects don't work with built-in functions (len, iter, etc.)
4. **Your Prediction**: What Python feature would enable "magic method" behavior?

---

## Part 2: AI Teaches Special Methods as Protocol Support

**Your Role**: Student learning from AI Teacher

### AI Teaching Prompt

Ask your AI companion:

> "I have a Vector class that stores x, y coordinates. I want to:
> 1. Add vectors with + operator (v1 + v2)
> 2. Compare vectors with < (v1 < v2, by magnitude)
> 3. Get magnitude with len(v1)
> 4. Print nicely with print(v1)
> 5. Iterate: for component in v1
>
> Currently, all of these fail. Explain:
> 1. What are special methods (or magic methods)?
> 2. How do they work? Why are they called 'special'?
> 3. Show me __add__, __lt__, __len__, __str__, and __iter__ implementations
> 4. Explain when each special method gets called automatically
> 5. What happens if I implement __add__ but the other argument doesn't support it?"

### Expected AI Response Summary

AI will explain:
- **Special methods**: Dunder methods that Python calls automatically when you use operators
- **Protocol implementation**: By defining __add__, you teach Python how to handle `+` on your objects
- **Composable protocols**: __len__, __getitem__, __iter__ together let objects act like containers
- **Convention matters**: __add__ should do addition, not something else (surprises are bad)
- **Error handling**: Return NotImplemented when operation doesn't make sense

**AI will show code like**:

```python
class Vector:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __add__(self, other):
        """Support v1 + v2"""
        if not isinstance(other, Vector):
            return NotImplemented
        return Vector(self.x + other.x, self.y + other.y)

    def __str__(self):
        """Support print(v1) - user-friendly"""
        return f"Vector({self.x}, {self.y})"

    def __repr__(self):
        """Support debugging display"""
        return f"Vector({self.x!r}, {self.y!r})"

    def __len__(self):
        """Support len(v1) - magnitude"""
        return (self.x ** 2 + self.y ** 2) ** 0.5

    def __iter__(self):
        """Support for x, y in v1"""
        yield self.x
        yield self.y

# Now everything works!
v1 = Vector(3, 4)
v2 = Vector(1, 2)
print(v1 + v2)  # Vector(4, 6)
```

### Convergence Activity

After AI explains, verify understanding:

> "Show me a Container class (like a custom list) that implements __len__, __getitem__, __setitem__, and __iter__. Explain how these protocols work together to make objects behave like built-in containers. What happens if you implement __getitem__ but not __iter__?"

### Deliverable

Write 1-paragraph summary: "How Special Methods Enable Protocol-Driven Design" explaining how __add__, __len__, __iter__ and others make custom objects integrate seamlessly with Python.

---

## Part 3: Student Challenges AI with Protocol Edge Cases

**Your Role**: Student testing AI's understanding

### Challenge Design Scenarios

Ask AI to handle these cases:

#### Challenge 1: Protocol Composition

> "I implement __getitem__ for indexing and __len__ for length, but I DON'T implement __iter__. Does a for loop work? What about list comprehensions? Show me exactly what Python tries when I use for item in my_object."

**Expected learning**: AI explains that Python has fallback behaviors and how protocol requirements compose.

#### Challenge 2: NotImplemented vs TypeError

> "Show me what happens when I do vector + 5 (adding a vector and a number). Should __add__ raise TypeError immediately, return NotImplemented, or handle it? What's the difference and when does each make sense?"

**Expected learning**: AI explains protocol negotiation and error handling philosophy.

#### Challenge 3: String Representations

> "I have a Book object. I implement both __str__ and __repr__. When is each called? When should they differ? Show me examples where having different outputs is crucial for debugging."

**Expected learning**: AI explains the philosophical difference: __str__ for users, __repr__ for developers.

### Deliverable

Document your three challenges, AI's responses, and your understanding of protocol composition and error handling in special methods.

---

## Part 4: Build Feature-Complete Custom Types with Protocols

**Your Role**: Knowledge synthesizer creating production custom types

### Your Custom Types System

Create two production-ready custom types demonstrating special methods:

**vector.py**:
```python
import math
from typing import Iterator


class Vector:
    """A 2D vector supporting mathematical operations"""

    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __str__(self) -> str:
        """User-friendly representation"""
        return f"({self.x}, {self.y})"

    def __repr__(self) -> str:
        """Developer-friendly representation"""
        return f"Vector({self.x}, {self.y})"

    def __add__(self, other: 'Vector') -> 'Vector':
        """Support v1 + v2"""
        if not isinstance(other, Vector):
            return NotImplemented
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other: 'Vector') -> 'Vector':
        """Support v1 - v2"""
        if not isinstance(other, Vector):
            return NotImplemented
        return Vector(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> 'Vector':
        """Support v * scalar"""
        if isinstance(scalar, (int, float)):
            return Vector(self.x * scalar, self.y * scalar)
        return NotImplemented

    def __rmul__(self, scalar: float) -> 'Vector':
        """Support scalar * v"""
        return self * scalar

    def __eq__(self, other: object) -> bool:
        """Support v1 == v2"""
        if not isinstance(other, Vector):
            return NotImplemented
        return self.x == other.x and self.y == other.y

    def __lt__(self, other: 'Vector') -> bool:
        """Support v1 < v2 (by magnitude)"""
        if not isinstance(other, Vector):
            return NotImplemented
        return self.magnitude() < other.magnitude()

    def __len__(self) -> float:
        """Support len(v) - return magnitude"""
        return self.magnitude()

    def __iter__(self) -> Iterator[float]:
        """Support for x, y in vector"""
        yield self.x
        yield self.y

    def __hash__(self) -> int:
        """Support using vector as dict key or set member"""
        return hash((self.x, self.y))

    def magnitude(self) -> float:
        """Calculate vector magnitude"""
        return math.sqrt(self.x ** 2 + self.y ** 2)
```

**sorted_list.py**:
```python
from typing import Generic, TypeVar, Iterator, Optional

T = TypeVar('T')


class SortedList(Generic[T]):
    """A list that automatically maintains sorted order"""

    def __init__(self, items: Optional[list[T]] = None) -> None:
        """Initialize with optional items"""
        self._items: list[T] = []
        if items:
            for item in items:
                self.add(item)

    def add(self, item: T) -> None:
        """Add item while maintaining sort order"""
        # Binary insertion
        left, right = 0, len(self._items)
        while left < right:
            mid = (left + right) // 2
            if self._items[mid] < item:
                left = mid + 1
            else:
                right = mid
        self._items.insert(left, item)

    def __str__(self) -> str:
        """User-friendly representation"""
        return f"SortedList({self._items})"

    def __repr__(self) -> str:
        """Developer-friendly representation"""
        return f"SortedList({self._items!r})"

    def __len__(self) -> int:
        """Support len(sorted_list)"""
        return len(self._items)

    def __getitem__(self, index: int) -> T:
        """Support sorted_list[i]"""
        return self._items[index]

    def __contains__(self, item: T) -> bool:
        """Support item in sorted_list"""
        return item in self._items

    def __iter__(self) -> Iterator[T]:
        """Support for item in sorted_list"""
        return iter(self._items)

    def __eq__(self, other: object) -> bool:
        """Support sorted_list1 == sorted_list2"""
        if not isinstance(other, SortedList):
            return NotImplemented
        return self._items == other._items
```

**main.py**:
```python
from vector import Vector
from sorted_list import SortedList

print("=== Vector Testing ===")
v1 = Vector(3, 4)
v2 = Vector(1, 2)

print(f"v1: {v1}")
print(f"v2: {v2}")
print(f"v1 + v2: {v1 + v2}")
print(f"v1 - v2: {v1 - v2}")
print(f"v1 * 2: {v1 * 2}")
print(f"3 * v2: {3 * v2}")
print(f"v1 == Vector(3, 4): {v1 == Vector(3, 4)}")
print(f"len(v1) (magnitude): {len(v1)}")
print(f"v1 < v2: {v1 < v2}")

print("\n=== Vector Iteration ===")
for component in v1:
    print(f"  Component: {component}")

print("\n=== Vector as Dict Key ===")
magnitudes = {v1: len(v1), v2: len(v2)}
print(f"Magnitudes: {magnitudes}")

print("\n=== SortedList Testing ===")
sl = SortedList([3, 1, 4, 1, 5, 9, 2])
print(f"SortedList: {sl}")
print(f"Length: {len(sl)}")
print(f"Item at [2]: {sl[2]}")
print(f"3 in list: {3 in sl}")

print("\n=== SortedList Iteration ===")
for item in sl:
    print(f"  Item: {item}")

print("\n=== SortedList Adding ===")
sl.add(6)
print(f"After adding 6: {sl}")
sl.add(0)
print(f"After adding 0: {sl}")
```

**Your task**: Expand this system with:
1. Add Fraction class with special methods for arithmetic
2. Add Money class with comparison and string formatting
3. Create test suite validating all protocol behavior
4. Write guide: `special_methods_reference.md`

### Validation Checklist

- ✅ Vector supports +, -, *, ==, \<, len(), iteration
- ✅ SortedList supports len(), indexing, containment, iteration
- ✅ __str__ and __repr__ provide clear output
- ✅ NotImplemented returned for unsupported operations
- ✅ Objects work as dict keys (hash implemented)
- ✅ All special methods follow Python conventions

### Deliverable

Complete `vector.py` and `sorted_list.py` with comprehensive special method implementations, plus test suite demonstrating all protocols work correctly.

---

## Summary: Bidirectional Learning in Action

**Part 1 (Student discovers)**: You found that custom objects can't use operators or integrate with built-in functions

**Part 2 (AI teaches)**: AI explained special methods and how Python uses them to enable protocol-driven design

**Part 3 (Student teaches)**: You challenged AI with edge cases about protocol composition and error handling

**Part 4 (Knowledge synthesis)**: You built production custom types (Vector, SortedList) supporting multiple protocols

### What You've Built

1. `objects_without_protocols_analysis.md` — Problem analysis
2. `special_methods_problem_statement.md` — Clear problem statement
3. Challenge documentation — Three protocol edge cases
4. `vector.py` — Feature-complete Vector with 10+ special methods
5. `sorted_list.py` — Generic SortedList with container protocol
6. `main.py` — Comprehensive test suite
7. `special_methods_reference.md` — Protocol guide

### Next Steps

Lesson 5 (the capstone) integrates all Chapter 25 concepts—inheritance, polymorphism, composition, and special methods—to build complex design patterns in production systems.
