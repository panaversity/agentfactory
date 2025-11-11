---
title: "Generic Classes and Protocols"
chapter: 27
lesson: 4
duration_minutes: 40-45

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Generic Class Design"
    proficiency_level: "B1-B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create Stack[T] or Queue[T] class with full type safety for multiple types"

  - name: "Multiple Type Parameters"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can design Cache[K, V] with independent key/value types and implement get/set operations"

  - name: "Bounded Type Variables"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can constrain T to specific interfaces using bounds and explain why constraints are necessary"

  - name: "Protocol Definition and Usage"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Understand"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create a Protocol for structural typing (e.g., Comparable) and apply it as a bound"

  - name: "Architectural Decision-Making with Generics"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Evaluate"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can justify when to use Generics vs simpler alternatives based on reusability and complexity tradeoffs"

learning_objectives:
  - objective: "CREATE generic classes using Generic[T] syntax for type-safe containers"
    proficiency_level: "B1-B2"
    bloom_level: "Create"
    assessment_method: "Student builds a working Stack[T] class with push/pop/peek operations"

  - objective: "IMPLEMENT classes with multiple type parameters (Generic[K, V]) for real-world containers"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student designs a Cache[K, V] class and demonstrates type safety with different key/value combinations"

  - objective: "APPLY bounded type variables to constrain what types are allowed in generics"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student creates a bounded generic function using T: Comparable and explains the constraint"

  - objective: "DESIGN Protocols for structural subtyping in generic contexts"
    proficiency_level: "B2"
    bloom_level: "Create"
    assessment_method: "Student creates a Protocol interface and applies it as a bound to a generic class or function"

  - objective: "EVALUATE when Generics add value vs when simpler approaches suffice"
    proficiency_level: "B2"
    bloom_level: "Evaluate"
    assessment_method: "Student explains pros/cons of genericizing code and decides appropriateness for given scenarios"

cognitive_load:
  new_concepts: 10
  assessment: "10 new concepts (Generic classes, Generic[T], multiple type parameters, bounded type variables, Protocols, Protocol as bound, variance concepts, type safety in methods, real-world containers, overengineering avoidance) at B1-B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore variance (covariance/contravariance) in generic classes; implement advanced Protocol patterns with multiple methods; design a generic DataStore[T] with persistence"
  remedial_for_struggling: "Start with single-type Stack[T] before multiple parameters; focus on why bounds matter through concrete comparison examples; pair programming with AI for complex implementations"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/001-part-4-chapter-27/spec.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Generic Classes and Protocols

You've seen how generic functions like `get_first_item[T]` work with any type while preserving type information. Now comes the next level: generic **classes**. Imagine building a Stack that works with integers, strings, User objects—any type—with full type safety. Or a Cache that maps any key type to any value type. This is where Generics transform from convenient functions into powerful design patterns.

The challenge is real: without Generics, you'd write separate Stack classes for every data type you use. With Generics, you write ONE generic Stack class that adapts to any type. But there's a catch—sometimes you need to constrain what types are allowed. You can't sort items if they don't support comparison. Enter **Protocols**: a way to define structural contracts that Generics can enforce.

In this lesson, you'll learn to build type-safe, reusable generic classes that scale from simple containers to sophisticated data structures. You'll also discover Protocols—an elegant way to specify "what an object can do" without forcing inheritance hierarchies.

## Section 1: Creating a Generic Stack Class

Let's start with the most fundamental generic class pattern: a **Stack**—a container that stores items in Last-In-First-Out order (like a stack of plates).

### Building Your First Generic Class

Here's a Stack[T] that works with any type:

```python
class Stack[T]:
    """A generic stack that works with any type.

    Type parameter T ensures all items in the stack are the same type,
    and that push/pop operations preserve that type.
    """

    def __init__(self) -> None:
        """Initialize an empty stack."""
        self._items: list[T] = []

    def push(self, item: T) -> None:
        """Add an item to the top of the stack."""
        self._items.append(item)

    def pop(self) -> T | None:
        """Remove and return the top item, or None if stack is empty."""
        return self._items.pop() if self._items else None

    def peek(self) -> T | None:
        """View the top item without removing it."""
        return self._items[-1] if self._items else None

    def is_empty(self) -> bool:
        """Check if the stack is empty."""
        return len(self._items) == 0

    def size(self) -> int:
        """Return the number of items in the stack."""
        return len(self._items)


# Type-safe usage with integers
int_stack = Stack[int]()
int_stack.push(1)
int_stack.push(2)
int_stack.push(3)

top: int | None = int_stack.pop()  # Type: int | None
print(top)  # Output: 3

# Type-safe usage with strings
str_stack = Stack[str]()
str_stack.push("hello")
str_stack.push("world")

greeting: str | None = str_stack.pop()  # Type: str | None
print(greeting)  # Output: world

# Custom types work too
class Book:
    def __init__(self, title: str):
        self.title = title

book_stack = Stack[Book]()
book_stack.push(Book("Python Guide"))
book_stack.push(Book("Web Dev"))

recent_book: Book | None = book_stack.pop()  # Type: Book | None
if recent_book:
    print(recent_book.title)  # Output: Web Dev
```

**What's happening here?**

- `Stack[T]` defines a class with a type parameter T
- Each instance specifies what T is: `Stack[int]`, `Stack[str]`, `Stack[Book]`
- All methods respect that type choice—`push` only accepts T, `pop` returns T | None
- Your IDE knows the exact type at every step. It autocompletes correctly and catches type mismatches

This is fundamentally different from a non-generic Stack:

```python
# Without generics—loses type information
class UntypedStack:
    def __init__(self) -> None:
        self._items: list[Any] = []  # Could be anything

    def push(self, item: Any) -> None:
        self._items.append(item)

    def pop(self) -> Any:  # Return type unknown to IDE
        return self._items.pop() if self._items else None

# Using it—no type safety
stack = UntypedStack()
stack.push(1)
stack.push("string")  # ✅ IDE allows this (but shouldn't!)
mixed = stack.pop()  # ❌ IDE has no idea what type this is

# You could accidentally call wrong methods
if mixed:
    print(mixed.upper())  # Type checking can't catch this—might be int!
```

#### 🎓 Instructor Commentary

> Generic classes are blueprints with type parameters. When you write `Stack[int]`, you're creating a different type than `Stack[str]`. Your IDE treats them as completely separate types, catching type mismatches before you run code. This is the power of Generics: not just runtime flexibility, but design-time safety.

---

## Section 2: Multiple Type Parameters

Real-world containers often need multiple type parameters. A Cache needs a key type AND a value type. A mapping needs source type and target type. Let's see how this works.

### Creating Cache[K, V]

Here's a generic Cache that maps keys to values:

```python
class Cache[K, V]:
    """A generic cache that maps keys of type K to values of type V.

    Type parameters:
    - K: Key type (typically str, int, or hashable type)
    - V: Value type (can be any type)
    """

    def __init__(self) -> None:
        """Initialize an empty cache."""
        self._store: dict[K, V] = {}

    def set(self, key: K, value: V) -> None:
        """Store a value associated with a key."""
        self._store[key] = value

    def get(self, key: K) -> V | None:
        """Retrieve a value by key, or None if not found."""
        return self._store.get(key)

    def has(self, key: K) -> bool:
        """Check if a key exists in the cache."""
        return key in self._store

    def clear(self) -> None:
        """Remove all entries from the cache."""
        self._store.clear()

    def size(self) -> int:
        """Return the number of cached items."""
        return len(self._store)


# Cache mapping strings to integers (user IDs)
user_cache = Cache[str, int]()
user_cache.set("alice", 1001)
user_cache.set("bob", 1002)

alice_id: int | None = user_cache.get("alice")  # Type: int | None
print(alice_id)  # Output: 1001

# Cache mapping integers to objects
class User:
    def __init__(self, name: str, email: str):
        self.name = name
        self.email = email

user_object_cache = Cache[int, User]()
user_object_cache.set(1001, User("Alice", "alice@example.com"))
user_object_cache.set(1002, User("Bob", "bob@example.com"))

user: User | None = user_object_cache.get(1001)  # Type: User | None
if user:
    print(f"{user.name}: {user.email}")
    # IDE knows user has .name and .email attributes

# Cache mapping tuples to strings
location_cache = Cache[tuple[float, float], str]()
location_cache.set((37.7749, -122.4194), "San Francisco")
location_cache.set((40.7128, -74.0060), "New York")

city: str | None = location_cache.get((37.7749, -122.4194))  # Type: str | None
print(city)  # Output: San Francisco
```

**Key points:**

- K and V are independent type parameters
- Each instance fully specifies both: `Cache[str, int]`, `Cache[int, User]`, etc.
- Methods respect both types: `set(key: K, value: V)`, `get(key: K) -> V | None`
- Your IDE knows exactly what types to expect at every step

#### ✨ Teaching Tip

> Use descriptive names for type parameters: T for a single generic type, K and V for key-value pairs (standard Python convention), and U/W for additional types if needed. When reading `Cache[K, V]`, anyone familiar with Python conventions immediately understands K is keys and V is values.

---

## Section 3: Bounded Type Variables

Sometimes you need to guarantee that your generic type can do certain things. For example, a function that finds the maximum item needs to compare items. Not all types support comparison. This is where **bounded type variables** come in.

### The Problem: Comparing Unknown Types

What if you write a function to find the maximum item in a list?

```python
# ❌ This doesn't work—you can't compare arbitrary items
def find_max[T](items: list[T]) -> T | None:
    if not items:
        return None

    max_item = items[0]
    for item in items[1:]:
        if item > max_item:  # ❌ Error: can't compare T with T
            max_item = item

    return max_item
```

The problem: Python doesn't know if T supports the `>` operator. Maybe T is a type that can't be compared. To solve this, you need a **bound**—a way to say "T must support comparison."

### Creating a Comparable Protocol

First, define what "comparable" means using a Protocol:

```python
from typing import Protocol

class Comparable(Protocol):
    """Protocol for types that support comparison operations."""

    def __lt__(self, other: object) -> bool:
        """Less than operator."""
        ...

    def __le__(self, other: object) -> bool:
        """Less than or equal operator."""
        ...

    def __gt__(self, other: object) -> bool:
        """Greater than operator."""
        ...

    def __ge__(self, other: object) -> bool:
        """Greater than or equal operator."""
        ...
```

**What's this?** A Protocol doesn't inherit from anything. It just says: "Any type that implements these methods is considered Comparable." It's a structural contract: "acts like a Comparable" rather than "is-a Comparable."

#### 💬 AI Colearning Prompt

> "Ask your AI: What's the difference between a Protocol and an abstract base class (ABC)? When would you use each? Give examples where Protocols are better than inheritance."

### Using Bounded Generics

Now you can constrain T:

```python
def find_max[T: Comparable](items: list[T]) -> T | None:
    """Find the maximum item in a list of comparable items.

    Type parameter T is bounded by Comparable, meaning any type passed
    to this function must support comparison operators.
    """
    if not items:
        return None

    max_item = items[0]
    for item in items[1:]:
        if item > max_item:  # ✅ Now this is legal—T is guaranteed Comparable
            max_item = item

    return max_item


# Works with int (supports comparison)
numbers = [3, 1, 4, 1, 5, 9, 2, 6]
largest_num = find_max(numbers)  # Type: int | None
print(largest_num)  # Output: 9

# Works with str (supports comparison)
names = ["Charlie", "Alice", "Bob"]
last_name = find_max(names)  # Type: str | None
print(last_name)  # Output: Charlie

# Works with float
values = [3.14, 2.71, 1.41]
largest_value = find_max(values)  # Type: float | None
print(largest_value)  # Output: 3.14

# Custom types must implement the protocol
class Product:
    def __init__(self, price: float):
        self.price = price

    def __lt__(self, other: object) -> bool:
        if not isinstance(other, Product):
            return NotImplemented
        return self.price < other.price

    def __le__(self, other: object) -> bool:
        if not isinstance(other, Product):
            return NotImplemented
        return self.price <= other.price

    def __gt__(self, other: object) -> bool:
        if not isinstance(other, Product):
            return NotImplemented
        return self.price > other.price

    def __ge__(self, other: object) -> bool:
        if not isinstance(other, Product):
            return NotImplemented
        return self.price >= other.price

# Now find_max works with Product (implements Comparable)
products = [
    Product(29.99),
    Product(9.99),
    Product(49.99)
]
most_expensive = find_max(products)  # Type: Product | None
if most_expensive:
    print(f"Most expensive: ${most_expensive.price}")  # Output: Most expensive: $49.99
```

**What bounded generics do:**

- `T: Comparable` means "T can be any type that implements Comparable"
- The function can now safely call `>`, `<`, `==` on items of type T
- Your IDE validates that the bound is satisfied before running code
- Custom types automatically work if they implement the methods the Protocol requires

#### 🚀 CoLearning Challenge

Ask your AI Co-Teacher:
> "Create a generic `find_min[T: Comparable](items: list[T]) -> T | None` function. Show usage with int, str, and a custom Product class. Explain why the Comparable bound is necessary and what would break without it."

**Expected Outcome**: A working find_min function demonstrating bounded generics across multiple types, with clear explanation of why the bound is required.

---

## Section 4: Protocols for Structural Typing

Protocols are a powerful feature on their own. Let's understand why they're better than inheritance for Generics.

### Protocols vs Inheritance

Traditional inheritance says: "B is-a A" (tight coupling):

```python
from abc import ABC, abstractmethod

# Inheritance approach
class Drawable(ABC):
    @abstractmethod
    def draw(self) -> str:
        """Draw the shape."""
        ...

class Circle(Drawable):
    def draw(self) -> str:
        return "Drawing a circle"

class Square(Drawable):
    def draw(self) -> str:
        return "Drawing a square"

# Works, but requires explicit inheritance
def render_shape(shape: Drawable) -> None:
    print(shape.draw())

render_shape(Circle())  # ✅ Works (inherits from Drawable)
```

Protocols say: "B acts-like A" (loose coupling):

```python
from typing import Protocol

# Protocol approach
class Drawable(Protocol):
    def draw(self) -> str:
        """Draw the shape."""
        ...

class Circle:
    def draw(self) -> str:
        return "Drawing a circle"

class Square:
    def draw(self) -> str:
        return "Drawing a square"

class Triangle:
    def draw(self) -> str:
        return "Drawing a triangle"

# Works with any type that has draw() method—no inheritance required!
def render_shape(shape: Drawable) -> None:
    print(shape.draw())

render_shape(Circle())      # ✅ Works (has draw method)
render_shape(Square())      # ✅ Works (has draw method)
render_shape(Triangle())    # ✅ Works (has draw method)

# Even works with types defined elsewhere that you don't control
class LegacyShape:
    def draw(self) -> str:
        return "Drawing legacy shape"

render_shape(LegacyShape())  # ✅ Works! (has draw method, no inheritance needed)
```

**Why Protocols are better for Generics:**

- **No inheritance required**: Types automatically satisfy a Protocol if they implement the methods
- **Less coupling**: You're not locked into a class hierarchy
- **Works with external types**: Even if a type wasn't designed as Drawable, it works if it has the right methods
- **Clearer intent**: "acts-like" is more flexible than "is-a"

#### 🎓 Instructor Commentary

> Protocols represent "duck typing formalized." If it walks like a duck and quacks like a duck, Protocols say it IS a duck—without forcing it to inherit from a Duck class. This flexibility is why Protocols are standard in modern Python for Generics. Inheritance is for class hierarchies; Protocols are for specifying behavior.

### Creating a Custom Protocol

Let's define a Serializable Protocol:

```python
class Serializable(Protocol):
    """Protocol for types that can be converted to JSON."""

    def to_json(self) -> str:
        """Convert to JSON string."""
        ...

class User:
    def __init__(self, name: str, email: str):
        self.name = name
        self.email = email

    def to_json(self) -> str:
        import json
        return json.dumps({"name": self.name, "email": self.email})

class Config:
    def __init__(self, setting: str, value: str):
        self.setting = setting
        self.value = value

    def to_json(self) -> str:
        import json
        return json.dumps({"setting": self.setting, "value": self.value})

# Works with any Serializable type
def save_to_file[T: Serializable](obj: T, filename: str) -> None:
    """Save any serializable object to a JSON file."""
    with open(filename, "w") as f:
        f.write(obj.to_json())

# Both work—both implement Serializable without explicit inheritance
save_to_file(User("Alice", "alice@example.com"), "user.json")
save_to_file(Config("theme", "dark"), "config.json")
```

### Specification Reference

**Specification**: Create generic class with bounded type parameters
**Prompt Used**: "Design a Stack[T] class with push/pop/peek operations. Then show how type information flows through all methods."
**Generated Code**: Stack[T] implementation above
**Validation Steps**:
1. Verify push accepts T, pop returns T | None
2. Test with multiple types (int, str, custom class)
3. Confirm IDE provides correct autocomplete for each type
4. Ensure type errors are caught at design time, not runtime

---

## Section 5: When NOT to Use Generics

Here's the paradox: just because you CAN make something generic doesn't mean you SHOULD. Over-engineering is real.

### The Overengineering Trap

```python
# ❌ Unnecessary generic—you'll only ever use this with strings
class StringProcessor[T]:
    """Overly generic StringProcessor that only processes strings."""

    def uppercase(self, value: T) -> T:
        return value.upper()  # This only works if T is str!

    def lowercase(self, value: T) -> T:
        return value.lower()  # This only works if T is str!

# Much simpler
class StringProcessor:
    """Just process strings—no unnecessary generics."""

    def uppercase(self, value: str) -> str:
        return value.upper()

    def lowercase(self, value: str) -> str:
        return value.lower()
```

### When to Genericize

Ask these questions:

1. **Will this code actually work with multiple types?** If no, don't make it generic.
2. **Is the implementation identical for different types?** If the logic changes per type, maybe inheritance or separate classes are better.
3. **Will users of this code appreciate type safety?** If it's internal utility code, simple might beat generic.
4. **Is the added complexity worth the flexibility?** Usually yes for data structures (Stack, Queue, Cache), usually no for business logic.

**Good candidates for Generics:**

- Container classes (Stack, Queue, Cache, LinkedList)
- Repository patterns (Repository[T] for any entity type)
- Generic functions (filter, map, reduce patterns)

**Bad candidates for Generics:**

- Business logic that only uses one type
- Simple utilities (string upper/lower, number formatting)
- Classes with many type-specific methods

---

## Common Mistakes

### Mistake 1: Confusing Generic[T] (Defining) with T (Using)

```python
# ❌ Wrong—mixing definition and usage
from typing import Generic, TypeVar

T = TypeVar('T')

class Stack(Generic[T]):  # Old-style, don't use this
    def push(self, item: T) -> None:
        ...

# ✅ Correct—PEP 695 modern syntax
class Stack[T]:  # Clean, modern, preferred
    def push(self, item: T) -> None:
        ...
```

### Mistake 2: Not Constraining When You Need Specific Operations

```python
# ❌ This will error—can't call .upper() without knowing T has that method
def process[T](items: list[T]) -> list[str]:
    return [item.upper() for item in items]  # Error: T might not have .upper()

# ✅ Bound T to ensure it supports the operation
class HasUpper(Protocol):
    def upper(self) -> str: ...

def process[T: HasUpper](items: list[T]) -> list[str]:
    return [item.upper() for item in items]  # Now safe—T is guaranteed HasUpper
```

### Mistake 3: Using Generics When a Simple Type Would Do

```python
# ❌ Overengineering
class IntegerCalculator[T: int]:
    def add(self, a: T, b: T) -> T:
        return a + b

# ✅ Just use int
class IntegerCalculator:
    def add(self, a: int, b: int) -> int:
        return a + b
```

### Mistake 4: Overthinking Variance

```python
# This is advanced—don't worry about it yet
# Variance (covariance/contravariance) is about whether
# Stack[Dog] is compatible with Stack[Animal]
# Save this for advanced study—it rarely matters in practice
```

---

## Try With AI

Now let's practice generic classes and protocols with your AI companion. These prompts progress from understanding concepts to designing sophisticated generic systems.

### Prompt 1: Understand How Generic Classes Preserve Type Safety

**Ask your AI**: "Explain how generic classes work in Python using a Stack[T] example. Show how type parameters flow through all methods (push, pop, peek) and how type information is preserved. What's the advantage over using a non-generic Stack with Any type?"

**Expected Outcome**: An explanation demonstrating:
- How Stack[int] and Stack[str] are different types to the IDE
- How push(item: T) and pop() -> T | None maintain type consistency
- Concrete examples showing where IDE catches type errors with generics but not with Any
- Clear distinction between type safety at design time vs runtime flexibility

---

### Prompt 2: Apply Generics to Create a Repository[T] Class

**Tell your AI**: "Create a generic Repository[T] class with add(item: T), find_by_id(id: int) -> T | None, and get_all() -> list[T] methods. Show usage with both User and Product types, demonstrating how type safety works across different types. Include type hints and a docstring explaining the generic pattern."

**Expected Outcome**: A working Repository class with:
- Proper PEP 695 generic syntax
- CRUD operations (Create, Read, List) working with type parameter T
- Type preservation: Repository[User] knows it stores Users, Repository[Product] knows it stores Products
- Usage examples showing IDE type inference and autocomplete for each type
- Explanation of why this pattern is useful for data access layers

---

### Prompt 3: Analyze When to Add Type Bounds to Generics

**Ask your AI**: "When should you add type bounds to a generic class? Give me 3 scenarios where bounds are necessary (the class needs specific methods) and 2 scenarios where bounds are not needed (storage-only classes). For each, explain why the bound is or isn't necessary."

**Expected Outcome**: Scenarios covering:
- **Bounds necessary**: Comparison (need __lt__, __gt__), serialization (need to_json), iteration (need __iter__)
- **Bounds not necessary**: Just storing items (Stack[T]), caching objects (Cache[K, V])
- Clear explanation of the decision: "Do my methods need to call specific operations on T?"
- Examples showing what breaks without bounds and why

---

### Prompt 4: Design a PriorityQueue[T: Comparable] Class

**Tell your AI**: "Design and implement a generic PriorityQueue[T: Comparable] class. It should maintain items in priority order (highest first) with enqueue(item: T) and dequeue() -> T | None methods. Justify why the Comparable bound is necessary. Implement a concrete example using a custom Task class that implements the comparison methods. Show how the type safety works."

**Expected Outcome**: A PriorityQueue implementation with:
- Correct bounded generic syntax: `class PriorityQueue[T: Comparable]:`
- Full type hints on all parameters and return types
- Clear explanation of why Comparable bound is required (need to compare priorities)
- Working Task class implementing comparison operators
- Usage examples showing type preservation and correct priority ordering
- Comments explaining how the bound enables the comparison logic
