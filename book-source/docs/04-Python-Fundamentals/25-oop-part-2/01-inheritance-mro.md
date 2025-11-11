---
title: "Inheritance and Method Resolution Order"
chapter: 25
lesson: 1
duration_minutes: 70

# HIDDEN SKILLS METADATA (Institutional Integration Layer)
# Not visible to students; enables competency assessment and differentiation
skills:
  - name: "Single Inheritance"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can create parent-child class relationships and call parent methods"

  - name: "super() Function Usage"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can properly call parent methods and explain why super() is superior to direct class calling"

  - name: "Multiple Inheritance"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Information Literacy"
    measurable_at_this_level: "Student can understand diamond inheritance patterns and their implications"

  - name: "Method Resolution Order (MRO)"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Understand"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can explain how Python searches for methods in inheritance hierarchies"

  - name: "C3 Linearization Algorithm"
    proficiency_level: "B2"
    category: "Conceptual"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can understand why C3 linearization is used and how it preserves order"

  - name: "Method Overriding"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Content Creation"
    measurable_at_this_level: "Student can replace parent methods with specialized behavior in child classes"

  - name: "AI-Assisted Hierarchy Design"
    proficiency_level: "B2"
    category: "Soft"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can use AI to visualize and refine inheritance hierarchies"

  - name: "Debugging Inheritance with __mro__"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Analyze"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can inspect inheritance chains and understand method lookup order"

learning_objectives:
  - objective: "Create inheritance hierarchies using super() and explain parent-child relationships"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Creating a multi-level inheritance hierarchy with proper super() usage"

  - objective: "Explain Method Resolution Order (MRO) and how Python finds methods in complex hierarchies"
    proficiency_level: "B2"
    bloom_level: "Understand"
    assessment_method: "Explaining MRO output for a diamond inheritance example"

  - objective: "Apply inheritance and MRO to solve real problems while recognizing when composition is better"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Evaluating design decisions in code examples"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (Single inheritance, super(), Multiple inheritance, Diamond pattern, MRO, C3 linearization, Method overriding, __mro__ debugging) within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Explore why C3 linearization is used instead of simpler algorithms; implement custom MRO visualization tool"
  remedial_for_struggling: "Focus on single inheritance first; use simplified diamond diagrams before exploring full MRO"

# Generation metadata
generated_by: "lesson-writer v3.0.0"
source_spec: "specs/020-oop-part-1-2/spec-chapter-25.md"
created: "2025-11-09"
last_modified: "2025-11-09"
git_author: "Claude Code"
workflow: "/sp.implement"
version: "1.0.0"
---

# Inheritance and Method Resolution Order

In Chapter 24, you built the foundations of object-oriented programming: classes that encapsulate data and behavior, methods that transform state, and principles of good design. Now we're stepping into the professional OOP world where you design systems with multiple related classes that share behavior and specialize behavior through **inheritance**.

Inheritance is the mechanism that lets you create hierarchies of classes, from general to specific. A `Dog` **is-a** `Animal`, an `ElectricCar` **is-a** `Car`. This "is-a" relationship is powerful: it lets you reuse code, create flexible designs, and build systems where many different types work through a common interface. But inheritance is also subtle. Get it wrong, and you'll spend hours debugging which method actually got called.

In this lesson, you'll master the technical skill of inheritance and the conceptual skill of **Method Resolution Order (MRO)**: the mechanism that answers the question "When I call `dog.speak()`, which speak() method does Python actually execute?" Understanding MRO is the difference between writing confident inheritance code and staring at confusing behavior in multi-level hierarchies.

---

## Single Inheritance: The Parent-Child Relationship

Let's start simple. The most common pattern is **single inheritance**: a child class has one parent.

```python
class Animal:
    """Parent class (superclass) - general animal properties"""

    def __init__(self, name: str) -> None:
        self.name = name

    def speak(self) -> str:
        return "Some generic sound"


class Dog(Animal):
    """Child class (subclass) - specializes Animal"""

    def __init__(self, name: str, breed: str) -> None:
        super().__init__(name)  # Call parent constructor
        self.breed = breed

    def speak(self) -> str:
        return f"{self.name} says: Woof!"


dog = Dog("Max", "Labrador")
print(dog.speak())  # Max says: Woof!
print(dog.name)     # Max (inherited from Animal)
```

Notice the syntax: `class Dog(Animal):` means "Dog is a child of Animal" and `super().__init__(name)` calls the parent's `__init__` method.

#### 💬 AI Colearning Prompt

> "Explain exactly what happens when we call `super().__init__(name)` in the Dog constructor. What if we forgot to call it? Show me the difference in memory state."

---

## The super() Function: Calling Parent Methods

The **`super()`** function is a gateway to parent class methods. It's more sophisticated than you might think.

```python
class Vehicle:
    def __init__(self, make: str) -> None:
        self.make = make
        print(f"Vehicle initialized: {make}")

    def describe(self) -> str:
        return f"A {self.make}"


class Car(Vehicle):
    def __init__(self, make: str, doors: int) -> None:
        super().__init__(make)  # Call parent constructor FIRST
        self.doors = doors
        print(f"Car initialized with {doors} doors")

    def describe(self) -> str:
        # Call parent describe() and extend it
        parent_description = super().describe()
        return f"{parent_description} with {self.doors} doors"


car = Car("Toyota", 4)
# Output:
# Vehicle initialized: Toyota
# Car initialized with 4 doors
print(car.describe())  # A Toyota with 4 doors
```

The key insight: **`super()` respects the parent's initialization**, ensuring both parent and child set up their state correctly. This is critical for avoiding bugs.

#### 🎓 Instructor Commentary

> In AI-native development, inheritance hierarchies model agent types and capabilities. Understanding super() prevents subtle bugs where initialization gets skipped. In multi-agent systems, a misconfigured agent might seem to have a capability it doesn't actually have because its parent's initialization was skipped.

#### 🚀 CoLearning Challenge

> Ask your AI Co-Teacher: "I have a Person class with age. I create a Manager subclass that adds salary. Write the __init__ methods correctly using super(). Then explain what happens if I forget the super() call."

---

## Method Overriding: Specialization

When a child class provides its own version of a parent method, that's **method overriding**. The child's version **replaces** the parent's version:

```python
class Shape:
    def area(self) -> float:
        return 0.0


class Circle(Shape):
    def __init__(self, radius: float) -> None:
        self.radius = radius

    def area(self) -> float:
        return 3.14159 * self.radius ** 2


class Rectangle(Shape):
    def __init__(self, width: float, height: float) -> None:
        self.width = width
        self.height = height

    def area(self) -> float:
        return self.width * self.height


shapes: list[Shape] = [Circle(5), Rectangle(4, 6)]
for shape in shapes:
    print(f"Area: {shape.area()}")
```

This code works because Python uses **polymorphism**: the same method name (`area()`) works differently on different object types. Python looks at the actual object type and calls the appropriate version. A `Circle`'s `area()` calculates circles differently from a `Rectangle`'s `area()`, but the caller doesn't need to know which type they have.

---

## Multiple Inheritance: Two Parents

Python allows a class to inherit from **multiple parents**:

```python
class Flyer:
    def fly(self) -> str:
        return "Flying high!"


class Swimmer:
    def swim(self) -> str:
        return "Swimming fast!"


class Duck(Flyer, Swimmer):
    def __init__(self, name: str) -> None:
        self.name = name

    def quack(self) -> str:
        return f"{self.name} says: Quack!"


duck = Duck("Donald")
print(duck.fly())   # Flying high!
print(duck.swim())  # Swimming fast!
print(duck.quack()) # Donald says: Quack!
```

A Duck inherits both flying and swimming abilities. This is powerful but introduces complexity: what happens when two parents have methods with the same name?

---

## The Diamond Problem: Multiple Paths to the Same Parent

Here's where MRO becomes critical. Imagine this inheritance structure:

```
       A (has greet() method)
      / \
     B   C (both override greet())
      \ /
       D
```

This is called the **diamond problem** because of its shape. When `D` inherits from both `B` and `C`, and both inherit from `A`, we have two inheritance paths to `A`:
- Path 1: D → B → A
- Path 2: D → C → A

If we call `d.greet()`, which version gets called? B's or C's? And does A's `greet()` get called?

```python
class A:
    def greet(self) -> str:
        return "Hello from A"


class B(A):
    def greet(self) -> str:
        return "Hello from B"


class C(A):
    def greet(self) -> str:
        return "Hello from C"


class D(B, C):
    pass


d = D()
print(d.greet())  # Which one?
print(D.mro())    # Let's see the Method Resolution Order
```

Run this code and you'll see:
```
Hello from B
[<class 'D'>, <class 'B'>, <class 'C'>, <class 'A'>, <class 'object'>]
```

The MRO tells us: Check D, then B, then C, then A, then object. That's the order Python searches for methods. Since B is listed first (left parent in `class D(B, C)`), B's `greet()` is found and executed.

#### 💬 AI Colearning Prompt

> "Explain why Python searches D → B → C → A in that order instead of D → B → A → C. What principle ensures this order makes sense?"

---

## Method Resolution Order (MRO): The Deep Dive

**MRO** is the ordered list of classes Python searches to find a method. Python uses an algorithm called **C3 Linearization** to compute MRO. Here's the principle:

1. **Subclasses before parents** — Always check D before B before A
2. **Inheritance order preserved** — If a class inherits from (B, C), search B before C
3. **No class visited twice** — Once A is in the search order, it appears exactly once, at the deepest level

#### 🎓 Instructor Commentary

> C3 Linearization prevents the chaos of older languages where the diamond problem could cause the same parent method to be called twice. Python solved this elegantly: every class appears exactly once, in a consistent order. This matters profoundly in AI agent hierarchies where you might have BaseAgent → SpecializedAgent → (ChatMixin, ToolMixin) → SomeAgent. Without C3, you'd have ambiguous behavior.

Here's a more complex example to see MRO in action:

```python
class Vehicle:
    def start(self) -> str:
        return "Vehicle starting"


class Car(Vehicle):
    def start(self) -> str:
        return "Car ignition sequence"


class Boat(Vehicle):
    def start(self) -> str:
        return "Boat engine sequence"


class AmphibiousCar(Car, Boat):
    pass


amp = AmphibiousCar()
print(amp.start())  # Car starting
print(AmphibiousCar.mro())
```

Output:
```
Car ignition sequence
[<class 'AmphibiousCar'>, <class 'Car'>, <class 'Boat'>, <class 'Vehicle'>, <class 'object'>]
```

The MRO is: AmphibiousCar → Car → Boat → Vehicle → object. Since Car comes before Boat (the inheritance order in `class AmphibiousCar(Car, Boat)`), Car's `start()` is found first and called.

#### 🚀 CoLearning Challenge

> Create a diamond inheritance example (Device, Phone adds call(), Computer adds compute(), Smartphone inherits from both). Add a start() method in Device and override it in Phone and Computer. Call start() from a Smartphone instance. Then ask AI to explain the MRO and why Python found the method it found.

---

## Inspecting MRO with __mro__ and mro()

Python gives you two ways to inspect MRO:

```python
# Method 1: __mro__ attribute (tuple)
print(AmphibiousCar.__mro__)
# (<class 'AmphibiousCar'>, <class 'Car'>, <class 'Boat'>, <class 'Vehicle'>, <class 'object'>)

# Method 2: mro() method
print(AmphibiousCar.mro())
# [<class 'AmphibiousCar'>, <class 'Car'>, <class 'Boat'>, <class 'Vehicle'>, <class 'object'>]
```

Both return the same information. When debugging inheritance, print the MRO to understand the search order:

```python
class Agent:
    def process(self, msg: str) -> str:
        return "Agent processing"


class ChatMixin:
    def process(self, msg: str) -> str:
        return "ChatMixin processing"


class ToolMixin:
    def process(self, msg: str) -> str:
        return "ToolMixin processing"


class SmartAgent(Agent, ChatMixin, ToolMixin):
    pass


smart = SmartAgent()
print(f"MRO: {SmartAgent.mro()}")
print(f"Result: {smart.process('hello')}")
```

Output:
```
MRO: [<class 'SmartAgent'>, <class 'Agent'>, <class 'ChatMixin'>, <class 'ToolMixin'>, <class 'object'>]
Result: Agent processing
```

Agent is searched first, so Agent's `process()` is called. If you want a different priority, change the inheritance order to `class SmartAgent(ChatMixin, Agent, ToolMixin):`.

#### ✨ Teaching Tip

> Use Claude Code to visualize MRO: "Draw the inheritance tree for this class hierarchy and show me the method lookup order step-by-step." This visual aid clarifies why certain methods are called.

---

## When NOT to Use Inheritance: A Design Perspective

Inheritance is powerful but can be misused. Here's an anti-pattern:

```python
# ANTI-PATTERN: Wrong inheritance
class Engine:
    def start(self) -> str:
        return "Engine started"


class Car(Engine):  # WRONG! A Car is not an Engine
    pass
```

A Car is not an Engine; a Car **has** an Engine. This is a **composition** problem, not an inheritance problem. More on this in Lesson 3. For now, remember: inheritance should represent true "is-a" relationships.

---

## Code Specification and Validation

**Specification**: Create a multi-level inheritance hierarchy (Vehicle → Car → SportsCar) with proper super() usage, method overriding, and MRO demonstration.

**Prompt Used**:
```
Create Vehicle class with make and engine_type.
Create Car(Vehicle) that adds doors.
Create SportsCar(Car) that adds top_speed.
All classes have describe() method that calls super().describe() and adds their own info.
Show MRO and test all three classes.
```

**Generated Code** (tested on Python 3.13+):

```python
class Vehicle:
    """Base class for all vehicles"""

    def __init__(self, make: str, engine_type: str) -> None:
        self.make = make
        self.engine_type = engine_type

    def describe(self) -> str:
        return f"Vehicle: {self.make} ({self.engine_type})"


class Car(Vehicle):
    """A vehicle with doors"""

    def __init__(self, make: str, engine_type: str, doors: int) -> None:
        super().__init__(make, engine_type)
        self.doors = doors

    def describe(self) -> str:
        parent_desc = super().describe()
        return f"{parent_desc}, {self.doors} doors"


class SportsCar(Car):
    """A car built for speed"""

    def __init__(
        self,
        make: str,
        engine_type: str,
        doors: int,
        top_speed: int
    ) -> None:
        super().__init__(make, engine_type, doors)
        self.top_speed = top_speed

    def describe(self) -> str:
        parent_desc = super().describe()
        return f"{parent_desc}, {self.top_speed} mph top speed"


# Test
print(SportsCar.mro())
print()

vehicle = Vehicle("Generic", "Petrol")
print(vehicle.describe())  # Vehicle: Generic (Petrol)

car = Car("Toyota", "Hybrid", 4)
print(car.describe())  # Vehicle: Toyota (Hybrid), 4 doors

sports = SportsCar("Ferrari", "V12", 2, 220)
print(sports.describe())  # Vehicle: Ferrari (V12), 2 doors, 220 mph top speed
```

**Validation**:
- ✅ Single inheritance chain: Vehicle → Car → SportsCar
- ✅ Each class calls super().__init__() correctly
- ✅ Method overriding works: describe() builds on parent descriptions
- ✅ MRO output shows proper linear ordering
- ✅ All code tested on Python 3.13+

---

## Try With AI

Use your preferred AI companion (Claude Code, Gemini CLI, or ChatGPT web) to complete these prompts in order. They progress from recalling basic syntax to analyzing design decisions.

### Prompt 1: Recall - Inheritance Syntax

**Copy this prompt to your AI:**

```
Create an Employee base class with name (string) and salary (float).
Create a Manager subclass that adds department (string).
Use super().__init__() in Manager's constructor to call the parent constructor.
Both classes should have a describe() method.
Create instances and test them.
```

**Expected Outcome**: Working code with proper super() usage where Manager can call parent's __init__ and override describe().

---

### Prompt 2: Understand - The super() Function

**Copy this prompt to your AI:**

```
Why do we use super().__init__(self, ...) instead of calling Parent.__init__(self, ...)?
Show me a concrete example where super() is critical (hint: multiple inheritance).
Explain what problem it solves.
```

**Expected Outcome**: Clear explanation of why super() handles MRO correctly while direct class calling bypasses it, including a multiple inheritance example.

---

### Prompt 3: Apply - Diamond Inheritance

**Copy this prompt to your AI:**

```
Create classes: Device (with power_on() method),
Phone (Device, adds call()),
Computer (Device, adds compute()),
Smartphone (Phone, Computer, inherits from both).
Override power_on() in Phone and Computer with different messages.
Call phone_obj.power_on() and show the MRO with .mro() method.
Explain which power_on() was called and why.
```

**Expected Outcome**: Working diamond inheritance with proper MRO output showing Device appears once and left parent (Phone) is searched before right parent (Computer).

---

### Prompt 4: Analyze - When Inheritance is Wrong

**Copy this prompt to your AI:**

```
I want to design a software with Shape, Circle, Rectangle, and Color.
Should Circle inherit from Shape?
Should Circle inherit from Color?
For each decision, explain if it's right or wrong and why.
What is the "is-a" vs "has-a" principle?
```

**Expected Outcome**: Analysis showing Circle **is-a** Shape (correct inheritance) but Circle **has-a** Color (should be composition), with explanation of design principles.

---

**Safety Note**: When AI generates inheritance hierarchies, always verify the MRO with `.mro()` method. Unexpected inheritance orders signal design problems. Test your inheritance code before using it in larger systems.

