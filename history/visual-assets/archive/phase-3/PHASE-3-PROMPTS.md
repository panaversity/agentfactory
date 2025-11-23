# Phase 3: Image Generation Prompts (V53-V67)

## Visual 53: File Operations Flow

Create a professional educational flowchart showing Python file operations in sequence:
1. START → open() (blue) → File Handle
2. File Handle → read() OR write() (green/orange split)
3. read()/write() → close() (red)
4. close() → END

Show two paths: Read path (green) and Write path (orange). Include warning: "Always close files!" Typography: Largest=operation names, Medium=arrows/flow, Smallest=notes. White background, sans-serif font.

## Visual 54: Context Manager vs Manual Close

Create a side-by-side comparison:
LEFT (Manual - red warning):
```
file = open("data.txt")
data = file.read()
file.close()  # Must remember!
```
Risk: "File left open if error occurs"

RIGHT (With Statement - green checkmark):
```
with open("data.txt") as file:
    data = file.read()
# Auto-closes!
```
Benefit: "Guaranteed cleanup"

Typography: Code in monospace, headers in bold sans-serif. Show checkmarks vs X marks.

## Visual 55: datetime Components

Create a 2×2 grid showing 4 datetime types:
- date (blue): Just calendar date (2024-01-15)
- time (green): Just clock time (14:30:00)
- datetime (purple): Combined date+time (2024-01-15 14:30:00)
- timedelta (orange): Duration/difference (3 days, 2 hours)

Each cell shows type name, example value, and use case. Clean grid with colored headers.

## Visual 56: Timezone Handling

Horizontal comparison showing:
LEFT (Naive - yellow warning):
- datetime without timezone
- Example: 2024-01-15 14:30
- Problem: "Which timezone?"

RIGHT (Aware - green):
- datetime with timezone
- Example: 2024-01-15 14:30 UTC
- Solution: "Explicit timezone"

Show globe icon with timezone markers. Typography: Clear distinction between naive/aware.

## Visual 57: Class Structure Anatomy

Annotated Python class showing:
```python
class Student:
    school = "ABC"  ← Class attribute

    def __init__(self, name):  ← Constructor
        self.name = name  ← Instance attribute

    def study(self):  ← Instance method
        return f"{self.name} studying"
```

6 callouts pointing to: class keyword, class name, class attribute, __init__, instance attribute, instance method. Color-coded: blue=class-level, green=instance-level.

## Visual 58: Inheritance Diagram

Tree showing inheritance:
```
        Animal (parent)
        /    |    \
     Dog  Cat  Bird (children)
```

Each box shows:
- Class name
- Inherited methods (from parent)
- Own methods (specific to child)

Arrows from children pointing up to parent. Color: Parent in blue, children in green. Show method inheritance with dotted lines.

## Visual 59: Composition vs Inheritance

Two-column comparison:

INHERITANCE (left, blue):
- Car IS-A Vehicle
- Tree diagram showing hierarchy
- Use: "IS-A relationship"

COMPOSITION (right, green):
- Car HAS-A Engine
- Box diagram showing containment
- Use: "HAS-A relationship"

Code examples for each. Typography: Clear distinction between IS-A vs HAS-A.

## Visual 60: Method Types

Three-column comparison table:

INSTANCE METHOD (green):
- Takes self parameter
- Accesses instance data
- Example: `def study(self):`

CLASS METHOD (blue):
- Takes cls parameter
- Decorator: @classmethod
- Example: `@classmethod def from_string(cls):`

STATIC METHOD (orange):
- No self/cls
- Decorator: @staticmethod
- Example: `@staticmethod def is_valid():`

Grid with syntax, use case, example for each.

## Visual 61: dataclass Benefits

Before/After comparison:

BEFORE (left, red = verbose):
```python
class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def __repr__(self):
        return f"Point({self.x}, {self.y})"
    # ... more boilerplate
```
Problem: "45 lines of boilerplate"

AFTER (right, green = concise):
```python
@dataclass
class Point:
    x: int
    y: int
```
Benefit: "Auto-generates methods!"

Show list of auto-generated methods: __init__, __repr__, __eq__, etc.

## Visual 62: Class Creation Flow

Flowchart showing metaclass involvement:
1. Python sees `class` keyword
2. Calls metaclass (usually `type`)
3. Metaclass creates class object
4. Class object ready to instantiate

Show: Source Code → Metaclass (type) → Class Object → Instances
Color: blue for metaclass level, green for class level, purple for instance level.

## Visual 63: Pydantic Validation Flow

Three-stage pipeline:

STAGE 1 (Input - red):
- Raw data (any type)
- Example: {"age": "25"}

STAGE 2 (Validation - yellow):
- Pydantic validates & converts
- Type checking, constraints

STAGE 3 (Output - green):
- Typed, validated object
- Example: User(age=25)

Show arrows between stages. Include validation rules: type conversion, range checks, required fields.

## Visual 64: Sync vs Async Timeline

Two horizontal timelines:

SYNC (top - red/slow):
Task1 ████████ (wait) → Task2 ████████ (wait) → Task3 ████████
Total: 24 seconds

ASYNC (bottom - green/fast):
Task1 ███
Task2    ███
Task3       ███
Total: 8 seconds (overlapping execution)

Show how async allows concurrent execution during wait times. Color-code: waiting=red, executing=green.

## Visual 65: Event Loop Concept

Circular diagram showing event loop:
1. Event Queue (tasks waiting)
2. Event Loop (checks queue)
3. Task Execution
4. Back to queue

Show tasks entering queue, loop processing them, and completing. Include: async/await keywords, coroutines, tasks. Central loop in blue, tasks in green, completed in gray.

## Visual 66: GIL Diagram

Visual showing Global Interpreter Lock:
- Single lock icon at top (red)
- Multiple threads below (trying to access)
- Only ONE thread can execute Python at a time
- Others wait for lock

Show timeline: Thread1 executes → releases GIL → Thread2 executes → releases → Thread3 executes
Problem: "No true parallelism for CPU-bound tasks"
Solution: "Use multiprocessing"

## Visual 67: CPython Pipeline

Five-stage pipeline showing:

1. SOURCE CODE (blue)
   example.py

2. PARSER (green)
   → AST (Abstract Syntax Tree)

3. COMPILER (yellow)
   → Bytecode (.pyc)

4. INTERPRETER (orange)
   → PVM execution

5. OUTPUT (purple)
   → Results

Show arrows between stages. Include file extensions: .py → AST → .pyc → execution → output
