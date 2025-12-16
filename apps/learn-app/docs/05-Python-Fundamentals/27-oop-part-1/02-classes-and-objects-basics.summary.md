### Core Concept

Classes are blueprints; instantiation creates independent objects. `self` means "the specific object this code runs on." Understanding this bridges class definition (template) and instance execution (specific object's data).

### Key Mental Models

**Self as Context**: `self.name = name` means "THIS object's name." `dog1.name` and `dog2.name` don't interfere because each has its own `self` space. **Constructor Bridge**: `__init__` runs when creating objects, automatically initializing attributes instead of manual assignment.

### Critical Patterns

Class syntax, constructors (`__init__`), instance methods (all have `self` first), attribute access through `self.attribute`. Type hints clarify intent. Multiple objects remain independent—core proof that encapsulation works.

### AI Collaboration Keys

Ask AI why `self` exists and why Python requires it explicitly. Challenge it to explain memory layout: where does each object's data live? Test its understanding by asking if modifying one object affects others.

### Common Mistakes

Forgetting `self` in methods. Calling `Dog` instead of `Dog()`. Using `name` instead of `self.name` inside methods. Expecting objects to share attributes when they shouldn't.

### Connections

Builds on Lesson 1 (why OOP exists). Enables Lesson 3 (constructors with defaults). Foundation for Lesson 4 (methods and encapsulation). Prerequisite for Capstone (multi-class systems).
