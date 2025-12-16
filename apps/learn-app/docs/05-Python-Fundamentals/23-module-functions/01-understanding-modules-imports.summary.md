### Core Concept
A module is a Python file containing organized code (functions, variables, classes). Modules solve the code organization problem: rather than one massive file, developers split code logically and import only what's needed. Python provides three import patterns—each appropriate for different situations.

### Key Mental Models
- **Module as Toolbox**: Like a labeled toolbox containing tools. You open it (import), access tools (functions), then close it when done. Tools are organized by function.
- **Namespace Pattern**: `import math` brings math's namespace into your code. You access items via `math.item` (explicit, clear).
- **Direct Access Pattern**: `from math import sqrt` brings sqrt directly. Shorter code, but less clear where sqrt comes from.
- **Aliasing Pattern**: `from math import sqrt as square_root` renames for clarity or to avoid name collisions.

### Critical Patterns
- **Import Pattern 1** (`import module_name`): Use most often. Clarity: you know where functions come from. Best for multi-function imports.
- **Import Pattern 2** (`from module import specific`): Use when importing 1-2 functions repeatedly. Risk: readers don't know where functions come from.
- **Import Pattern 3** (`from module import X as alias`): Use when avoiding naming conflicts or clarifying long names (e.g., `import numpy as np`).
- **Documentation Discovery**: Professionals don't memorize modules. Instead, ask AI or check Python docs to explore available functions.

### AI Collaboration Keys
AI knows all standard Python modules and their functions. Instead of guessing, ask AI: "Which module should I use for X?" AI validates import choices and explains tradeoffs between patterns.

### Common Mistakes
- Importing entire modules when you only need one function (unnecessary namespace overhead)
- Using `from X import *` (unclear what gets imported, dangerous name collisions)
- Assuming import order doesn't matter (top-level imports are standard convention)

### Connections
- **Builds on**: Chapter 19 (syntax), Chapter 18 (Python basics)
- **Leads to**: Lesson 2 (writing custom functions), understanding code organization
