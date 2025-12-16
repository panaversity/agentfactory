### Core Concept
CPython is Python's reference implementation—written in C, it compiles your code to bytecode, then interprets it through a virtual machine. Every Python installation from python.org is CPython. Understanding this matters because alternative implementations (PyPy, Jython) exist, each optimized for different use cases.

### Key Mental Models
**Python Execution Pipeline**: Source code → Bytecode (recipe card) → Interpreter (follows recipe). Bytecode is platform-independent intermediate code that the interpreter reads sequentially. **Reference Counting**: Every object tracks "how many parts of your code reference me?" When count drops to zero, memory is freed immediately (deterministic, no pausing). **Memory Management Hybrid**: Reference counting handles simple cases; garbage collection cleans up circular references (objects that point to each other).

### Critical Patterns
Detect your Python implementation with `platform.python_implementation()` and `sys.executable`. Different implementations have different GIL behavior and performance characteristics. PyPy uses JIT compilation (slower startup, faster computation); MicroPython targets embedded systems. Production systems often detect implementation at runtime to enable implementation-specific optimizations.

### AI Collaboration Keys
Ask AI to compare implementations without referencing official docs. Have AI explain WHY CPython is written in C instead of Python (bootstrapping problem, performance, OS/hardware integration). Request concrete code examples showing bytecode compilation and reference counting in action using `dis.dis()` and `sys.getrefcount()`.

### Common Mistakes
Confusing "Python the language" with "CPython the implementation"—they're separate. Assuming all Python code runs identically on all implementations (95% compatibility, not 100%). Thinking bytecode is machine code (it's not—it's intermediate, interpreted). Treating reference counting and garbage collection as competing approaches when CPython uses both strategically.

### Connections
Foundational for understanding the GIL (Lesson 3): CPython's reference counting design necessitates thread safety mechanisms. Sets up performance evolution (Lesson 2): interpreter optimizations compound across billions of bytecode instructions. Enables implementation choice framework (Lesson 5): understanding tradeoffs between CPython, PyPy, and alternatives informs concurrency strategy decisions.
