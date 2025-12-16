### Core Concept
Variables have scope—regions of code where they exist and are accessible. Python searches for variables using LEGB rule: Local, Enclosing, Global, Built-in. Understanding scope prevents bugs, clarifies intent, and enables advanced patterns like closures. Nested functions accessing outer function variables create closures—functions that "remember" enclosing scope data.

### Key Mental Models
- **LEGB Scope Resolution**: Python searches Local → Enclosing → Global → Built-in, using the first match. Each scope level shadows outer levels.
- **Local Isolation**: Variables inside functions are local—invisible outside. Each function has its own local scope.
- **Shadowing**: Assigning to a variable inside a function creates LOCAL variable, shadowing (hiding) any outer-scope variable with same name.
- **Closure**: Inner function accessing outer function variables "closes over" those variables. Even after outer function returns, inner function remembers them.
- **Global is Rarely Needed**: Good design uses parameters and return values instead of global state.

### Critical Patterns
- **Reading vs. Modifying Globals**: Read without `global` keyword. Modifying requires `global variable_name` first.
- **Nested Functions (Closures)**: `def outer(x): def inner(): return x * 2; return inner` creates inner function that remembers x.
- **Closure Factory**: Outer function with parameter creates specialized inner function: `double = make_multiplier(2)`.
- **LEGB in Action**: When Python sees variable, it checks Local → Enclosing (for nested functions) → Global → Built-in.
- **nonlocal for Modifying Enclosing**: Like `global` but for enclosing scope. Use sparingly.

### AI Collaboration Keys
AI understands scope rules and can debug shadowing, closure, and scope-related bugs. Ask AI to explain LEGB lookup for ambiguous cases, or redesign code to avoid global state.

### Common Mistakes
- Expecting variables defined in function to exist outside (scope isolation)
- Assigning to variable in function thinking it modifies global (actually creates local shadowing)
- Overusing `global` keyword (indicates poor design)
- Lambda closures capturing variables incorrectly in loops

### Connections
- **Builds on**: Lessons 1-3 (functions, parameters), Chapter 22 (control flow)
- **Leads to**: Lesson 5 (capstone), Chapter 29+ (advanced patterns, decorators)
