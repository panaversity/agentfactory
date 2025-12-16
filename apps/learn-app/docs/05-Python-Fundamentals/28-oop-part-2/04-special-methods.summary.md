### Core Concept

**Special methods** (dunder methods: `__str__`, `__add__`, `__len__`) define protocols that make custom objects behave like built-in types. A Vector supporting `+` or Playlist supporting `len()` feel natural by following Python protocols. Duck typing: if your object has `__len__()`, Python treats it as having length.

### Key Mental Models

**Protocols as Contracts**: `__str__` gives print() behavior. `__getitem__` gives `[index]` syntax.

**User vs Developer Representation**: `__str__()` for print() and users. `__repr__()` for shell and debugging—ideally valid Python code recreating the object.

**Operator Overloading**: `__add__` defines `+`. Return `NotImplemented` for invalid types to let Python try reverse operator (`__radd__`).

**Container and Iteration**: `__len__`, `__getitem__` behave like list/dict. `__iter__` and `__next__` enable for-loops—separate contracts.

### Critical Patterns

- `def __str__(self)` — User-friendly, called by print()
- `def __repr__(self)` — Developer-friendly, shell display
- `def __add__(self, other)` — Return `NotImplemented` for wrong types
- `def __len__` + `__getitem__` — Support len() and [index]
- `def __iter__` + `def __next__` — Support for loops

### AI Collaboration Keys

Show all operator special methods (__add__, __sub__, __radd__). Create Money class. Why do some operators have reverse versions? When does `__radd__` get called?

### Common Mistakes

- Returning None instead of NotImplemented
- Not implementing both `__str__` and `__repr__`
- Partial container protocol (len but no getitem)
- Forgetting StopIteration in `__next__()`
- Hash inconsistency with `__eq__` changes

### Connections

**Prerequisite**: Lessons 1-3 classes, inheritance, composition. **Enables**: Lesson 5 agents (__str__ logging, __call__ invocation). **Cross-cutting**: All OOP patterns benefit from good special methods.
