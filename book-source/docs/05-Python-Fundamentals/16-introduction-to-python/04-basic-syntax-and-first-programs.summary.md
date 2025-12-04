### Core Concept
Python's unique syntax (indentation over braces) structures code visually; comments explain intent; print() displays results; f-strings format output cleanly. Together, these enable you to write complete, working programs—the first integration of variables, types, and functionality.

### Key Mental Models
- **Indentation as Structure**: Python's whitespace-based syntax means code appearance directly reflects logical structure; future control flow (if/loops) will be visually obvious, not hidden in braces
- **Comments as Future Context**: Comments explain WHY code does something, not WHAT (code shows what); good comments are written for your six-months-later self who forgot context
- **F-Strings as Modern Standard**: F-strings replace older concatenation and .format() methods; learning current patterns, not historical ones, respects pace of language evolution
- **Terminal Execution as Proof**: Running .py files from terminal with `python filename.py` proves code actually works; REPL exploration is practice; file execution is reality

### Critical Patterns
- **Indentation Consistency Crisis**: Mixing tabs and spaces causes IndentationError; configure editor to convert tabs to spaces; this single mistake blocks beginners frequently
- **Print as Validation Tool**: `print()` is how you see program internals; throughout Part 4, printing intermediate values reveals whether code does what you expect
- **F-String Over Concatenation**: Old way (`"Text: " + variable`) requires explicit type conversion; modern way (`f"Text: {variable}"`) handles types automatically and reads clearer
- **Comments Guide Future Readers**: Bad comment repeats code ("subtract 1 from position"); good comment explains reasoning ("Python counts from 0, not 1, so subtract 1")

### AI Collaboration Keys
- Syntax knowledge matters less than understanding patterns; when you forget f-string syntax, asking AI for "modern Python string formatting" teaches you to seek current practices
- Comments describing intent (not just code mechanics) help AI understand design goals when you ask for program improvements or extensions
- Indentation errors reveal deep misunderstanding of Python structure; when you encounter IndentationError, AI can diagnose that tabs/spaces mixed rather than just saying "fix indentation"

### Common Mistakes
- Mixing tabs and spaces (results in cryptic IndentationError; solution: configure editor for consistent spaces)
- Forgetting quotes around text in print (`print(Hello)` looks for variable; `print("Hello")` prints text)
- Confusing print (display) with assignment (storage); `age = 25` stores value; `print(age)` displays it; `print(age) = 25` is meaningless
- Using outdated string formatting (`"text " + var` or `.format()`) instead of f-strings

### Connections
- **Builds on**: Variables and type hints, terminal execution basics
- **Leads to**: Complete programs with input/output, control flow (if/while/for), debugging through print statements, code review practices
