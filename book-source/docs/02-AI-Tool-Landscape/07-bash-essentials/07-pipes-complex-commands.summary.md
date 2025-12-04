### Core Concept
Pipes (`|`) connect commands like an assembly line—the output of one command becomes the input of the next—allowing you to build complex operations from simple building blocks.

### Key Mental Models
- **Assembly Line**: Data flows through stations; each command transforms it, passes it on
- **One Command, One Job**: Each command does one thing well (list, filter, count); pipes chain them together
- **Trace the Flow**: Understand any pipeline by imagining data moving step-by-step through each command
- **AI Builds, You Supervise**: You describe WHAT you want; AI builds the HOW; you verify by tracing the flow

### Critical Patterns
- `command1 | command2` — pipe sends command1's output to command2's input
- `ls -la | grep ".py"` — list all files, filter to Python files
- `grep "ERROR" | wc -l` — find error lines, count them
- `cat file | grep "pattern" | wc -l` — read file → filter lines → count matches
- Remove the last pipe step to see intermediate output (`cat file | grep "ERROR"` shows actual errors)

### Common Mistakes
- Memorizing complex pipe syntax instead of understanding the data flow concept
- Not verifying intermediate outputs when debugging long pipelines
- Trying to write complex pipelines from scratch instead of asking AI to build and explain them
- Forgetting that ENTIRE output flows through the pipe, not just selected parts

### Connections
- **Builds on**: File listing (`ls`), file reading (`cat`), directory concepts
- **Leads to**: Redirect operators (`>`, `>>`), complex log analysis, automated scripts
