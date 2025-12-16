### Core Concept
Nested control structures combine conditionals and loops at multiple levels to solve complex, multi-step problems. They enable decision trees with multiple validation layers, two-dimensional data processing, and selective iteration through collections.

### Key Mental Models
- **Decision Trees**: Each indentation level represents a deeper validation or decision branch; prerequisites at earlier levels gate access to later ones
- **Two-Dimensional Iteration**: Nested loops process grid-like data (rows × columns); inner loop completes all iterations for each outer loop iteration
- **Conditional Filtering**: Place `if` statements inside loops to selectively process items without breaking the loop
- **Conditional Execution**: Place entire loops inside `if` statements to run iteration blocks only when prerequisites are met

### Critical Patterns
- **Multi-Criteria Eligibility**: Nested `if` statements check age range, then subranges, then additional requirements
- **Multiplication Table / Grid**: Outer loop for rows, inner loop for columns; prints one complete row before advancing
- **Sum Only Positives**: Loop through list, conditionally add only numbers matching criteria
- **Validated Processing**: Check user authentication and payment method before looping through cart items

### AI Collaboration Keys
- Ask AI to generate nested structures from problem descriptions; AI handles indentation syntax while you verify logic
- Request AI to trace execution through nested code with specific inputs, showing which branches activate
- When nesting feels too deep (3+ levels), ask AI for flattening strategies using `and`/`or` operators
- Request conversion of deeply nested code to simpler sequential logic when readability suffers

### Common Mistakes
- **Indentation Errors**: Mixing tabs and spaces or incorrect indentation depth changes when code executes
- **Variable Name Collisions**: Reusing loop variable names in nested loops (`for i` inside `for i`) overwrites the outer loop counter
- **Placement Errors**: Code at wrong indentation level runs too many/few times (print inside vs. after inner loop)
- **Deep Nesting Complexity**: 3+ levels of indentation becomes hard to read; flatten with combined conditions when possible

### Connections
- **Builds on**: Lessons 1-4 (all control flow concepts: conditionals, pattern matching, loops, loop control)
- **Leads to**: Chapter 22+ (functions refactor deeply nested code; algorithms build on efficient nesting patterns)
