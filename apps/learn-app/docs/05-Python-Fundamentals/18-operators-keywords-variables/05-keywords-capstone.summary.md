### Core Concept
Keywords are Python's 35 reserved words that define language structure. You cannot use keywords as variable names because Python claims them for syntax (control flow, functions, classes, special values). The capstone calculator integrates all four operator types with keyword awareness and type validation.

### Key Mental Models
- **Language Grammar**: Keywords are Python's grammar—they define how the language works (if, for, def, class, import, etc.)
- **Reserved Protection**: Keywords prevent ambiguity: if `if` weren't reserved, Python couldn't parse `if x > 5:` correctly
- **Keyword Categories**: Control flow (if, for, while), function/class definition (def, class), special values (True, False, None), exception handling (try, except), operators (and, or, not)
- **Defensive Programming**: Check `keyword.iskeyword(word)` before using name as variable; defensive habit prevents hard-to-debug SyntaxErrors

### Critical Patterns
- Integration challenge: Calculator demonstrates all four operator types (arithmetic, comparison, logical, assignment) plus type validation in unified project
- Common reserved words to avoid: for, if, while, def, class, import, return, try, except, pass
- Name alternatives: If desired name is reserved, add suffix (_var, _count, _loop) or use synonym (iterations instead of for)

### AI Collaboration Keys
- Ask AI about language design: "Why did Python reserve these specific words?" (need for parsing, syntax unambiguity)
- Collaborate on keyword workarounds: AI suggests alternative variable names when desired name is reserved
- Validate capstone code: have AI review calculator for correct operator usage, type handling, and keyword avoidance

### Common Mistakes
- Trying to use `for = 5` or `if = True` (SyntaxError: for and if are reserved)
- Forgetting to check keyword status before naming variables; hard to debug later
- Assuming less common words aren't reserved (e.g., `lambda`, `yield`, `async` are all reserved)

### Connections
- **Builds on**: All four operator types (Lessons 1-4) are integrated in capstone project
- **Leads to**: Chapter 21 (Strings) shows operators work with text too; Chapter 22 (Control Flow) uses comparisons and logical operators in if statements and loops
