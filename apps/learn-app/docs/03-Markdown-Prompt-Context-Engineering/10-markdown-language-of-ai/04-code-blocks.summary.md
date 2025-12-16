### Core Concept
Code blocks (fenced with triple backticks and language tags) provide **specification by example**—showing exact expected output and code format so AI agents see concrete targets instead of interpretations, reducing ambiguity dramatically.

### Key Mental Models
- **Specification by Example**: Showing expected output in a code block is clearer than describing it in prose ("show task number, description, status" → actual output format)
- **Language Tags as Context**: `python`, `bash`, `text` tags tell AI which language to generate and which parser applies (prevents syntax mismatches)
- **Exact vs Abstract**: Abstract: "show current temperature"; Exact: code block showing "Hello! The time is 14:30:00"; AI implements to match exact output
- **Acceptance Tests Embedded**: Code blocks become the measurable success criteria—"did the generated code produce exactly this output?"

### Critical Patterns
- **Fenced syntax**: Triple backticks ``` at start and end; content between treated literally (no markdown formatting inside)
- **Language tags**: `python`, `bash`, `text`, `json`, `yaml`, etc. go immediately after opening backticks
- **Inline code syntax**: Single backticks for short references in text (`pip install`, `app.py`, `calculate()`)
- **Fenced vs Inline**: Fenced = multiple lines or program output; Inline = commands/variable names within sentences
- **Space requirement**: No space between opening backticks and language tag (`` ```python `` not `` ``` python ``)

### AI Collaboration Keys
- **Output specification**: Code blocks showing expected output give AI an unambiguous target to implement against
- **Language clarity**: Language tags prevent AI from mixing syntaxes (Python code in bash block, etc.)
- **Example code**: Code blocks can show example implementations or patterns the AI should follow
- **Validation target**: Exact output in code blocks becomes the test—AI knows implementation succeeded when output matches exactly

### Common Mistakes
- Forgetting closing triple backticks (leaves code block unclosed, breaks rendering)
- Using inline code for multiple lines (backticks work line-by-line; use fenced blocks for multi-line)
- Missing language tag when it matters (unlabeled code blocks are ambiguous—is this Python or pseudocode?)
- Showing description instead of exact output ("menu appears" vs actual menu text with specific options)
- Spaces in code blocks treated literally (important for exact output matching—"Hello" is different from " Hello")

### Connections
- **Builds on**: Lesson 1-3 (structure, headings, lists context where code blocks appear)
- **Leads to**: Lesson 5 (integrating code blocks into complete specifications with emphasis and links)
- **Professional practice**: "Specification by example" is industry standard—reduces clarification cycles by 60-80% because everyone sees what "correct" looks like
