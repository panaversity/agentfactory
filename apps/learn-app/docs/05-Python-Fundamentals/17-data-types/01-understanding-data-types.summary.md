### Core Concept
Data types are Python's classification system that tells Python "what kind of data is this?" and "what operations are valid?" Understanding types prevents errors and enables correct operations on data.

### Key Mental Models
- **Types → Operations**: Different types support different operations (numbers support math, text supports joining, booleans support decisions)
- **Type Mismatch → Error**: `5 + "hello"` fails because Python can't add numbers to text
- **Type Decision Framework**: Ask "What kind of data?" to determine the right type (whole number? decimal? text? yes/no? multiple items? nothing?)

### Critical Patterns
- Use `type()` to verify what type Python assigned to a value: `type(42)` returns `<class 'int'>`
- Type hints express intent and communicate to both AI partners and future readers: `age: int = 25` clearly states expectations
- Seven main categories organize all data: Numeric (int, float, complex), Text (str), Boolean (bool), Collections (list, tuple, dict, set, range), Binary (bytes, bytearray, memoryview), Special (None)

### AI Collaboration Keys
When uncertain about which type to use, ask AI: "I need to store [description of data]. Which type should I use?" AI can help reason through decision factors like mutability, ordering, and lookup requirements.

### Common Mistakes
- Treating `0`, `""`, and `None` as equivalent (they're different: zero is a value, empty string has structure but no content, None means no value exists)
- Forgetting that string numbers like `"25"` can't be used in math operations without explicit conversion to int
- Assuming operations work across types (you can't add 5 + "hello" without first converting one to match the other's type)

### Connections
- **Builds on**: Python basics and print() function (Chapter 17)
- **Leads to**: Deep dives into specific types (numeric types in Lesson 2, strings/booleans in Lesson 3, collections in Lesson 4)
