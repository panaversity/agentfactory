### Core Concept
Type casting transforms data between Python's core types (int, float, str, bool) using explicit conversion functions. User input arrives as strings but calculations require numbers; conversions bridge this gap. Validation-first thinking—checking that conversions will succeed before attempting them—prevents errors and enables graceful error handling.

### Key Mental Models
- **Validation-First Pattern**: Check input validity (`isdigit()`, `strip()`) before converting rather than handling errors after failures—prevents crashes and provides specific feedback on why conversion failed.
- **Conversion Asymmetry**: Some conversions always succeed (`str()` works on any type), while others fail conditionally (`int("abc")` raises ValueError)—understanding which conversions are risky guides defensive programming.
- **Boolean as Universal Type**: Any Python value converts to boolean following consistent rules (0/empty → False, non-zero/non-empty → True)—enables decision-making on any data type.

### Critical Patterns
- **String-to-Number Safety**: Always validate string contains only digits (`isdigit()`) or is numeric format before using `int()` or `float()`—prevents ValueError crashes.
- **Clean Before Convert**: Chain `strip()` before validation to remove accidental whitespace from user input—accounts for common copy-paste errors.
- **Float Path for Decimals**: Converting decimal strings to int fails directly (`int("3.14")` errors); convert to float first (`int(float("3.14"))`), accepting truncation.

### AI Collaboration Keys
- Ask AI why certain conversions fail while others succeed (boolean conversion never fails; string-to-int is conditional)—understanding design patterns improves error prediction.
- Explore real-world scenarios: payment processing (currency strings), sensor data (numeric strings), database queries (type mismatch problems).

### Common Mistakes
- Attempting conversion without validation, causing crashes on unexpected input—professional code validates before converting.
- Forgetting that `bool("False")` returns True because non-empty string converts to True; use explicit string comparison instead.
- Assuming `int("3.14")` works because float representation looks numeric; Python is strict about type formatting rules.

### Connections
- **Builds on**: String fundamentals (Lesson 1); understanding data types and immutability.
- **Leads to**: Conditional logic (Chapter 22) for validating type conversions; error handling (advanced chapters) for handling conversion failures gracefully.
