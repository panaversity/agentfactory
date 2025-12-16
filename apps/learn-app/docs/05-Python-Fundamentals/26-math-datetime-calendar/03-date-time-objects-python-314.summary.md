### Core Concept

The `datetime` module provides objects for working with dates and times as structured data rather than numbers or strings. Python 3.14 introduces **new parsing methods** (`date.strptime()`, `time.strptime()`) that directly convert user-provided strings into objects. The key insight: **separate concerns**—parse from strings (user input), construct programmatically (known dates), and understand naive vs timezone-aware for global applications.

### Key Mental Models

**Objects vs Primitives**: A `datetime` object stores year, month, day, hour, minute, second as separate attributes, enabling direct access and type safety compared to storing a formatted string.

**Parsing vs Construction**: Construction (`date(2025, 11, 9)`) is for code you control. Parsing (`date.strptime("2025-11-09", "%Y-%m-%d")`) is for user input—the format string tells the parser how to interpret the string's structure.

**Naive vs Timezone-Aware**: A naive datetime doesn't specify timezone (ambiguous across regions). An aware datetime includes timezone info (UTC offset). For global applications, always use aware datetimes to prevent bugs.

### Critical Patterns

**Python 3.14 New Methods**: `date.strptime()` and `time.strptime()` allow direct parsing into date and time objects respectively, then combine with `datetime.combine()`. This is more direct than pre-3.14 approaches.

**Validation Through Parsing**: If `strptime()` succeeds, the input was valid. If it raises `ValueError`, the input doesn't match the expected format. This replaces manual validation logic.

**UTC Storage Pattern**: For timezone-aware datetimes, always store in UTC internally (`datetime.now(timezone.utc)`), then convert to local time only for display.

### AI Collaboration Keys

Format code syntax (`%Y`, `%m`, `%d`, `%H`, `%M`, `%S`) is memorization-free in AI-native development—you understand the pattern and ask AI when you need specific codes. Timezone concepts are complex (DST, half-hour offsets), and AI excels at explaining edge cases you'd struggle to discover alone.

### Common Mistakes

Confusing `datetime.now()` (local timezone-naive) with `datetime.now(timezone.utc)` (timezone-aware UTC). Students also mix naive and aware datetimes in the same calculation, causing `TypeError` that's cryptic to debug. Another error: wrong format codes in `strptime()` causing silent parsing failures.

### Connections

This lesson **abstracts away epoch thinking** from Lesson 2 through high-level objects. It enables Lesson 4's formatting and timezone conversion by providing structured data. It's the prerequisite for Lesson 6's capstone (parsing user timezone input). Understanding when to parse (user input) vs construct (known dates) is foundational for all subsequent datetime work.
