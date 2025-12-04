### Core Concept
Python has three numeric types—int (whole numbers), float (decimals), and complex (imaginary numbers)—and choosing the right one depends on whether your data naturally includes decimals and the operations you'll perform.

### Key Mental Models
- **Whole vs. Decimal**: Integers represent counts (things you can't have half of); floats represent measurements (things that can be divided)
- **Precision Loss in Conversion**: Converting float to int truncates (loses) the decimal part rather than rounding
- **Type Casting Principle**: When mixing int and float in arithmetic, Python automatically promotes int to float to prevent data loss

### Critical Patterns
- Use `int` for discrete counts and indices: age, student count, array positions
- Use `float` for continuous measurements: prices, distances, percentages, scientific values
- Use `//` (floor division) for integer division that returns an int; use `/` (regular division) which always returns float
- Integer division surprise: `5 / 2` returns `2.5` (float), but `5 // 2` returns `2` (int)

### AI Collaboration Keys
When deciding between int and float, ask AI: "Should I store this as int or float?" and describe the data and operations. AI can help identify edge cases like currency calculations (where Decimal is safer than float) or precision requirements.

### Common Mistakes
- Using float for financial calculations without understanding precision errors (0.1 + 0.2 ≠ 0.3 exactly due to IEEE 754 representation)
- Choosing int for prices or measurements that naturally have decimals, causing data loss
- Confusing integer division (`//`) with regular division (`/`), leading to unexpected type changes
- Treating type hints as enforced types (Python doesn't enforce them at runtime—they're developer promises)

### Connections
- **Builds on**: Understanding types and the type() function (Lesson 1)
- **Leads to**: Type utilities like isinstance() and explicit casting (Lesson 5)
