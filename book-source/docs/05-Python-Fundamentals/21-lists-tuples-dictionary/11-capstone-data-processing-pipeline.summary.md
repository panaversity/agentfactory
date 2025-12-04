### Core Concept
A complete data processing pipeline demonstrates all three structures: parsing raw data into list of dicts, filtering with comprehensions, aggregating statistics into nested dicts, and formatting results. This real-world pattern (ingest→parse→filter→aggregate→output) is foundational for data engineering and analytics.

### Key Mental Models
- **Pipeline Stages**: Raw string → structured list → filtered subset → aggregated statistics → formatted report
- **Structure Integration**: Lists hold ordered records, dicts map fields within records and aggregate groups, comprehensions transform efficiently
- **Accumulator Pattern**: Initialize empty dict, check key existence, accumulate counts/sums, calculate final values
- **Nested Data Design**: `list[dict[str, str|float]]` for records; `dict[str, dict[str, float|int]]` for aggregated statistics

### Critical Patterns
- **Parsing**: Split raw string into lines, extract headers, create dicts per record with type conversions
- **Filtering**: List comprehension with multiple conditions: `[item for item in source if cond1 and cond2]`
- **Aggregation**: For-loop with dict accumulation: check if key exists, initialize if needed, accumulate values, calculate averages
- **Output**: Format dicts into readable report with spacing and alignment

### AI Collaboration Keys
- AI helps design data structures: "Should each student be tuple or dict?"
- AI debugs parsing errors: "Why is my list empty after parsing?"
- AI optimizes aggregation: "How do I track multiple stats per group?"
- AI extends pipeline: "How would I find top performer in each major?"

### Common Mistakes
- Wrong data structure choice early on (ripples through entire pipeline)
- Off-by-one errors in parsing: forgetting `lines[1:]` to skip headers
- Not checking key existence before accessing in aggregation

### Connections
- **Builds on**: All 10 prior lessons (pure integration/synthesis, 0 new concepts)
- **Prepares for**: Functions (Chapter 23), file I/O (Chapter 27), real data processing projects
