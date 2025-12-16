### Core Concept

Computers measure time as a single number: **seconds since the Unix epoch** (January 1, 1970 UTC). This universal reference point enables instant comparisons and calculations across global systems. The key insight: timestamps are simple numbers that hide complexity—understand the epoch concept, let tools handle conversions.

### Key Mental Models

**The Epoch as Reference Point**: All timestamps measure distance from one fixed moment (1970-01-01 00:00 UTC). This makes timestamps language-agnostic and universal—any computer on Earth agrees on the number at this instant.

**Timestamp Structure**: The timestamp `1731120456.7382598` has two parts: integer seconds (1731120456) and fractional seconds (.7382598 microseconds). This dual structure supports both coarse (log timestamps) and precise (benchmarking) timing needs.

**Time Tuples as Decomposition**: A time tuple unpacks a timestamp into human-readable fields (year, month, day, hour, etc.), enabling extraction of specific components for calculations or display.

### Critical Patterns

**Getting Current Time**: `time.time()` returns floating-point seconds since epoch—the universal way to capture "right now" for logging, timestamps, and benchmarking.

**Converting to Components**: Use `time.localtime()` to decompose timestamps into time tuple fields (`tm_year`, `tm_mon`, `tm_mday`, `tm_hour`, `tm_min`, `tm_sec`, `tm_wday`).

**Duration Calculation**: Subtract timestamps to find elapsed time—this arithmetic works correctly because timestamps are just numbers. No calendar complexity needed.

### AI Collaboration Keys

The epoch seems abstract initially, but contextualizing it with real dates (your birthday as a timestamp) makes it concrete. AI helps explain the historical reasoning and handles conversion details while you focus on understanding why computers prefer numbers to date strings.

### Common Mistakes

Students confuse timestamps (seconds since 1970) with dates (calendar notation). They also forget that `time.time()` returns a float—the integer part is complete seconds, the decimal part is microseconds. Another error: not handling timezone information—timestamps are always UTC, but converting to local time requires additional work.

### Connections

This lesson provides the **temporal foundation** for Lesson 3 (datetime objects), which wrap timestamps in a more human-friendly interface. Understanding epoch makes datetime's design choices clear. Timestamp operations enable Lesson 4's duration calculations and Lesson 6's capstone project.
