### Core Concept

Datetime objects become useful when formatted for display and manipulated for calculations. `strftime()` converts objects to human-readable strings, `timedelta` represents durations and enables date arithmetic, and timezone conversion ensures accuracy across regions. The key insight: **format for humans, calculate with objects**—never do date math with strings.

### Key Mental Models

**Format Codes as Language**: `%Y-%m-%d` is a language describing date structure. You don't memorize 30+ codes—you understand the pattern (`%` = escape, letter = meaning) and ask AI for unfamiliar codes.

**Timedelta as Duration**: Unlike datetime (a specific moment), timedelta represents "how much time." Adding timedelta to datetime gives a future/past datetime. Subtracting datetimes gives timedelta.

**Timezone Offsets as Simple Math**: UTC is the reference (offset 0). All timezones are +/- hours from UTC. `astimezone()` handles the conversion automatically.

### Critical Patterns

**Format Selection**: ISO 8601 for data/APIs (`%Y-%m-%dT%H:%M:%S`), friendly for UI (`%A, %B %d, %Y`), timestamp for debugging (`%s`). Choose format based on audience.

**Date Arithmetic with Timedelta**: Instead of calculating days manually, use `today + timedelta(days=30)`. Timedelta respects calendar complexity (leap years, month lengths).

**Timezone Conversion Pattern**: Keep system in UTC internally, convert to local only for display. Use `astimezone()` which handles offset adjustments automatically.

### AI Collaboration Keys

Timezone conversion has edge cases (DST transitions, half-hour offsets like India), making it error-prone to code manually. Format code selection requires judgment about audience and context. Both areas benefit from AI guidance—you state what you need ("display for humans," "compare dates across timezones"), AI suggests implementation.

### Common Mistakes

Students hardcode timezone offsets instead of using `timezone()` objects, losing the abstraction. They also try formatting before assigning a timezone to naive datetimes, getting cryptic errors. Another pattern: subtracting datetimes across timezones without ensuring both are in the same timezone reference first.

### Connections

This lesson **operationalizes** Lesson 3's datetime objects, making them useful for real applications. It builds on Lesson 1's validation patterns (handling edge cases) and Lesson 2's timestamp concepts (debugging with timestamps). Together, Lessons 3-4 form the **foundation for Lesson 6's capstone**, which requires all three skills: parsing (Lesson 3), formatting and conversion (Lesson 4), and integration into a user-facing application.
