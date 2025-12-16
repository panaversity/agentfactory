### Core Concept

The `calendar` module displays calendar grids for scheduling interfaces. Advanced math functions (trigonometry, logarithms, factorial) unlock scientific computing, physics simulations, and probability calculations. The key insight: **recognize when these functions apply** rather than memorizing formulas—domain-specific problems require domain-appropriate mathematics.

### Key Mental Models

**Calendar as Structured Data**: The `calendar` module answers scheduling questions programmatically: "What day does this month start?" (`weekday()`), "How many days in February?" (`monthrange()`), "Is this a leap year?" (`isleap()`).

**Radians as Angle Measurement**: Trigonometry in Python uses radians (2π = full rotation), not degrees. Converting degrees to radians with `math.radians()` is the first step in any trigonometric calculation.

**Logarithms as Compression**: Logarithms compress exponential scales into linear ranges. A sound intensity increasing 1 million times only increases 60 dB—this logarithmic perception matches human senses.

### Critical Patterns

**Calendar Display**: `calendar.month(year, month)` generates ASCII calendars with Python 3.14 highlighting today's date in color. Use for scheduling UI and informational displays.

**Trigonometric Pattern**: Convert degrees to radians → calculate sin/cos/tan → apply to formulas (projectile motion, wave simulation). The conversion is the easy part; recognizing *when* to use trig is the skill.

**Logarithmic Calculations**: Use `math.log()` for continuous growth (exponential), `math.log10()` for scientific scales (decibels, pH). Always validate input > 0 to avoid domain errors.

**Validation with Special Values**: Use `math.inf` as sentinel (initialize min to infinity, actual values replace it) and `math.nan` to mark undefined results (0/0, sqrt(-1) caught in try/except).

### AI Collaboration Keys

Trigonometry and logarithms have intimidating names but straightforward applications. Instead of memorizing formulas, you describe your problem ("calculate projectile range at 30° angle"), AI suggests the formula, and you understand the principle. Special values (`inf`, `nan`) exist for validation logic—AI explains when and why to use them.

### Common Mistakes

Students memorize trigonometric identities instead of recognizing problem patterns that need trig. They also forget `math.radians()` conversion, passing degrees directly to `sin()`/`cos()` and getting wrong answers silently. Another error: writing `math.log()` without validating input > 0, causing domain errors.

### Connections

This lesson **completes the mathematical toolkit** from Lesson 1, adding domain-specific functions for scientific and visualization work. The validation patterns (checking domain constraints) connect to Lesson 1's error handling. It enables advanced applications: Lesson 6 could integrate calendar display (scheduling context), and beyond this chapter, these tools support physics simulations, data analysis, and scientific computing workflows.
