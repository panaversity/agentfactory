### Core Concept

Custom validation logic moves beyond type checking to enforce business rules: email format, password strength, cross-field relationships, environment-specific constraints. `@field_validator` decorators, `Field()` constraints, and `@model_validator` for cross-field logic enable declarative specification of what makes data valid in YOUR domain, not just Python's type system.

### Key Mental Models

**Validator Precedence Layers**: Field() constraints run first (structural validation: type, length, pattern), then `@field_validator` (custom logic per field), then `@model_validator` (cross-field relationships). Each layer sees the output of previous layers, building validation progressively.

**Specification as Validation Code**: When you write `Field(min_length=3, max_length=20, pattern=r"^[a-z0-9_]+$")`, you're encoding business rules as executable constraints. The specification (what makes valid data) and implementation (what accepts that data) become one artifact.

**Environment Configuration as First-Class Citizen**: BaseSettings treats environment variables and .env files as primary configuration sources with type conversion and validation. No manual `os.getenv()` fragility—declare fields, let Pydantic handle loading and validation.

### Critical Patterns

**Field() vs @field_validator Decision**: Use Field() for structural constraints (min/max, regex, type coercion). Use @field_validator for complex logic (conditional checks, external validation, custom error messages). Combine both: Field() for simple rules, validators for sophisticated logic.

**Validator Mode Matters**: `mode='after'` (default) runs after type conversion—receives Python objects. `mode='before'` receives raw input before parsing—useful for transformations like normalizing case before validation.

**model_validator for Dependencies**: When one field's validity depends on another (password confirmation, conditional fields), use `@model_validator(mode='after')` to validate the complete model after all fields pass individual validation.

### AI Collaboration Keys

When designing systems that consume AI-generated data, detailed validators serve as specification for the AI: they encode exactly what formats, constraints, and dependencies the data must satisfy. Validation errors become feedback for prompt improvement—use error messages to guide the AI toward correct outputs.

### Common Mistakes

**Overusing @field_validator when Field() suffices** creates unnecessary complexity. Always try Field() first; graduate to validators only when Field() can't express the rule.

**Validating late (not at boundaries)** allows invalid data deep into your system. Validate immediately when data enters—API endpoints, AI output parsing, configuration loading.

**Hardcoding secrets instead of using BaseSettings** creates security vulnerabilities. Always move passwords, API keys, credentials to environment variables with `Field(repr=False)`.

**Mixing validator modes** confuses intent. Pick 'after' (default) for most cases; use 'before' specifically for transformations. Don't mix unless you have a clear reason.

### Connections

Builds on: Lesson 1 (basic Pydantic), type hints, Python decorators

Enables: Lesson 5 (AI output validation), production configuration systems, API frameworks (FastAPI), validation-driven architecture, error recovery loops
