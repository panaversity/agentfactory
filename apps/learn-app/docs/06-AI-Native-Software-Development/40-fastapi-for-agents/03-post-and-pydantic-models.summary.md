### Core Concept
Pydantic models define what data your API accepts and returns—they validate JSON structure, type-check fields, and filter response output before it reaches clients. This prevents bad data from reaching expensive LLM operations in agent endpoints.

### Key Mental Models
- **Validation pipeline**: Existence → Type check → Coercion → Pass to function. Each step filters invalid data. Required fields missing = 422. Wrong type = 422. Business rules broken = 400 (your code).
- **Models control API surface**: Separating `TaskCreate` (what clients send) from `TaskResponse` (what you return) ensures clients don't provide `id` and `status` fields—the server controls those.
- **Filtering through response_model**: `response_model=TaskResponse` doesn't just validate—it filters output. If your function returns extra fields, only the model's fields appear in the response. This is a security feature preventing accidental data leaks.

### Critical Patterns
- Define models with `class NameModel(BaseModel)` and type-annotated fields
- Use `str | None = None` for optional fields (both parts required: union type AND default)
- `@app.post("/path", response_model=Model, status_code=201)` handles POST with 201 Created status
- `Field(min_length=3, max_length=100)` adds constraints beyond type checking
- Use `response_model` on every endpoint that returns data to control output shape

### Common Mistakes
- Using one model for everything instead of separate create/response models
- Forgetting `response_model` and accidentally leaking internal fields to clients
- Writing `description: str | None` without `= None`, making the field required despite being nullable
- Not understanding 422 vs 400: 422 = schema violation (Pydantic), 400 = business logic violation (your code)

### Connections
- **Builds on**: Lesson 1's basic endpoints, expanding to POST operations
- **Leads to**: Lesson 3's CRUD operations that combine multiple models in one API
