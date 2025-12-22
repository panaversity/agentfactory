### Core Concept
FastAPI automates HTTP server setup and documentation—you write endpoint functions with decorators, and FastAPI handles routing, validation, and generates interactive documentation automatically. This foundation directly enables exposing AI agents via REST in later lessons.

### Key Mental Models
- **Uvicorn as the bridge**: Uvicorn handles raw networking (listening, connections, HTTP parsing), FastAPI handles application logic (routing, validation, your code). They work together—you never write socket code.
- **Type hints as API contracts**: Type annotations on function parameters don't just document intent—they're instructions to FastAPI for validation. `task_id: int` means FastAPI rejects "abc" automatically before your code runs.
- **Decorators as configuration**: `@app.get("/path")` is declarative—it tells FastAPI the HTTP method, path, and connects it to your function. This separation means you can test functions independent of HTTP.

### Critical Patterns
- Create project with `uv init`, add dependencies with `uv add`, activate virtual environment
- Use `@app.get("/path")` and `@app.get("/path/{param}")` decorators to define routes
- Return dictionaries from endpoints—FastAPI serializes them to JSON automatically
- Use path parameters `{param}` for resource IDs, query parameters for optional filtering
- Swagger UI at `/docs` provides interactive testing without curl/Postman

### Common Mistakes
- Forgetting to return a value from endpoint (returns None, client sees empty response)
- Trying to manually handle HTTP details instead of letting FastAPI validate types automatically
- Confusing path parameters (identify a specific resource) with query parameters (filter results)

### Connections
- **Builds on**: Chapter 38 HTTP concepts and MCP servers that use HTTP transports
- **Leads to**: Lesson 2's POST endpoints with request bodies, where Pydantic models enforce validation
