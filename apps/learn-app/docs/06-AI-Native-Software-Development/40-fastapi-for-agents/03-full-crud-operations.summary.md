### Core Concept
CRUD operations (Create, Read, Update, Delete) form the data layer for any application—every agent that manages state uses these four patterns. Map HTTP methods semantically: GET retrieves, POST creates, PUT updates, DELETE removes.

### Key Mental Models
- **HTTP method semantics matter**: GET is safe (no side effects, can be cached), POST is not idempotent (each call creates new resource), PUT is idempotent (retry safely), DELETE is idempotent (retry safely). This matters for agents—they retry idempotent operations automatically.
- **Filtering through optional parameters**: `GET /tasks?status=pending` filters server-side. The parameter is optional (default None), and the endpoint returns different data based on it. This is how agents request specific subsets.
- **404 handling as explicit validation**: Raising `HTTPException(404)` when a resource doesn't exist is the standard HTTP contract. Without it, endpoints return None or empty, which confuses clients.

### Critical Patterns
- List operations accept optional query parameters for filtering
- Single resource endpoints use path parameters and return 404 if not found
- PUT updates accept a model with all fields and returns the updated resource (so client sees result)
- DELETE returns confirmation, either 200 with message or 204 No Content
- All CRUD operations validate input before modifying (check task exists before updating)

### Common Mistakes
- Not returning the updated resource from PUT—client has to make follow-up GET
- Using POST for updates instead of PUT, violating HTTP semantics
- Using GET for operations with side effects (`GET /tasks/1/delete`), breaking caching and proxy assumptions
- Not handling not-found cases explicitly, letting endpoints return None which causes confusion

### Connections
- **Builds on**: Lesson 2's Pydantic models and basic POST endpoints
- **Leads to**: Lesson 4's error handling that makes CRUD operations production-ready
