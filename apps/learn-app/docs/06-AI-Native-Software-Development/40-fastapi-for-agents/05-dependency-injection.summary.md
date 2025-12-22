### Core Concept
Dependency injection makes dependencies explicit and replaceable—instead of reaching for global state, endpoints declare what they need, and FastAPI provides it. This enables testing (swap real repo for mock), flexibility (swap in-memory for database without changing endpoints), and clarity (dependencies visible in function signature).

### Key Mental Models
- **Depends() as a seam**: The dependency function is where you hook in different behaviors. Returning the same instance enables testing via `app.dependency_overrides`. This indirection is intentional—it's the point of dependency injection.
- **Repository pattern encapsulation**: All data operations (find, create, update, delete) are methods on TaskRepository. Endpoints don't know if it's in-memory or database—they just call methods.
- **Singleton pattern**: One shared instance across all requests. Each worker doesn't need its own copy—they share the same repository state. This is why `global` is problematic (workers can't share), but Depends() works.

### Critical Patterns
- Create a dependency function that returns what you need: `def get_task_repo() -> TaskRepository: return task_repo`
- Inject via parameter: `def endpoint(..., repo: TaskRepository = Depends(get_task_repo))`
- Repository class encapsulates all data operations: `create()`, `get_by_id()`, `update()`, `delete()`
- Validate business logic in endpoints, data operations in repository
- Dependency function is the seam for testing—override it with `app.dependency_overrides[get_task_repo] = mock_repo`

### Common Mistakes
- Calling the dependency function directly `get_task_repo()` instead of using `Depends(get_task_repo)`—bypasses FastAPI's management and testing overrides
- Creating a new instance in the dependency function each request instead of using a singleton—data doesn't persist between requests
- Accessing repository directly (`task_repo.get_by_id()`) instead of through dependency—couples code and prevents testing
- Putting business logic in the repository when it belongs in endpoints (or vice versa)

### Connections
- **Builds on**: Lesson 4's error handling that endpoints now delegate to repository
- **Leads to**: Lesson 6's async operations where dependencies manage resource lifecycle
