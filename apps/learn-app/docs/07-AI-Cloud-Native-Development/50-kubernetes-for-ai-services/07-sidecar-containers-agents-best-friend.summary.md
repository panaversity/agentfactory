### Core Concept
Sidecars are helper containers that run in the same Pod as your main app, handling operational concerns independently. Your agent focuses on business logic; the logging sidecar ships logs to a central system, the metrics sidecar scrapes metrics, the proxy sidecar handles TLS. Kubernetes 1.28+ native sidecars use `initContainers` with `restartPolicy: Always`—they start before the app but keep running.

### Key Mental Models
- **Separation of concerns**—Application code doesn't know about logging infrastructure. Sidecar watches log files and ships them. Both improve independently. Testing is simpler
- **Same Pod, same network**—Agent listens on localhost:8000, metrics sidecar scrapes from localhost:8000/metrics. No service discovery, no network latency—direct localhost communication
- **Lifecycle coupling**—When Pod terminates, all containers (app + sidecars) terminate together. Sidecars never outlive the app. This is stronger than separate Pods where lifecycles are independent
- **Native sidecars vs old pattern**—Kubernetes 1.28+ allows `initContainers` with `restartPolicy: Always` to act as proper sidecars. Before that, sidecars were regular containers in `containers` list, which caused ambiguity

### Critical Patterns
- **Logging sidecar pattern**—App writes logs to /var/log/, sidecar (fluent-bit, logstash) reads and ships to central system. App doesn't care where logs go
- **Metrics sidecar pattern**—App exposes metrics on localhost:9090, Prometheus sidecar scrapes them every 15 seconds. Metrics stay local to Pod until sidecar forwards them
- **Security sidecar pattern**—Sidecar handles mTLS termination, request routing, rate limiting. App only handles business logic
- **READY count in kubectl get pods**—With sidecars, you see `2/2 READY` (app + sidecar) instead of `1/1`. Both must be healthy

### Common Mistakes
- Confusing sidecar (always-running helper) with init container (one-time setup)—Sidecars use `restartPolicy: Always`
- Putting unrelated services in one Pod (sidecars are for operational helpers; use separate Pods for functional microservices)
- Assuming sidecar failures crash the app (they don't; sidecar restarts independently, app keeps running)
- Resource limits on sidecars (easy to forget; sidecar logging can use significant CPU/memory)

### Connections
- **Builds on**: Pods (Lesson 3), init containers (Lesson 6)—sidecars are often configured as init containers with different restart policy
- **Leads to**: Multi-container Pod architectures, observability patterns for AI agents, security patterns in Lesson 8+ namespaces
