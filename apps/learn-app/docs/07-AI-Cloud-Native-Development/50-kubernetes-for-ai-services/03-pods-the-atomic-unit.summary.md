### Core Concept
A Pod is the smallest deployable unit in Kubernetes—one or more containers that share network and storage. Despite looking simple, Pods are fundamentally different from Docker containers: they're ephemeral (temporary), they share localhost networking, and they're designed to be replaced constantly. Understanding Pod ephemerality is critical for mastering Kubernetes.

### Key Mental Models
- **Pod as apartment, containers as roommates**—Containers in a Pod share everything (network interface, loopback, volumes). They're co-located for tight coupling. Don't force unrelated services into one Pod.
- **Ephemeral design**—Pods are NOT pets that you nurture. They're cattle that you replace. When a Pod terminates, its IP is gone, its data (in emptyDir) is lost, and it doesn't come back unless managed by a controller.
- **Shared network namespace**—All containers in a Pod see localhost. Container A on port 8080 and Container B both exist at 127.0.0.1 in the same Pod—no bridge needed.
- **Request vs Limit semantics**—Requests are scheduler hints (place Pod on nodes with available resources). Limits are hard ceilings (exceed them and Kubernetes kills the container). Both matter for production stability.

### Critical Patterns
- **Single-container Pods are the norm**—Most Pods contain exactly one container. Multi-container means sidecar pattern (logging, monitoring, security) or init container pattern (setup before app starts)
- **Volume mounting for persistence**—Data in containers is ephemeral. Use persistentVolumeClaims for data that must survive Pod termination
- **Label everything**—Pods use labels extensively for selection. Services find Pods by labels. Deployments find Pods by labels. Without labels, nothing works

### Common Mistakes
- Storing important data in Pod containers (LOST when Pod dies—use persistent volumes)
- Hard-coding Pod IPs in configuration (IPs are ephemeral—use Services for stable addressing)
- Creating multi-container Pods for unrelated services (only use for tightly coupled concerns like logging sidecars)
- Ignoring resource limits (runaway Pods can starve the entire cluster)

### Connections
- **Builds on**: Docker container knowledge from Chapter 49 (image concepts, ports, volumes)
- **Leads to**: Deployments (Lesson 4) which manage Pod replicas, Services (Lesson 5) which provide stable access to Pods, and sidecar patterns (Lesson 7)
