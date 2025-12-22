### Core Concept
Kubernetes is a distributed operating system for containers. The fundamental insight is that you declare *what* you want (desired state) and Kubernetes continuously works to make it happen through reconciliation loops—not *how* to make it happen. Docker handles single machines; Kubernetes handles clusters.

### Key Mental Models
- **Declarative vs Imperative**: You describe desired state (3 replicas running), not step-by-step instructions (start container 1, start container 2...). Controllers continuously reconcile reality with your declarations.
- **Desired State vs Observed State**: Kubernetes constantly compares what should exist with what actually exists. When they don't match, controllers take action to close the gap.
- **Reconciliation Loop**: Observe → Compare → Act → Wait → Loop. This is Kubernetes' heartbeat. When a container crashes, the mismatch triggers automatic recovery.

### Critical Patterns
- **The Control Plane decides WHAT**—API server, scheduler, controller manager, etcd make decisions about placement and scaling
- **Worker Nodes do HOW**—kubelet, kube-proxy, container runtime execute the decisions
- **Self-healing emerges automatically**—Pod crashes trigger replication controllers to replace it without human intervention
- **No manual restarts needed**—Unlike Docker where you manually restart failed containers, Kubernetes controllers detect and fix failures in seconds

### Common Mistakes
- Thinking Kubernetes is "just Docker but distributed" (it's fundamentally different: declarative vs imperative)
- Expecting instant failure detection (controllers check every few seconds, not instantly)
- Deploying Pods directly instead of using Deployments for production (ephemeral Pods won't auto-restart)

### Connections
- **Builds on**: Docker containerization knowledge from Chapter 49 (images, registries, container concepts)
- **Leads to**: Deployments (Lesson 4) which manage Pods, Services (Lesson 5) for stable networking, and the entire orchestration stack
