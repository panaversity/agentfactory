### Core Concept
StatefulSets guarantee stable Pod identity (pod-0, pod-1, pod-2 remain the same across restarts) and ordered lifecycle (startup and shutdown happen sequentially), making them essential for stateful workloads like databases and distributed caches where topology and data locality matter.

### Key Mental Models
- **Stable Identity**: Unlike Deployments creating random Pod names, StatefulSets create ordinal-based names that persist across restarts, enabling predictable discovery
- **Ordered Lifecycle**: Pods start in order (0→1→2) and scale down in reverse (2→1→0), critical for quorum-based systems and cluster initialization
- **Headless Service Model**: DNS resolves directly to individual Pod IPs (pod-0.service.cluster.local) instead of a virtual IP, enabling peer-to-peer communication

### Critical Patterns
- Define StatefulSet with serviceName matching a headless Service (clusterIP: None) to enable stable DNS
- Use volumeClaimTemplates to create one PVC per Pod with stable naming (qdrant-data-qdrant-0, etc.)
- Scale StatefulSets cautiously—scaling up creates new Pods and their storage, scaling down deletes both
- Rolling updates use partition strategy to control which Pods get updated, enabling safe canary deployments

### Common Mistakes
- Forgetting to create the headless Service before StatefulSet—Pods won't have stable DNS
- Scaling StatefulSet expecting it to work like Deployment—order matters and storage is per-Pod
- Not testing Pod deletion and recreation to verify storage reattachment works correctly
- Assuming Pod 0 is always primary—topology depends on your application's quorum rules

### Connections
- **Builds on**: Persistent storage (Lesson 12) and Services (Lesson 5) for discovery
- **Leads to**: Production deployments requiring multi-replica coordination, distributed agent systems
