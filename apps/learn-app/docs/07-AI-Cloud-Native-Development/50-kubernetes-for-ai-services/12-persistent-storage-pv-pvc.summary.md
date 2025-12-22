### Core Concept
PersistentVolumes (PVs) and PersistentVolumeClaims (PVCs) decouple storage from container instances, allowing data to survive Pod restarts. PVs are cluster-level storage infrastructure, PVCs are namespace-level consumer requests that Kubernetes binds together.

### Key Mental Models
- **Storage Abstraction Layer**: Like Node/Pod separation, PV/PVC separates infrastructure (storage) from consumption (requests), enabling true data persistence
- **Dynamic vs Static Provisioning**: Static PVs are manually created; StorageClasses provision PVs dynamically when PVCs request storage
- **Access Mode Hierarchy**: ReadWriteOnce (single Pod), ReadOnlyMany (multiple readers), ReadWriteMany (multiple writers) enable different sharing patterns

### Critical Patterns
- Create PVCs in manifests instead of raw PVs—Kubernetes automatically provisions backing storage via StorageClass
- Mount PVCs into container volumeMounts so applications access persistent data at a filesystem path
- Use reclaim policies (Delete, Retain) to control what happens to storage when PVCs are deleted
- Test storage survival across Pod restarts by deleting a Pod and verifying data persists when it rebuilds

### Common Mistakes
- Assuming all volumes are persistent—emptyDir volumes disappear when Pods terminate
- Setting requests larger than actual usage, wasting cluster storage capacity
- Binding wrong access modes (e.g., using ReadWriteOnce when you need multi-pod access)
- Not testing PVC mounting until production, discovering path issues too late

### Connections
- **Builds on**: Pod volumes (ephemeral storage from Lesson 4), Service discovery enabling multi-pod access patterns
- **Leads to**: StatefulSets (Lesson 13, which use volumeClaimTemplates for per-Pod storage), Helm parameterization of storage claims
