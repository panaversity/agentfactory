### Core Concept
Role-Based Access Control (RBAC) restricts what Pods can do by connecting ServiceAccounts (identity) to Roles (permissions) via RoleBindings. Principle of least privilege: each workload gets only the permissions it needs, preventing compromised containers from pivoting through the cluster.

### Key Mental Models
- **Five-Component Stack**: ServiceAccount (identity) → RoleBinding (connection) → Role (permissions) → applied to resources (ConfigMaps, Secrets, Pods)
- **Scope Hierarchy**: Role/RoleBinding are namespace-scoped; ClusterRole/ClusterRoleBinding are cluster-wide
- **Verb Patterns**: get (retrieve specific resource), list (enumerate all), create/delete/patch/watch for modifications

### Critical Patterns
- Create ServiceAccount explicitly in manifests; don't rely on default ServiceAccount (has no permissions)
- Define Roles with minimal required verbs (read-only = get/list, no create/delete/patch)
- Use resourceNames in Role rules to restrict access by specific resource name (e.g., only get secrets named "api-credentials")
- Audit permissions with kubectl auth can-i to verify—test both positive (can do X) and negative (cannot do Y) cases

### Common Mistakes
- Creating ClusterRoleBinding to cluster-admin for a namespaced workload (massive over-privilege)
- Specifying only resources without apiGroups (breaks for non-core API groups like apps/v1)
- Assuming default ServiceAccount has permissions—it's restricted by default (correct design, but requires explicit assignment)
- Not testing RBAC until deployment—permission errors appear only when Pods actually try to access APIs

### Connections
- **Builds on**: Pod execution (Lesson 4), Kubernetes API concepts
- **Leads to**: Production security hardening, multi-tenant cluster isolation
