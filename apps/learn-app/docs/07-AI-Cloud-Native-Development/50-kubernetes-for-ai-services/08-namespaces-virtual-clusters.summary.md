### Core Concept
Namespaces are virtual cluster partitions that enable multi-tenancy on a single cluster. Deploy dev, staging, and production in the same cluster but isolated. ResourceQuotas prevent one environment's workloads from starving others (e.g., a dev experimental agent can't consume all GPU memory needed for staging). LimitRanges set per-container defaults so developers can't accidentally request unreasonable resources.

### Key Mental Models
- **Namespace = logical partition, not physical separation**—Pods in different namespaces can communicate (using FQDN), share the same node, share the same network. Isolation is administrative (quotas, RBAC) not physical
- **ResourceQuota prevents starvation**—Total CPU/memory across all Pods in a namespace is capped. When quota exhausted, new Pods can't be scheduled. Prevents runaway experiments from affecting production
- **LimitRange provides defaults**—Containers without explicit requests/limits get namespace defaults. Prevents both resource waste (underprovisioned) and cluster starvation (overprovisioned)
- **Short vs FQDN DNS names**—Within namespace, `service-name` works. Across namespaces, use `service-name.namespace.svc.cluster.local`. Default namespace search path includes own namespace first

### Critical Patterns
- **Multi-environment strategy**—Create dev, staging, prod namespaces. Deploy same manifests to all three. Different ResourceQuotas/LimitRanges per namespace enforce environment-appropriate constraints
- **Quota enforcement prevents accidents**—A dev team's resource-hungry training job hits the dev namespace quota and waits, instead of crashing production
- **Service accounts per namespace**—RBAC (role-based access control) restricts what Pods can do. Dev namespace Pods can't delete prod resources
- **Monitoring all namespaces**—Central logging/monitoring needs cross-namespace access: use FQDN or tools like Prometheus that scrape all namespaces

### Common Mistakes
- Assuming namespaces are isolated (they're not; use NetworkPolicies for real isolation)
- Setting unrealistic quotas (quota too low prevents legitimate work; too high defeats the purpose)
- Forgetting to set LimitRanges (developers create resource hogs without defaults to constrain them)
- Hardcoding "default" namespace (always use current namespace or explicit FQDN for cross-namespace refs)

### Connections
- **Builds on**: Services (Lesson 5) for DNS discovery, Deployments (Lesson 4) for workload management
- **Leads to**: Security patterns (RBAC per namespace), multi-team deployments, CI/CD workflows that deploy to different namespaces
