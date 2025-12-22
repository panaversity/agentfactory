### Core Concept
Services solve the Pod ephemerality problem. A Service is a stable virtual IP and DNS name that routes traffic to a dynamic set of Pods. When Pods crash and restart with new IPs, the Service IP and DNS name never change. This decouples clients from Pod instability—they connect to service-name, not pod-ip, and Kubernetes handles the routing underneath.

### Key Mental Models
- **Service as load balancer + service discovery**—Service provides a stable IP (or DNS name) and load-balances traffic across matching Pods. Pods change, Service stays constant
- **Label selectors are the connection mechanism**—Service uses selectors to find Pods. "Find all Pods with label app=backend" is how the Service knows which Pods to route to. Selector mismatch = zero endpoints = broken service
- **ClusterIP (internal) vs NodePort (external) vs LoadBalancer (cloud)**—ClusterIP routes within cluster (Pod-to-Pod). NodePort opens a port on each node (development/testing). LoadBalancer provisions a cloud LB (production)
- **DNS resolution is automatic**—Services get DNS names like service-name.namespace.svc.cluster.local. CoreDNS resolves them. Within-namespace, short names work; across namespaces, use FQDN

### Critical Patterns
- **Endpoints tell the truth**—`kubectl get endpoints service-name` shows actual Pod IPs. No endpoints? Selector doesn't match Pods. This is 90% of Service connectivity issues
- **Service discovers Pods continuously**—Service controller runs constantly. Pods added/removed? Service endpoints update immediately. No manual refreshing
- **Port mapping isolates layer 4 from layer 7**—Service port (80) routes to Pod port (8080). Clients hit port 80, Pods listen on 8080. Useful for port standardization

### Common Mistakes
- Hardcoding Pod IPs (LOST when Pod restarts—use Service names always)
- Service selector doesn't match Pod labels (endpoints show `<none>`, traffic fails)
- Confusing ClusterIP (virtual, internal) with Pod IP (real, ephemeral)—Services are virtual abstractions over actual Pod IPs
- Using LoadBalancer for everything (expensive in cloud; Ingress is preferred for multiple services)
- Testing with port-forward instead of Service DNS (won't catch networking issues that occur with real Services)

### Connections
- **Builds on**: Pods (Lesson 3) and Deployments (Lesson 4)—Services route to Pods managed by Deployments
- **Leads to**: Ingress (Lesson 9) for Layer 7 HTTP routing, namespaces (Lesson 8) with cross-namespace discovery, service discovery deep dive (Lesson 10)
