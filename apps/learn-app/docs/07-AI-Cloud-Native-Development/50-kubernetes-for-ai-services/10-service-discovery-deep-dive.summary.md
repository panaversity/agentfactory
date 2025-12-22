### Core Concept
Service discovery is how Kubernetes translates service names (api.services.svc.cluster.local) into IP addresses. CoreDNS, the cluster's internal DNS server, watches for Service changes and automatically updates DNS records. Understanding DNS architecture is critical for debugging "host not found" errors that kill microservices architectures.

### Key Mental Models
- **CoreDNS is the DNS server**—Runs as Pods in kube-system namespace. Every Pod's /etc/resolv.conf points to kube-dns service (10.96.0.10). CoreDNS watches Kubernetes API and maintains DNS records for every Service
- **FQDN = service.namespace.svc.cluster.local**—Fully qualified name includes service name, namespace, cluster domain suffix. Within same namespace, short name works (DNS search path adds namespace automatically)
- **Headless Services for direct Pod discovery**—Normal Service returns virtual IP. Headless Service (clusterIP:None) returns all Pod IPs. Used for direct Pod-to-Pod communication (databases, peer discovery)
- **Endpoints bridge Service selector to Pod IPs**—Service selector says "find Pods with app=api". Endpoints controller continuously updates an Endpoints object with matching Pod IPs. Service routes to these endpoints

### Critical Patterns
- **DNS debugging with nslookup/dig**—`nslookup service.namespace.svc.cluster.local` reveals if DNS resolves. `dig` shows A records (IPs) and SRV records (Pod-specific DNS names with ports)
- **Endpoint troubleshooting flow**—Service exists but no endpoints? Check Service selector (kubectl describe svc). Check Pod labels (kubectl get pods --show-labels). Does selector match labels? If not, fix Deployment labels
- **Cross-namespace DNS**—From agents namespace calling services namespace: use FQDN `api.services.svc.cluster.local`. Short name `api` searches in agents namespace first, fails
- **DNS caching behavior**—Pod resolvers cache results. Rapid DNS changes (Pod churn) might see stale IPs briefly. Not usually a problem

### Common Mistakes
- Using short names across namespaces (fails silently; looks like "host not found" in the cross-namespace Pod)
- Assuming endpoint mismatch is a DNS problem (it's not—DNS resolves fine, but Service has no endpoints so traffic fails at routing layer)
- Forgetting that labels define what Services manage (service selector must exactly match Pod labels)
- Not understanding SRV records (for stateful apps, SRV records enable direct Pod connection instead of service load balancing)

### Connections
- **Builds on**: Services (Lesson 5) which DNS serves, Namespaces (Lesson 8) for cross-namespace discovery
- **Leads to**: Troubleshooting multi-service architectures, headless Services for stateful workloads
