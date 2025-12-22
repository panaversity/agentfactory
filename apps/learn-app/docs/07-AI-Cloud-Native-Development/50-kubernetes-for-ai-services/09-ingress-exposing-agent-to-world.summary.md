### Core Concept
Ingress exposes HTTP and HTTPS routes from outside the cluster to services within it. Unlike LoadBalancer (which creates a load balancer per service and costs $$), Ingress shares one load balancer across many services. You can route based on hostname (agent.example.com vs dashboard.example.com) or URL path (/api/v1 vs /api/v2/beta). Ingress is Layer 7 (application layer), while LoadBalancer is Layer 4 (transport layer).

### Key Mental Models
- **Ingress = smart receptionist**—LoadBalancer = direct tunnel to one service (one phone line per service). Ingress = receptionist who reads the address, looks at the URL path, checks the hostname, and routes you to the right department
- **Resource + Controller = complete solution**—Ingress resource (YAML) specifies rules. Ingress controller (nginx-ingress, AWS ALB) watches Ingress objects and configures real load balancers to implement them
- **Path-based vs host-based routing**—Same hostname, different paths go to different backends (/api/v1 → stable, /api/v2/beta → experimental). Different hostnames go to different backends (agent.example.com → agent-service, dashboard.example.com → dashboard-service)
- **TLS termination at edge**—Ingress handles encryption/decryption. Clients connect over HTTPS, Ingress terminates TLS, and routes plaintext HTTP to backend services. Reduces CPU load on app containers

### Critical Patterns
- **IngressClass selects controller**—`ingressClassName: nginx` tells Kubernetes which controller should implement this Ingress
- **Service selectors in backends**—Ingress routes to Services, not Pods. Services handle the load balancing to Pods
- **Annotations for advanced features**—Rate limiting, CORS headers, request rewriting all via annotations (nginx.ingress.kubernetes.io/limit-rps, etc.)
- **Single Ingress for multiple services**—One host, multiple paths, each path → different service. Consolidates routing logic

### Common Mistakes
- Creating LoadBalancer Services when Ingress would be simpler (LoadBalancer = cost per service, Ingress = cost per gateway)
- Mismatching backend service names (Ingress specifies service name that doesn't exist → 503 Service Unavailable)
- Forgetting that pathType:Prefix means /api/v1 matches /api/v1, /api/v1/, /api/v1/foo (use pathType:Exact for strict matching)
- Not checking Ingress controller is installed (IngressClass shows available controllers; if none, install nginx-ingress first)

### Connections
- **Builds on**: Services (Lesson 5) for backend routing
- **Leads to**: Advanced networking (Lesson 10 service discovery), multi-tenant deployments with per-tenant Ingresses
