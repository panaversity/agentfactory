### Core Concept
Production-grade security requires multiple layers: non-root execution (SecurityContext), read-only filesystems (emptyDir for writable paths), network isolation (NetworkPolicies), and image scanning. Together they limit damage if a container is compromised.

### Key Mental Models
- **Defense in Depth**: Each security layer independently reduces attack surface—non-root alone doesn't prevent compromise, but it limits what attackers can do post-compromise
- **Explicit Allow Model**: NetworkPolicies default-deny everything, then explicitly allow specific ingress/egress routes
- **Pod Security Standards**: Three tiers (Privileged, Baseline, Restricted) codify security practices—Restricted is appropriate for production workloads

### Critical Patterns
- Build images with non-root user (useradd in Dockerfile, UID > 1000), then enforce with SecurityContext (runAsNonRoot: true)
- Mount emptyDir volumes for /tmp and /var/log; make root filesystem readOnly—constrains attackers to temporary modifications
- Implement NetworkPolicy deny-all, then allow specific source namespaces/pods for ingress and destination ports for egress
- Scan container images with Trivy or similar before pushing to registry; integrate into CI/CD pipeline

### Common Mistakes
- Running containers as root (default for many base images) without forcing non-root in SecurityContext
- Enabling root filesystem write access when read-only would work (unnecessary attack surface)
- Creating overly permissive NetworkPolicies (defeating the purpose of isolation)
- Ignoring image vulnerabilities until production incident—scanning should be pre-deployment gate

### Connections
- **Builds on**: Pod execution (Lesson 4), RBAC (Lesson 8), deployment practices (Lessons 1-7)
- **Leads to**: Compliance (PCI-DSS, SOC2 require security contexts), secure multi-tenant clusters
