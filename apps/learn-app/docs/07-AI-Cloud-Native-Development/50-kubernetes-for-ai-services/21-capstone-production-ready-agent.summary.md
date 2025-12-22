### Core Concept
Specification-first deployment: write what you're building and why before generating manifests. Specification becomes the contract—manifests implement it, Kubernetes maintains it. Success means specification requirements are satisfied by actual running system.

### Key Mental Models
- **Specification as Contract**: Declares intent, resource needs, configuration requirements, success criteria—prevents implementation drift
- **Generated Implementation**: AI produces manifests from spec, developer validates against foundation knowledge, then deploys and validates
- **Declarative Assurance**: Kubernetes maintains desired state—if Pods fail, new ones start; if configuration changes, rollback restores previous state

### Critical Patterns
- Write specification template answering: What/Why/How many/What config/How validate—forces completeness thinking before coding
- Provide full spec context to AI: image, port, environment variables, resources, probes, scaling expectations
- Validate deployment using checklist: Pods running, service accessible, health checks passing, environment variables injected, Pod recovery works
- Test self-healing by deleting Pods and verifying Kubernetes recreates them automatically

### Common Mistakes
- Skipping specification and jumping directly to manifests (misses requirements, unmaintainable)
- Not testing environment variable injection until deployment (config issues caught too late)
- Assuming health checks work without manual testing—port-forward and curl endpoints before deployment
- Deploying without verifying self-healing—don't discover recovery issues in production

### Connections
- **Builds on**: All prior lessons (1-10), consolidates knowledge into complete deployment workflow
- **Leads to**: Repeatable deployment patterns, skills accumulation (Lesson 11)
