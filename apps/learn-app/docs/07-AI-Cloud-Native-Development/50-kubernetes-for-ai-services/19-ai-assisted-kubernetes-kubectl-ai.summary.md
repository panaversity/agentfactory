### Core Concept
kubectl-ai bridges natural language intent to Kubernetes operations—describe what you want, it generates manifests or commands. Efficiency comes from rapid iteration: AI generates, you evaluate against your L1 foundation, you refine constraints, repeat until production-ready.

### Key Mental Models
- **Three-Role Collaboration**: You (domain knowledge and production context), AI (syntax and scaffold generation), together reaching better configurations than either could alone
- **Validation Against Foundation**: L1 knowledge from earlier lessons becomes filter—you evaluate AI output for resource appropriateness, probe correctness, completeness
- **Iterative Refinement**: Initial output is scaffold, not finished product—add constraints, clarify resource expectations, refine until aligned with production needs

### Critical Patterns
- Provide full context in initial prompt: image, port, environment variables, health check requirements, resource expectations
- Evaluate AI output against your requirements: Does it match your deployment specification? Are health checks appropriate? Resources reasonable?
- Iterate on constraints (not final code): "Increase memory limit to 384Mi" or "Adjust liveness delay to 20 seconds" guides AI to refine manifest
- Use for diagnosis: "Here's my kubectl describe output, why is the Pod failing?" gets faster root-cause analysis than searching docs

### Common Mistakes
- Accepting AI output without validation (might have over-specification, missing probes, wrong resource assumptions)
- Using kubectl-ai for security-sensitive configuration (RBAC, Secrets)—review these line-by-line manually
- Not providing context in prompts (generic "create a deployment" → generic manifest with wrong resource assumptions)
- Treating AI output as final—scaffolding is 70% done, refinement is required for production

### Connections
- **Builds on**: Manual manifest writing (Lessons 1-8), resource and probe configuration knowledge
- **Leads to**: Faster iteration on new deployments, reduced YAML boilerplate overhead
