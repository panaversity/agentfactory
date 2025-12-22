### Core Concept
ConfigMaps and Secrets decouple configuration from container images. ConfigMaps store non-sensitive data (URLs, feature flags, timeouts). Secrets store sensitive data (API keys, passwords, tokens). Both use the same injection patterns (environment variables or volume mounts). The key insight: your image contains no configuration. Kubernetes injects configuration at runtime, enabling the same image across dev/staging/prod with different configurations.

### Key Mental Models
- **Configuration externalization**—Image has zero configuration. All config comes from ConfigMaps/Secrets injected at runtime. Update API key? No image rebuild needed. Change database URL? No rebuild. Just update the ConfigMap/Secret and restart Pods
- **Base64 is encoding, NOT encryption**—Kubernetes base64-encodes Secrets for easy text handling, but base64 is trivially reversible. A person with cluster access can immediately decode it. Secrets prevent *accidental* exposure (in logs, error messages), not *intentional* exposure. For cryptographic security, encrypt at rest in etcd or use external secret managers
- **Volume mounts vs environment variables**—Use environment variables for simple key-value config. Use volume mounts for file-based config (JSON files, config.yaml) or when config needs to be updated without restarting the Pod
- **ConfigMap is unencrypted, Secret is "special"**—Both are stored in etcd. Both can be encrypted at rest (cloud providers support this). ConfigMap is obviously unencrypted. Secret is also unencrypted by default but conveys intent (this data is sensitive)

### Critical Patterns
- **Two-tier config injection**—ConfigMaps via `envFrom` (all keys at once), Secrets via `env[].valueFrom.secretKeyRef` (cherry-pick sensitive keys). Prevents accidental Secret leaks when using envFrom
- **File mounting for complex config**—Applications expecting /etc/config/app.yaml get it from ConfigMap mounted as volume. Single source of truth in cluster, not baked into image
- **ConfigMap changes require Pod restart**—ConfigMap updates don't auto-propagate to running Pods. Change a ConfigMap, restart Deployment with `kubectl rollout restart` to pick up new values
- **Separate ConfigMaps per environment**—dev-config, staging-config, prod-config ConfigMaps. Same Deployment manifest references the right ConfigMap per environment

### Common Mistakes
- Treating base64-encoded Secrets as encrypted (they're not; anyone with kubectl access reads them)
- Storing credentials in ConfigMaps (ConfigMaps are visible in plain text; use Secrets)
- Expecting ConfigMap changes to auto-apply to running Pods (they don't; Pods read config at startup)
- Using SecretKeyRef for all config (some data is legitimately non-sensitive; use ConfigMap, save Secrets for actual secrets)
- Forgetting `readOnly: true` when mounting Secrets as volumes (prevents accidental modification by container)

### Connections
- **Builds on**: Pods (Lesson 3) and Deployments (Lesson 4)—configuration is injected into Pod specs
- **Leads to**: CI/CD workflows (different environments use different ConfigMaps), security patterns (RBAC limiting Secret access), external secret systems for production-grade security
