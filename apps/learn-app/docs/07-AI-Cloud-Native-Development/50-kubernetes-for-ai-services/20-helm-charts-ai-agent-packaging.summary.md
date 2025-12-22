### Core Concept
Helm is the package manager for Kubernetes—instead of managing separate YAML files for each environment, Helm charts use Go templates to parameterize deployments. Single chart source + multiple values files = consistent deployments across dev/staging/production.

### Key Mental Models
- **Template Abstraction**: Chart templates are YAML with placeholders ({{ .Values.replicaCount }}), values files provide concrete values—separation enables reusability
- **Release Model**: Helm tracks deployment versions (revisions)—upgrade updates Pods progressively, rollback reverts to previous revision atomically
- **Helm Repo Pattern**: Helm repos (Bitnami, Stable) curate production-ready charts—using them is faster than building from scratch

### Critical Patterns
- Create chart scaffold with `helm create`, then customize Chart.yaml, values.yaml, and templates/
- Use volumeClaimTemplates in charts for persistent deployments; use imagePullSecrets for private registries
- Define separate values files (values-dev.yaml, values-prod.yaml) to parameterize across environments
- Test templates with `helm template` before deploying—dry-run catches missing placeholders and syntax errors

### Common Mistakes
- Copying values into templates instead of using placeholders—breaks parameterization
- Creating one monolithic values file with conditionals instead of environment-specific value files (harder to maintain)
- Not testing `helm upgrade` and `helm rollback`—discover issues only in production
- Forgetting `--set` overrides when deploying—template might use defaults you didn't intend

### Connections
- **Builds on**: Raw manifests (Lessons 1-9), ConfigMaps and Secrets (Lesson 6)
- **Leads to**: GitOps workflows (charts + values in version control), multi-environment deployments at scale
