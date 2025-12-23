---
sidebar_position: 14
title: "Secrets Management for GitOps"
description: "Handle secrets safely without storing them in Git"
keywords: [argocd, secrets, sealed secrets, external secrets, vault, gitops, security]
---

# Secrets Management for GitOps

You've deployed applications with ArgoCD, configured sync policies, and mastered ApplicationSets. But you haven't stored any secrets in Git yet—and for good reason. Committing API keys, database passwords, or authentication tokens to version control is a security breach waiting to happen.

GitOps requires everything to be in Git, but secrets are the exception: they must never be stored in plaintext in version control. This lesson teaches you patterns for handling secrets safely while maintaining the GitOps principle of "Git as truth."

By the end, you'll understand:

1. Why plaintext secrets in Git are dangerous
2. How to encrypt secrets using Sealed Secrets (Bitnami approach)
3. How to sync secrets from external stores using External Secrets Operator
4. When to use HashiCorp Vault integration through ArgoCD plugins
5. Best practices for API key rotation and access control
6. How to deploy applications that need secrets without committing those secrets

## The Problem: Secrets in Git

Imagine your Python FastAPI agent needs:

- OpenAI API key (`OPENAI_API_KEY=sk-...`)
- Database password (`DB_PASSWORD=secure123!`)
- JWT signing key (`JWT_SECRET=abc123xyz...`)

In non-GitOps workflows, you might:

1. Store secrets in a `.env` file (never commit to Git)
2. Pass them via environment variables at deployment time
3. Manage them manually in each environment

But GitOps says everything should be in Git—including the specification of what secrets your application needs. This creates a tension:

**Requirement 1**: Everything must be versioned in Git (GitOps principle)
**Requirement 2**: Secrets must never be in plaintext in Git (security principle)

These aren't contradictory if you encrypt secrets at rest (in Git) while keeping them decrypted at runtime (in the cluster).

### Why Plaintext Secrets in Git Are Dangerous

1. **Permanent history**: Once committed, secrets exist in Git history forever (even if you delete them)
2. **Broad access**: Everyone with repository access can read plaintext secrets
3. **Audit trail**: No way to know who accessed or rotated secrets
4. **Accidental exposure**: Easy to commit `.env` files, API keys in comments, etc.

Even private repositories are risky—developers accidentally grant too many permissions, contractors get access, or repositories are acquired in acquisitions where permissions aren't immediately revoked.

## Sealed Secrets: Encrypt Secrets with the Cluster Key

**Sealed Secrets** (by Bitnami) is the simplest approach for Kubernetes-native secret management.

### How Sealed Secrets Works

1. **Cluster generates a key pair** during Sealed Secrets installation
2. **You encrypt plaintext secrets** using the public key
3. **Encrypted secrets go into Git** (they look like gibberish)
4. **ArgoCD applies the encrypted YAML** to the cluster
5. **The controller automatically decrypts** using the private key
6. **Your application reads the plaintext Secret** from the cluster

The private key never leaves the cluster, so only your cluster can decrypt the secrets.

### Install Sealed Secrets

First, install the Sealed Secrets controller:

```bash
kubectl apply -f https://github.com/bitnami-labs/sealed-secrets/releases/download/v0.18.0/controller.yaml
```

**Output:**
```
namespace/sealed-secrets created
serviceaccount/sealed-secrets-key created
clusterrolebinding.rbac.authorization.k8s.io/sealed-secrets-service-accounts-sealer created
deployment.apps/sealed-secrets-controller created
service/sealed-secrets created
```

Verify it's running:

```bash
kubectl get pods -n sealed-secrets
```

**Output:**
```
NAME                                     READY   STATUS    RESTARTS   AGE
sealed-secrets-controller-749df74fb7c   1/1     Running   0          30s
```

### Create an Encrypted Secret

You need the `kubeseal` CLI tool to encrypt secrets locally:

```bash
# macOS
brew install kubeseal

# Linux
wget https://github.com/bitnami-labs/sealed-secrets/releases/download/v0.18.0/kubeseal-0.18.0-linux-amd64.tar.gz
tar xfz kubeseal-0.18.0-linux-amd64.tar.gz
sudo install -m 755 kubeseal /usr/local/bin/kubeseal
```

**Output** (after install):
```
kubeseal 0.18.0
```

Now, encrypt a secret. First, create a plaintext Kubernetes Secret manifest:

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: agent-secrets
  namespace: production
type: Opaque
stringData:
  OPENAI_API_KEY: sk-proj-abc123xyz789...
  DB_PASSWORD: my-secure-password-123
```

Encrypt it:

```bash
kubeseal -f secret.yaml -w sealed-secret.yaml
```

**Output:**
```
secret "agent-secrets" sealed
```

Examine the encrypted output:

```bash
cat sealed-secret.yaml
```

**Output:**
```yaml
apiVersion: bitnami.com/v1alpha1
kind: SealedSecret
metadata:
  name: agent-secrets
  namespace: production
spec:
  encryptedData:
    DB_PASSWORD: AgBvK3x+K9p8qL...mK9vL8mN9oO0pP1...
    OPENAI_API_KEY: AgCwD4eF7gH1iJ...kL2mN3oP4qR5sT6...
  template:
    metadata:
      name: agent-secrets
      namespace: production
    type: Opaque
```

The values are unreadable. This is safe to commit to Git.

### Commit and Deploy

Commit the sealed secret to Git:

```bash
git add sealed-secret.yaml
git commit -m "Add sealed secrets for agent"
git push
```

**Output:**
```
[main abc1234] Add sealed secrets for agent
 1 file changed, 15 insertions(+)
```

Create an ArgoCD Application that includes this sealed secret:

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-with-secrets
  namespace: argocd
spec:
  project: default
  source:
    repoURL: https://github.com/yourname/agent-config.git
    path: .
    targetRevision: HEAD
  destination:
    server: https://kubernetes.default.svc
    namespace: production
  syncPolicy:
    automated:
      prune: true
      selfHeal: true
    syncOptions:
    - CreateNamespace=true
```

**Output** (after sync):
```
NAME: agent-with-secrets
STATUS: Synced
HEALTH: Healthy

Resources:
  - SealedSecret: agent-secrets (Synced)
  - Deployment: agent (Synced)
```

Your application now has access to plaintext secrets without them being stored in Git:

```bash
kubectl get secret agent-secrets -n production -o yaml
```

**Output:**
```yaml
apiVersion: v1
kind: Secret
metadata:
  name: agent-secrets
  namespace: production
type: Opaque
data:
  OPENAI_API_KEY: c2stcHJvai1hYmMxMjN4eXo3ODkuLi4=  # base64 encoded
  DB_PASSWORD: bXktc2VjdXJlLXBhc3N3b3JkLTEyMw==   # base64 encoded
```

The secret is decrypted in the cluster. Your Deployment mounts it:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent
spec:
  template:
    spec:
      containers:
      - name: agent
        image: agent:latest
        envFrom:
        - secretRef:
            name: agent-secrets
```

Your container sees the plaintext environment variables.

## External Secrets Operator: Sync from External Stores

**Sealed Secrets** work well for small, static secrets. But if you use AWS Secrets Manager, Azure Key Vault, or HashiCorp Vault, **External Secrets Operator** keeps those systems as the single source of truth and syncs them into Kubernetes.

### How External Secrets Works

1. **You store secrets in an external vault** (AWS Secrets Manager, Vault, etc.)
2. **External Secrets creates a Kubernetes Secret** that mirrors the vault
3. **ArgoCD manages the ExternalSecret CRD** (which goes in Git)
4. **The controller automatically syncs updates** from vault to cluster
5. **Your application reads the mirrored Secret** just like before

### Install External Secrets Operator

```bash
helm repo add external-secrets https://external-secrets.github.io/external-secrets
helm repo update
helm install external-secrets external-secrets/external-secrets \
  -n external-secrets-system --create-namespace
```

**Output:**
```
NAME: external-secrets
LAST DEPLOYED: Fri Dec 23 12:34:56 2025
NAMESPACE: external-secrets-system
STATUS: deployed
REVISION: 1
```

### Create a SecretStore (Vault Example)

A `SecretStore` defines how to connect to your external vault:

```yaml
apiVersion: external-secrets.io/v1beta1
kind: SecretStore
metadata:
  name: vault-store
  namespace: production
spec:
  provider:
    vault:
      server: "https://vault.example.com:8200"
      path: "secret"
      auth:
        kubernetes:
          mountPath: "kubernetes"
          role: "agent-role"
```

**Output** (after apply):
```
secretstore.external-secrets.io/vault-store created
```

### Create an ExternalSecret

An `ExternalSecret` specifies which vault secrets to sync:

```yaml
apiVersion: external-secrets.io/v1beta1
kind: ExternalSecret
metadata:
  name: agent-secrets
  namespace: production
spec:
  refreshInterval: 1h          # Sync every hour
  secretStoreRef:
    name: vault-store
    kind: SecretStore
  target:
    name: agent-secrets        # Name of the Kubernetes Secret created
    creationPolicy: Owner
  data:
  - secretKey: OPENAI_API_KEY
    remoteRef:
      key: agent/openai-key    # Path in Vault
  - secretKey: DB_PASSWORD
    remoteRef:
      key: agent/db-password
```

Apply it:

```bash
kubectl apply -f external-secret.yaml
```

**Output:**
```
externalsecret.external-secrets.io/agent-secrets created
```

The External Secrets controller immediately syncs from Vault. Check the Secret:

```bash
kubectl get secret agent-secrets -n production
```

**Output:**
```
NAME            TYPE     DATA   AGE
agent-secrets   Opaque   2      15s
```

Your Deployment mounts it exactly like a Sealed Secret:

```yaml
envFrom:
- secretRef:
    name: agent-secrets
```

The difference: secrets now stay synchronized with Vault. If you rotate a secret in Vault, the Kubernetes Secret updates automatically within 1 hour (or immediately with a webhook trigger).

## When to Use Which Approach

| Pattern | Best For | Complexity | Cost |
|---------|----------|-----------|------|
| **Sealed Secrets** | Simple static secrets, single cluster, learning | Low | Free |
| **External Secrets + Vault** | Multiple clusters, secret rotation, audit trails | High | Free (Vault) or Paid (HashiCorp Cloud) |
| **External Secrets + AWS Secrets Manager** | AWS-native teams, managed service preferred | Medium | Pay-per-secret |
| **External Secrets + Azure Key Vault** | Azure-native teams, managed service preferred | Medium | Included in Azure subscription |

For this course, **Sealed Secrets** is sufficient. For production deployments in enterprises, **External Secrets + Vault** is standard.

## Best Practices for API Key Management

### 1. Never Commit Secrets

Create a `.gitignore` rule:

```
# .gitignore
secret.yaml
sealed-secret.yaml
.env
.env.local
```

### 2. Rotate Secrets Regularly

For OpenAI, Anthropic, and other provider keys:

1. **Set an expiration date** in your notes (e.g., rotate every 90 days)
2. **Create a new key** in the provider dashboard
3. **Update the Sealed Secret or Vault**
4. **Delete the old key** from the provider
5. **Restart the deployment** so containers pick up the new key

Example rotation for an OpenAI key:

```bash
# Create new key in OpenAI dashboard, note it as: sk-proj-new-key...

# Create plaintext secret with new key
cat > secret-updated.yaml <<EOF
apiVersion: v1
kind: Secret
metadata:
  name: agent-secrets
  namespace: production
type: Opaque
stringData:
  OPENAI_API_KEY: sk-proj-new-key...
EOF

# Encrypt it
kubeseal -f secret-updated.yaml -w sealed-secret.yaml

# Commit and push
git add sealed-secret.yaml
git commit -m "Rotate OpenAI API key (90-day rotation)"
git push

# Restart pods
kubectl rollout restart deployment/agent -n production
```

**Output**:
```
deployment.apps/agent restarted
```

### 3. Limit Secret Scope

Use Kubernetes namespace isolation:

- **`production` namespace**: Sealed secrets for production API keys
- **`staging` namespace**: Sealed secrets for staging API keys (different keys)
- **`development` namespace**: Sealed secrets for dev API keys (lower security)

Each namespace has its own Sealed Secret, encrypted with the cluster key, but never mixed.

### 4. Audit Secret Access

For production deployments, use Vault with audit logging:

```yaml
apiVersion: external-secrets.io/v1beta1
kind: SecretStore
metadata:
  name: vault-audit
spec:
  provider:
    vault:
      server: https://vault.company.com:8200
      path: secret
      auth:
        kubernetes:
          role: agent-audit-role
```

Vault logs every secret access. Check the logs:

```bash
# In Vault
vault audit list
vault audit enable file file_path=/var/log/vault-audit.log
```

**Output:**
```
Path      Type       Description
----      ----       -----------
file/     file       File backend
```

### 5. Separate Secrets by Environment

Production secrets should NOT be the same as development secrets:

```yaml
# production/sealed-secret.yaml
apiVersion: bitnami.com/v1alpha1
kind: SealedSecret
metadata:
  name: agent-secrets
  namespace: production
spec:
  encryptedData:
    OPENAI_API_KEY: AgCpQ3r9S7T1uV... # Production key

---
# development/sealed-secret.yaml
apiVersion: bitnami.com/v1alpha1
kind: SealedSecret
metadata:
  name: agent-secrets
  namespace: development
spec:
  encryptedData:
    OPENAI_API_KEY: AgCqR4s0T8u2vW... # Development key (different)
```

If a development key is compromised, production remains secure.

## Complete Example: FastAPI Agent with Secrets

Here's a full example deploying your agent with Sealed Secrets:

**Step 1: Create plaintext secret**

```yaml
# secret.yaml (never commit this)
apiVersion: v1
kind: Secret
metadata:
  name: agent-api-keys
  namespace: production
type: Opaque
stringData:
  OPENAI_API_KEY: sk-proj-abc123...
  ANTHROPIC_API_KEY: sk-ant-def456...
  DATABASE_URL: postgresql://user:pass@db:5432/agent
```

**Step 2: Encrypt it**

```bash
kubeseal -f secret.yaml -w sealed-secret.yaml
```

**Output:**
```
secret "agent-secrets" sealed
```

**Step 3: Commit encrypted version**

```bash
git add sealed-secret.yaml
git commit -m "Add API key secrets for agent"
git push
```

**Output:**
```
[main abc1234] Add API key secrets for agent
 1 file changed, 15 insertions(+)
```

**Step 4: Create ArgoCD Application**

```yaml
apiVersion: argoproj.io/v1alpha1
kind: Application
metadata:
  name: agent-fastapi
  namespace: argocd
spec:
  project: default
  source:
    repoURL: https://github.com/yourname/agent-deployment.git
    path: k8s
    targetRevision: HEAD
  destination:
    server: https://kubernetes.default.svc
    namespace: production
  syncPolicy:
    automated:
      prune: true
      selfHeal: true
    syncOptions:
    - CreateNamespace=true
```

**Step 5: Create Deployment that uses the secret**

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: agent
  namespace: production
spec:
  replicas: 2
  selector:
    matchLabels:
      app: agent
  template:
    metadata:
      labels:
        app: agent
    spec:
      containers:
      - name: agent
        image: yourregistry/agent:latest
        envFrom:
        - secretRef:
            name: agent-api-keys
        ports:
        - containerPort: 8000
```

Apply everything:

```bash
kubectl apply -f sealed-secret.yaml
kubectl apply -f app.yaml
```

**Output:**
```
sealedsecret.bitnami.com/agent-api-keys created
deployment.apps/agent created
service/agent created
```

Your agent is now running with encrypted secrets, safely committed to Git.

## Comparison: Secret Patterns

| Aspect | Sealed Secrets | External Secrets | Vault Plugin |
|--------|---|---|---|
| **Setup complexity** | Low | Medium | High |
| **Encryption** | Cluster public key | External vault | Vault |
| **Rotation** | Manual kubeseal | Auto-sync | Auto-sync |
| **Multi-cluster** | Each cluster separate | Shared source | Shared source |
| **Audit trail** | Kubernetes events | Vault logs | Vault logs |
| **Cost** | Free | Free + vault cost | Vault subscription |
| **Recommended for** | Learning, small deployments | Production, enterprises | Large-scale organizations |

For this course and small production deployments, **Sealed Secrets** is ideal. For enterprises with strict audit and rotation requirements, **External Secrets Operator** with HashiCorp Vault is the standard.

## Try With AI

**Setup**: You have an OpenAI API key and a PostgreSQL connection string that need to be deployed with your FastAPI agent.

**Part 1: Initial Request**

Ask AI: "I need to securely store my OpenAI API key and PostgreSQL connection string in Kubernetes. I want to keep them encrypted in Git. Should I use Sealed Secrets or External Secrets Operator? What are the tradeoffs?"

**Part 2: Critical Evaluation**

Review AI's response. Ask yourself:

- Does AI explain when each approach is appropriate?
- Which approach seems simpler for your current setup?
- What assumptions did AI make about your infrastructure?

**Part 3: Share Your Constraints**

Tell AI your constraints: "We're learning GitOps in a Minikube environment, so we need something that doesn't require external services. What's the minimal setup for Sealed Secrets?"

**Part 4: Refinement**

Ask AI to help you: "Generate the kubeseal encryption steps and the YAML manifest for my FastAPI agent Deployment that reads these secrets."

**Part 5: Final Check**

Compare your result:

- Can you trace how secrets flow from plaintext → kubeseal → encrypted YAML → Kubernetes Secret → environment variables?
- Would this approach work for rotating your OpenAI key in 90 days?
- Could you adapt this for External Secrets later when you move to production?

**Safety note**: Never paste your actual API keys into public AI conversations. Use placeholder values like `sk-proj-example-key-for-demo` and test with real keys locally.
