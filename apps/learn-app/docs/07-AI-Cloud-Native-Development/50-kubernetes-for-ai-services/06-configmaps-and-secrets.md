---
sidebar_position: 6
chapter: 50
lesson: 6
duration_minutes: 35
title: "ConfigMaps and Secrets"
proficiency_level: B1
teaching_stage: 1
stage_name: "Manual Foundation"
stage_description: "Manual configuration injection builds understanding of separation between code and configuration"
cognitive_load:
  concepts_count: 7
  scaffolding_level: "Moderate"
learning_objectives:
  - id: LO1
    description: "Understand why configuration must be separate from container images"
    bloom_level: "Understand"
  - id: LO2
    description: "Create ConfigMaps using kubectl create configmap and YAML manifests"
    bloom_level: "Apply"
  - id: LO3
    description: "Create Secrets for sensitive data using kubectl create secret generic"
    bloom_level: "Apply"
  - id: LO4
    description: "Inject configuration as environment variables using envFrom and valueFrom"
    bloom_level: "Apply"
  - id: LO5
    description: "Mount configuration as files using volumes"
    bloom_level: "Apply"
  - id: LO6
    description: "Explain that base64 encoding is NOT encryption and recognize security implications"
    bloom_level: "Understand"
  - id: LO7
    description: "Validate that Pods receive configuration correctly using kubectl exec"
    bloom_level: "Apply"
digcomp_mapping:
  - objective_id: LO1
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
  - objective_id: LO2
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO3
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO4
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO5
    competency_area: "3. Digital Content Creation"
    competency: "3.4 Programming"
  - objective_id: LO6
    competency_area: "5. Problem Solving"
    competency: "5.2 Identifying needs and technological responses"
  - objective_id: LO7
    competency_area: "5. Problem Solving"
    competency: "5.1 Solving technical problems"
---

# ConfigMaps and Secrets

In Lesson 4, you built a Deployment that ran a specific container image with a specific environment. What happens when your API key expires and you need to update it? Or when you move from development to production and need different database URLs? Rebuilding your entire container image just to change a configuration value is wasteful and error-prone.

ConfigMaps and Secrets solve this by decoupling configuration from your container image. You store configuration in Kubernetes objects, inject them into your Pods, and change configuration without rebuilding anything. ConfigMaps handle non-sensitive data (URLs, feature flags). Secrets handle sensitive data (API keys, database credentials). Both use the same injection patterns—the difference is in how they're stored and accessed.

---

## The Problem: Configuration Baked Into Images

Imagine your AI agent needs an API key to call an external service. Your Dockerfile hardcodes it:

```dockerfile
FROM python:3.11
WORKDIR /app
COPY . .
RUN pip install -r requirements.txt

# Hardcoding config—terrible practice
ENV API_KEY="sk-prod-12345"
CMD ["python", "agent.py"]
```

Problems with this approach:

1. **Same image for all environments**: Production, staging, and development all use the same image with the same API key. Security risk.
2. **Rebuilding for changes**: Update an API key? Rebuild the entire image.
3. **Secrets in registries**: Your API key is stored in the container image registry—visible to anyone with access.
4. **No separation of concerns**: The developer who builds the image shouldn't know production credentials.

Kubernetes ConfigMaps and Secrets invert this model: the image contains no configuration. Kubernetes injects configuration at runtime.

---

## ConfigMaps: Non-Sensitive Configuration

A ConfigMap is a Kubernetes object that stores key-value pairs or file content. It's not encrypted—use it for non-sensitive configuration like URLs, feature flags, and log levels.

### Creating a ConfigMap with kubectl

The fastest way to create a ConfigMap is with the kubectl command:

```bash
kubectl create configmap app-config \
  --from-literal=LOG_LEVEL=DEBUG \
  --from-literal=DATABASE_URL=postgresql://localhost:5432/mydb \
  --from-literal=FEATURE_FLAGS={"analytics":true,"beta":false}
```

**Output:**
```
configmap/app-config created
```

This creates a ConfigMap named `app-config` with three key-value pairs. Let's examine it:

```bash
kubectl get configmap app-config -o yaml
```

**Output:**
```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: app-config
  namespace: default
spec: {}
data:
  LOG_LEVEL: "DEBUG"
  DATABASE_URL: "postgresql://localhost:5432/mydb"
  FEATURE_FLAGS: '{"analytics":true,"beta":false}'
```

Notice: ConfigMaps are plain text. No encryption. They're for non-sensitive data only.

### Creating a ConfigMap with YAML Manifest

For reproducibility, define ConfigMaps in YAML:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: app-config
  namespace: default
data:
  LOG_LEVEL: "DEBUG"
  DATABASE_URL: "postgresql://localhost:5432/mydb"
  FEATURE_FLAGS: '{"analytics":true,"beta":false}'
```

Apply it:

```bash
kubectl apply -f configmap.yaml
```

**Output:**
```
configmap/app-config created
```

The YAML approach is preferred in production because it's version-controlled and reproducible.

---

## Secrets: Sensitive Data

Secrets work like ConfigMaps but are designed for sensitive data: API keys, database passwords, tokens. The key difference: Kubernetes stores Secrets separately from regular data, and some storage backends can encrypt them (though by default, Kubernetes base64-encodes Secrets, which is NOT encryption—see the security note below).

### Creating a Secret with kubectl

```bash
kubectl create secret generic db-credentials \
  --from-literal=username=admin \
  --from-literal=password=super-secret-password
```

**Output:**
```
secret/db-credentials created
```

Let's examine it:

```bash
kubectl get secret db-credentials -o yaml
```

**Output:**
```yaml
apiVersion: v1
kind: Secret
metadata:
  name: db-credentials
  namespace: default
type: Opaque
data:
  password: c3VwZXItc2VjcmV0LXBhc3N3b3Jk
  username: YWRtaW4=
```

Notice the `data` field contains base64-encoded values. Let's decode one:

```bash
echo "c3VwZXItc2VjcmV0LXBhc3N3b3Jk" | base64 --decode
```

**Output:**
```
super-secret-password
```

### Creating a Secret with YAML Manifest

For production, define Secrets in YAML with base64-encoded values:

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: db-credentials
  namespace: default
type: Opaque
data:
  username: YWRtaW4=  # base64 of "admin"
  password: c3VwZXItc2VjcmV0LXBhc3N3b3Jk  # base64 of "super-secret-password"
```

Or let Kubernetes do the encoding:

```yaml
apiVersion: v1
kind: Secret
metadata:
  name: db-credentials
  namespace: default
type: Opaque
stringData:
  username: admin
  password: super-secret-password
```

When using `stringData`, Kubernetes automatically base64-encodes the values when storing them. Apply it:

```bash
kubectl apply -f secret.yaml
```

**Output:**
```
secret/db-credentials created
```

---

## Injecting Configuration as Environment Variables

Once you have a ConfigMap or Secret, inject it into a Pod as environment variables.

### Environment Variable Injection from ConfigMap

Create a Deployment that injects the `app-config` ConfigMap:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: api-server
spec:
  replicas: 2
  selector:
    matchLabels:
      app: api-server
  template:
    metadata:
      labels:
        app: api-server
    spec:
      containers:
      - name: api
        image: myregistry/agent:v1.0
        envFrom:
        - configMapRef:
            name: app-config
        # Individual environment variables from Secrets
        env:
        - name: DB_PASSWORD
          valueFrom:
            secretKeyRef:
              name: db-credentials
              key: password
```

Let's break this down:

- **`envFrom.configMapRef`**: Injects ALL keys from the `app-config` ConfigMap as environment variables. The Pod sees `LOG_LEVEL`, `DATABASE_URL`, and `FEATURE_FLAGS`.
- **`env.valueFrom.secretKeyRef`**: Injects a single Secret key (`password` from `db-credentials`) as an environment variable named `DB_PASSWORD`.

Apply this Deployment:

```bash
kubectl apply -f deployment.yaml
```

**Output:**
```
deployment.apps/api-server created
```

Verify the Pods received the configuration:

```bash
kubectl get pods -l app=api-server
```

**Output:**
```
NAME                         READY   STATUS    RESTARTS   AGE
api-server-5d8c7f9b4-7xvzq   1/1     Running   0          15s
api-server-5d8c7f9b4-kq9mfl   1/1     Running   0          15s
```

Check the environment inside a Pod:

```bash
kubectl exec -it api-server-5d8c7f9b4-7xvzq -- env | grep LOG_LEVEL
```

**Output:**
```
LOG_LEVEL=DEBUG
```

The Pod sees the configuration value. Verify the database password is also present:

```bash
kubectl exec -it api-server-5d8c7f9b4-7xvzq -- env | grep DB_PASSWORD
```

**Output:**
```
DB_PASSWORD=super-secret-password
```

---

## Mounting Configuration as Files

Sometimes your application expects configuration in files, not environment variables. Use volume mounts to inject ConfigMaps and Secrets as files.

### Mounting a ConfigMap as a Volume

Create a ConfigMap with file-like data:

```yaml
apiVersion: v1
kind: ConfigMap
metadata:
  name: app-config-files
data:
  config.json: |
    {
      "database": "postgresql://localhost:5432/mydb",
      "log_level": "DEBUG",
      "features": {
        "analytics": true,
        "beta": false
      }
    }
  settings.env: |
    TIMEOUT=30
    RETRIES=3
```

Now mount this ConfigMap as files in a Deployment:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: api-server
spec:
  replicas: 1
  selector:
    matchLabels:
      app: api-server
  template:
    metadata:
      labels:
        app: api-server
    spec:
      containers:
      - name: api
        image: myregistry/agent:v1.0
        volumeMounts:
        - name: config-volume
          mountPath: /etc/config
      volumes:
      - name: config-volume
        configMap:
          name: app-config-files
```

Apply this Deployment:

```bash
kubectl apply -f deployment-with-volumes.yaml
```

**Output:**
```
deployment.apps/api-server created
```

Verify the files exist inside the Pod:

```bash
kubectl exec -it api-server-abc123-xyz -- ls -la /etc/config/
```

**Output:**
```
total 8
drwxr-xr-x 3 root root   120 Dec 22 10:30 .
drwxr-xr-x 1 root root  4096 Dec 22 10:30 ..
-rw-r--r-- 1 root root   156 Dec 22 10:30 config.json
-rw-r--r-- 1 root root    31 Dec 22 10:30 settings.env
```

Read the configuration file:

```bash
kubectl exec -it api-server-abc123-xyz -- cat /etc/config/config.json
```

**Output:**
```json
{
  "database": "postgresql://localhost:5432/mydb",
  "log_level": "DEBUG",
  "features": {
    "analytics": true,
    "beta": false
  }
}
```

### Mounting a Secret as a Volume

Mounting Secrets as files works identically to ConfigMaps:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: api-server
spec:
  replicas: 1
  selector:
    matchLabels:
      app: api-server
  template:
    metadata:
      labels:
        app: api-server
    spec:
      containers:
      - name: api
        image: myregistry/agent:v1.0
        volumeMounts:
        - name: secrets-volume
          mountPath: /etc/secrets
          readOnly: true
      volumes:
      - name: secrets-volume
        secret:
          secretName: db-credentials
```

Apply:

```bash
kubectl apply -f deployment-with-secrets.yaml
```

**Output:**
```
deployment.apps/api-server created
```

Verify the secret files exist:

```bash
kubectl exec -it api-server-def456-uvw -- cat /etc/secrets/password
```

**Output:**
```
super-secret-password
```

---

## Important: Base64 Encoding Is NOT Encryption

This is critical for security understanding: Kubernetes Secrets are base64-encoded, but **base64 is encoding, not encryption**. Anyone with access to the Secret object can decode it immediately.

```bash
kubectl get secret db-credentials -o yaml
```

```yaml
data:
  password: c3VwZXItc2VjcmV0LXBhc3N3b3Jk
```

This password is trivially decodable:

```bash
echo "c3VwZXItc2VjcmV0LXBhc3N3b3Jk" | base64 --decode
```

**Output:**
```
super-secret-password
```

**What this means for your deployments:**

1. **Don't treat base64 as security**: Secrets are NOT secure by default in Kubernetes.
2. **Use RBAC to control access**: Restrict who can `kubectl get secret`.
3. **Enable encryption at rest**: Many Kubernetes distributions (AWS EKS, Google GKE, Azure AKS) support encrypting Secrets in the etcd database. Enable this in production.
4. **Use external secret systems**: For highly sensitive credentials, consider external secret managers (HashiCorp Vault, AWS Secrets Manager) that integrate with Kubernetes.

For this course, understand that Secrets prevent credentials from being accidentally logged or displayed, but they're not cryptographically secure against someone with cluster access.

---

## Practice Exercises

### Exercise 1: Create a ConfigMap and Inject It

Create a ConfigMap named `agent-config` with these keys:

```
API_ENDPOINT=https://api.example.com
TIMEOUT_SECONDS=30
DEBUG_MODE=false
```

Then create a Pod that injects this ConfigMap as environment variables. Verify the Pod receives the configuration.

**Solution:**

```bash
kubectl create configmap agent-config \
  --from-literal=API_ENDPOINT=https://api.example.com \
  --from-literal=TIMEOUT_SECONDS=30 \
  --from-literal=DEBUG_MODE=false
```

Create a Pod manifest:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: agent-pod
spec:
  containers:
  - name: agent
    image: alpine
    command: ["sh", "-c", "env | grep API_ENDPOINT && sleep 3600"]
    envFrom:
    - configMapRef:
        name: agent-config
  restartPolicy: Never
```

Apply and verify:

```bash
kubectl apply -f agent-pod.yaml
kubectl exec -it agent-pod -- env | grep API_ENDPOINT
```

**Output:**
```
API_ENDPOINT=https://api.example.com
```

---

### Exercise 2: Create a Secret and Mount as Files

Create a Secret named `db-secrets` with:

```
username=postgres
password=prod-db-password
host=db.example.com
```

Mount this Secret into a Pod at `/etc/db-secrets/` and verify you can read the files.

**Solution:**

```bash
kubectl create secret generic db-secrets \
  --from-literal=username=postgres \
  --from-literal=password=prod-db-password \
  --from-literal=host=db.example.com
```

Create a Pod manifest:

```yaml
apiVersion: v1
kind: Pod
metadata:
  name: db-client
spec:
  containers:
  - name: client
    image: alpine
    command: ["sh", "-c", "cat /etc/db-secrets/password && sleep 3600"]
    volumeMounts:
    - name: db-secrets
      mountPath: /etc/db-secrets
      readOnly: true
  volumes:
  - name: db-secrets
    secret:
      secretName: db-secrets
  restartPolicy: Never
```

Apply and verify:

```bash
kubectl apply -f db-client.yaml
kubectl logs db-client
```

**Output:**
```
prod-db-password
```

---

### Exercise 3: Update a ConfigMap and Redeploy

Create a Deployment that uses a ConfigMap. Update the ConfigMap and observe whether the Pods automatically receive the new configuration.

**Solution:**

Create initial ConfigMap:

```bash
kubectl create configmap app-settings --from-literal=ENVIRONMENT=development
```

Create Deployment:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: env-printer
spec:
  replicas: 1
  selector:
    matchLabels:
      app: env-printer
  template:
    metadata:
      labels:
        app: env-printer
    spec:
      containers:
      - name: printer
        image: alpine
        command: ["sh", "-c", "echo $ENVIRONMENT && sleep 3600"]
        env:
        - name: ENVIRONMENT
          valueFrom:
            configMapKeyRef:
              name: app-settings
              key: ENVIRONMENT
  restartPolicy: Never
```

Apply:

```bash
kubectl apply -f deployment.yaml
kubectl logs -l app=env-printer
```

**Output:**
```
development
```

Now update the ConfigMap:

```bash
kubectl patch configmap app-settings -p '{"data":{"ENVIRONMENT":"production"}}'
```

**Important discovery**: The existing Pod still sees the old value. Kubernetes doesn't automatically reload ConfigMap changes into running Pods. To apply the new configuration, you must restart the Deployment:

```bash
kubectl rollout restart deployment/env-printer
kubectl logs -l app=env-printer
```

**Output:**
```
production
```

This teaches an important lesson: ConfigMaps aren't "live"—they're read when the Pod starts. To update configuration, you redeploy the Pods.

---

## Try With AI

In this section, you'll collaborate with Claude to generate ConfigMaps and Secrets for your Part 6 agent.

### Part 1: Planning Your Configuration

Your Part 6 FastAPI agent needs external configuration. You'll ask Claude to help you design which values belong in ConfigMaps and which in Secrets.

Ask Claude:

```
I'm deploying a FastAPI agent to Kubernetes. The agent needs:
- OpenAI API key (sensitive)
- PostgreSQL database URL with credentials (sensitive)
- Agent name for logging (non-sensitive)
- Request timeout in seconds (non-sensitive)
- Feature flags in JSON format (non-sensitive)
- TLS certificate path (non-sensitive, but file-based)

Which of these should go in ConfigMaps vs Secrets? Why? And provide the Kubernetes YAML manifests for both.
```

### Part 2: Critical Evaluation

Review Claude's response. Ask yourself:

- Did Claude correctly classify sensitive vs non-sensitive data?
- Did Claude create separate ConfigMap and Secret objects?
- Does the YAML follow Kubernetes conventions (metadata, kind, apiVersion)?
- Are the data keys clearly named?

### Part 3: Refinement

Based on your evaluation, tell Claude:

```
I like your classification. However, I also need:
- Agent version (for the logs)
- Maximum concurrent connections (for rate limiting)

Can you update both manifests to include these? Also, in the Deployment template,
show me how to inject the ConfigMap using envFrom and the Secret using individual
valueFrom references.
```

### Part 4: Validation

Claude generates an updated Deployment. Review it:

- Does the ConfigMap injection use `envFrom.configMapRef`?
- Does the Secret injection use `env[].valueFrom.secretKeyRef`?
- Are all the configuration keys properly referenced?

### Part 5: Implementation Check

Ask Claude:

```
Now show me the complete Deployment YAML that injects both the ConfigMap
and Secret, includes proper pod labels and replica count of 3, and includes
a liveness probe on the /health endpoint. Also add a security note about
why we're using Secrets here (not for cryptographic security, but for
access control and to prevent accidental logging).
```

This exercises your ability to prompt Claude for iterative refinement of configuration manifests—a skill you'll use frequently when deploying to Kubernetes.

