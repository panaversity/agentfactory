# Google ADK Deployment Patterns

## Environment Setup

### Development (Gemini API)

```bash
# .env
GOOGLE_API_KEY=your_api_key
```

### Production (Vertex AI)

```bash
# .env
GOOGLE_GENAI_USE_VERTEXAI=TRUE
GOOGLE_CLOUD_PROJECT=your_project_id
GOOGLE_CLOUD_LOCATION=us-central1
```

```bash
# Authenticate
gcloud auth application-default login
gcloud services enable aiplatform.googleapis.com
```

## Deployment Options

### 1. Vertex AI Agent Engine (Recommended)

Fully managed, scalable, enterprise-grade.

```bash
# CLI deployment
uv run adk deploy agent_engine \
  --project=your-project-id \
  --region=us-central1 \
  --staging_bucket="gs://your-bucket" \
  --display_name="My Agent" \
  ./my_agent
```

```python
# Python SDK deployment
from vertexai.preview import reasoning_engines
from vertexai import agent_engines
import vertexai

vertexai.init(
    project="your-project",
    location="us-central1",
    staging_bucket="gs://your-bucket"
)

adk_app = reasoning_engines.AdkApp(
    agent=root_agent,
    enable_tracing=True  # Cloud Trace integration
)

remote_app = agent_engines.create(
    agent_engine=adk_app,
    extra_packages=["./my_agent"],
    requirements=["google-cloud-aiplatform[adk,agent_engines]"]
)
```

### 2. Cloud Run (Containerized)

Full control, custom scaling.

```bash
# CLI deployment
uv run adk deploy cloud_run \
  --project=your-project-id \
  --region=us-central1 \
  --service_name=my-agent-service \
  ./my_agent
```

```bash
# Manual deployment with gcloud
gcloud run deploy my-agent-service \
  --source . \
  --region us-central1 \
  --project your-project-id \
  --allow-unauthenticated \
  --set-env-vars="GOOGLE_CLOUD_PROJECT=your-project-id,GOOGLE_CLOUD_LOCATION=us-central1,GOOGLE_GENAI_USE_VERTEXAI=TRUE"
```

### 3. GKE (Kubernetes)

Enterprise, multi-service deployment.

```yaml
# deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: adk-agent
spec:
  replicas: 3
  selector:
    matchLabels:
      app: adk-agent
  template:
    metadata:
      labels:
        app: adk-agent
    spec:
      containers:
      - name: agent
        image: gcr.io/your-project/adk-agent:latest
        ports:
        - containerPort: 8080
        env:
        - name: GOOGLE_GENAI_USE_VERTEXAI
          value: "TRUE"
        - name: GOOGLE_CLOUD_PROJECT
          valueFrom:
            secretKeyRef:
              name: gcp-config
              key: project-id
```

## MCP in Production

### Cloud Run with Remote MCP

```python
import os

if os.getenv('K_SERVICE'):  # Cloud Run environment
    mcp_connection = SseConnectionParams(
        url=os.getenv('MCP_SERVER_URL'),
        headers={'Authorization': f"Bearer {os.getenv('MCP_AUTH_TOKEN')}"}
    )
else:  # Local development
    mcp_connection = StdioConnectionParams(
        server_params=StdioServerParameters(
            command='npx',
            args=["-y", "@modelcontextprotocol/server-filesystem", "/tmp"]
        )
    )
```

### GKE with Service Discovery

```python
# Use Kubernetes service discovery
mcp_connection = SseConnectionParams(
    url="http://mcp-service.default.svc.cluster.local:8080/sse"
)
```

### Vertex AI with External MCP

```python
# Managed deployment with authenticated MCP
mcp_connection = SseConnectionParams(
    url="https://your-mcp-service.googleapis.com/sse",
    headers={'Authorization': 'Bearer $(gcloud auth print-access-token)'}
)
```

## Testing Deployed Agents

### Verify Deployment

```bash
# List deployed agents
curl -X GET \
  -H "Authorization: Bearer $(gcloud auth print-access-token)" \
  "https://us-central1-aiplatform.googleapis.com/v1/projects/your-project/locations/us-central1/reasoningEngines"
```

### Test Agent

```python
# tests/test_deployment.py
from vertexai import agent_engines

remote_app = agent_engines.get(
    resource_name="projects/your-project/locations/us-central1/reasoningEngines/engine-id"
)

response = remote_app.query(
    user_id="test-user",
    message="Hello!"
)
print(response)
```

## Observability

### Cloud Trace Integration

```python
# Enable in deployment
adk_app = reasoning_engines.AdkApp(
    agent=root_agent,
    enable_tracing=True  # Sends traces to Cloud Trace
)
```

### Logging

```python
import logging

# ADK uses standard Python logging
logging.basicConfig(level=logging.INFO)

# View in Cloud Logging when deployed
```

## Security Patterns

### Service Account Authentication

```yaml
# Cloud Run IAM
gcloud run services add-iam-policy-binding my-agent-service \
  --member="serviceAccount:sa@project.iam.gserviceaccount.com" \
  --role="roles/run.invoker"
```

### API Key Protection

```python
# Never commit API keys
# Use Secret Manager in production
from google.cloud import secretmanager

def get_secret(secret_id: str) -> str:
    client = secretmanager.SecretManagerServiceClient()
    name = f"projects/your-project/secrets/{secret_id}/versions/latest"
    response = client.access_secret_version(name=name)
    return response.payload.data.decode("UTF-8")
```

## Scaling Patterns

### Cloud Run Auto-scaling

```bash
gcloud run services update my-agent-service \
  --min-instances=1 \
  --max-instances=100 \
  --concurrency=80
```

### Agent Engine Scaling

Automatic, managed by Vertex AI.

## Cost Optimization

| Pattern | Cost Impact | When to Use |
|---------|-------------|-------------|
| Agent Engine | Higher base, managed | Enterprise, production |
| Cloud Run | Pay per request | Variable traffic |
| GKE | Predictable | Multi-service systems |

## Troubleshooting

| Issue | Check | Fix |
|-------|-------|-----|
| 403 Forbidden | IAM permissions | Enable Vertex AI API, check service account |
| Timeout | Agent complexity | Increase timeout, simplify agent |
| Cold start | Min instances | Set min-instances=1 |
| Memory errors | Container limits | Increase memory allocation |
