### Core Concept
Minikube is a real Kubernetes cluster running on your laptop. It's not a toy or simulator—it uses the identical API and architecture as production Kubernetes. The only difference is scale: Minikube runs one node in a VM, production runs many nodes in data centers. Everything you learn here transfers directly to cloud deployment.

### Key Mental Models
- **kubeconfig is your connection string**—Stored at ~/.kube/config, it tells kubectl which cluster to talk to, with what credentials, and which context (namespace/user combo) to use by default
- **kubectl is your CLI to the API**—Every kubectl command goes through the API server. When you run `kubectl get pods`, you're querying the API server for Pod objects
- **Minikube as a VM wrapper**—Minikube runs a Linux VM containing both control plane and worker components. Everything inside runs as normal Kubernetes—same API, same architecture

### Critical Patterns
- **minikube start** creates a fresh cluster (takes 1-2 minutes on first run, faster on subsequent starts)
- **kubectl config use-context** switches between clusters (useful when managing Minikube + cloud clusters)
- **minikube dashboard** provides a GUI alternative to kubectl (useful for visual learners but limited compared to CLI)
- **minikube ssh** gets you inside the VM for debugging at the container level

### Common Mistakes
- Assuming Minikube needs cleanup between tests (it persists state; use `minikube delete` only when you want a fresh start)
- Forgetting to start Docker before running `minikube start` (Docker Desktop or Docker Engine is a prerequisite)
- Not understanding that kubeconfig changes are persistent (switching contexts via `kubectl config use-context` changes the default for future commands)

### Connections
- **Builds on**: Docker knowledge (Minikube uses Docker or other container runtimes)
- **Leads to**: All subsequent lessons—Pods, Deployments, Services, Networking all run on your Minikube cluster
