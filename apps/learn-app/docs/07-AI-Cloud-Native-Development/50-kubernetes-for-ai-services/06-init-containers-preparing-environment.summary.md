### Core Concept
Init containers run before application containers and must complete successfully before the app starts. They solve the "dependencies ready" problem: download ML models, verify database connectivity, wait for configuration—all before your production code runs. No retry logic in your app needed; init container failures simply restart the whole Pod.

### Key Mental Models
- **Sequential execution guarantee**—Multiple init containers run one at a time, in order. Init 1 completes, then Init 2 starts, then Init 3, only then app containers. Impossible for race conditions between init phases
- **Success is mandatory**—If any init container exits with non-zero status, Kubernetes restarts the whole Pod and retries. This is different from regular containers that might keep running despite failures
- **Shared volumes bridge init and app**—Init container downloads 500MB model to /models. App container reads from /models on the same volume. Both see same filesystem
- **Pod delay vs app resilience**—Init containers add startup latency but remove runtime crashes. Better to wait 30 seconds at startup for model download than to have the app crash 100 times during startup

### Critical Patterns
- **Multi-step initialization**—Chain init containers: create directories, verify config, download artifacts. Each step is a separate init container; failures in step N restart from step 1
- **Retry with backoff**—Init containers can implement retry loops internally (while loop with sleep). Kubernetes also retries the whole Pod on failure
- **Volume sharing between init and app**—emptyDir volumes enable data passing. Init writes, app reads. On Pod termination, data is lost (which is fine for temporary setup)
- **Resource limits per init container**—Init containers can use different CPU/memory limits than app. Setup might be CPU-heavy, app might be memory-heavy

### Common Mistakes
- Assuming failed init container blocks subsequent inits indefinitely (Kubernetes restarts the whole Pod on first init failure)
- Not mounting shared volumes (init and app can't communicate about setup state)
- Making init containers too complex (should be simple setup; complex logic belongs in app)
- Ignoring init container logs (when Pod won't start, init logs reveal the blocker; use `kubectl logs -c init-container-name`)

### Connections
- **Builds on**: Pods (Lesson 3)—init containers are part of Pod spec
- **Leads to**: Sidecar containers (Lesson 7) which also use init container syntax with restartPolicy:Always
