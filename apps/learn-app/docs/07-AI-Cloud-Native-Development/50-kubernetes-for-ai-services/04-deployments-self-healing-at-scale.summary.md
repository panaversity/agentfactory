### Core Concept
A Deployment is a controller that manages Pods at scale with zero downtime. You declare desired state (3 replicas, image v1.25), and the Deployment continuously ensures that state exists. When a Pod crashes, the Deployment creates a replacement. When you update the image, the Deployment performs a rolling update—swapping old Pods for new ones gradually so traffic never stops. Deployments are where Kubernetes' declarative model becomes powerful.

### Key Mental Models
- **Deployment → ReplicaSet → Pod hierarchy**—Deployments manage ReplicaSets. ReplicaSets manage Pods. This hierarchy enables rolling updates (create new ReplicaSet, scale down old one) and rollbacks (reactivate old ReplicaSet)
- **Self-healing through reconciliation**—ReplicaSet controller watches Pod count. Desired 3, actual 2? Create one more. A Pod crashes and is gone? Create a replacement. No retry logic in your app code needed.
- **Rolling updates preserve availability**—Default strategy: replace one old Pod with one new Pod at a time. During update, both old and new versions handle traffic. No downtime.
- **Instant rollback on failure**—Old ReplicaSets stay in place. New version crashes? `kubectl rollout undo` reactivates the old ReplicaSet in seconds

### Critical Patterns
- **Selector → Pod label matching**—Deployment selector finds Pods to manage. Labels in Pod template must match selector. Mismatch = infinite Pod creation loop or no Pods managed
- **Replicas for availability**—Multiple replicas mean failure is transparent. One Pod crashes, others keep serving. Client never sees downtime
- **Image updates drive reconciliation**—Change image version in Deployment spec, save, and Kubernetes automatically orchestrates the rollout. No manual Pod replacement needed
- **ReplicaSet as versioning mechanism**—Each update creates a new ReplicaSet. Old ReplicaSets are preserved for rollback history

### Common Mistakes
- Mixing Pods from different Deployments (labels from pod-template-hash prevent this, but misconfigured selectors cause chaos)
- Changing Pod template metadata.name (doesn't affect Deployment functionality but breaks readability)
- Deploying bare Pods for production work (will never auto-recover from crashes)
- Assuming old ReplicaSets are cleaned up (they're preserved for rollback history; manually delete if desired)

### Connections
- **Builds on**: Pod knowledge (Lesson 3—what Deployments manage), declarative model (Lesson 1—core concept)
- **Leads to**: Services (Lesson 5) for stable access to dynamic Pod replicas, scaling policies, and multi-container patterns
