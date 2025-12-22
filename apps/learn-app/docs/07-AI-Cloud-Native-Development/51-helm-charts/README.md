---
sidebar_position: 51
title: "Chapter 51: Helm Charts for AI Services"
description: "Master Helm chart architecture for production AI deployments"
---

# Chapter 51: Helm Charts for AI Services

Chapter 50 introduced Helm as "the package manager for Kubernetes" in a 45-minute lesson. You learned to install charts, create basic templates, and manage releases. That was enough to deploy—but not enough to architect.

This chapter transforms you from a Helm user into a Helm chart architect. You'll master advanced Go templating, compose multi-chart dependencies, orchestrate deployment lifecycles with hooks, distribute charts via OCI registries, and create organizational standards with library charts. By the end, you'll build production-grade charts that deploy your AI agent with a single command across any environment.

## Prerequisites

**Required from Chapter 50:**
- Kubernetes architecture (control plane, workers, declarative model)
- Core primitives: Pods, Deployments, Services, ConfigMaps, Secrets
- Advanced patterns: Init containers, sidecars, StatefulSets
- Security: RBAC, SecurityContext, NetworkPolicy
- Resource management: Requests/limits, HPA
- Basic Helm: Chart structure, values.yaml, `helm install/upgrade/rollback`
- Minikube cluster setup and kubectl operations

**Proficiency Level:** B1-B2 (Intermediate to Upper-Intermediate)

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Master Advanced Templating**: Variables, pipelines, flow control, and named templates
- **Design Multi-Environment Charts**: Values hierarchy, schema validation, environment-specific configs
- **Compose Chart Dependencies**: Integrate PostgreSQL, Redis, and other subcharts
- **Orchestrate with Hooks**: Pre-install migrations, post-upgrade notifications, test hooks
- **Validate Charts**: Linting, template debugging, and integration testing
- **Distribute via OCI**: Push, pull, and install from OCI-compliant registries
- **Create Library Charts**: Organizational standards that enforce consistency
- **Build Reusable Intelligence**: A Helm Chart Architect skill for AI-native development

## Chapter Structure

### Foundation Phase (Lessons 1-3): Templating Vocabulary

| Lesson | Title | Focus |
|--------|-------|-------|
| 1 | Advanced Go Templating | Variables, pipelines, conditionals, ranges |
| 2 | Named Templates and Helpers | `_helpers.tpl`, `include` vs `template`, scope rules |
| 3 | Values Deep Dive | Hierarchy, schema validation, multi-environment configs |

### Application Phase (Lessons 4-6): Real-World Patterns

| Lesson | Title | Focus |
|--------|-------|-------|
| 4 | Chart Dependencies | Subcharts, conditions, tags, import-values |
| 5 | Helm Hooks and Lifecycle | 9 hook types, weights, delete policies |
| 6 | Testing Your Charts | `helm lint`, `helm test`, template debugging |

### Distribution Phase (Lessons 7-8): Enterprise Scaling

| Lesson | Title | Focus |
|--------|-------|-------|
| 7 | OCI Registries and Distribution | `helm push`, `helm pull`, OCI URLs |
| 8 | Library Charts and Standardization | `type: library`, organizational patterns |

### Synthesis Phase (Lessons 9-11): Intelligence and Mastery

| Lesson | Title | Layer |
|--------|-------|-------|
| 9 | AI-Assisted Chart Development | Layer 2: AI Collaboration |
| 10 | Capstone: Production AI Agent Chart | Layer 4: Spec-Driven |
| 11 | Building a Helm Chart Skill | Layer 3: Intelligence Design |

## Differentiation from Chapter 50

Chapter 50 Lesson 20 covered:
- Why Helm exists (repetitive YAML problem)
- Basic chart structure (Chart.yaml, values.yaml, templates/)
- Installing public charts (Bitnami Redis example)
- Creating simple custom charts (`helm create` scaffold)
- Basic release management (install, upgrade, rollback, uninstall)
- Environment-specific values files (dev vs prod)

**This chapter adds** (not covered in Lesson 20):
- Advanced Go templating (variables, `with`, `range`, named templates)
- Chart dependencies and subchart composition
- Helm hooks and lifecycle management
- Chart testing strategies
- OCI registry distribution
- Library charts for organizational standards
- Production patterns (umbrella charts, GitOps integration points)
- AI-assisted chart development (Layer 2)
- Reusable Helm skill creation (Layer 3)

## Looking Ahead

After mastering Helm, you'll use your charts in:
- **Chapter 52 (Kafka)**: Event-driven architecture with Helm-deployed message brokers
- **Chapter 55 (CI/CD)**: GitOps pipelines that automatically deploy your Helm charts
- **Chapter 56 (Observability)**: Monitoring charts with Prometheus and Grafana dependencies
