---
sidebar_position: 51
title: "Chapter 51: Helm Charts for AI Services"
description: "Master Helm chart architecture for production AI deployments"
---

# Chapter 51: Helm Charts for AI Services

Helm is the package manager for Kubernetes. Instead of managing individual YAML files, you create **Helm charts**—templated packages that parameterize deployments across environments. This chapter takes you from Helm basics to production architecture.

This chapter starts with Helm fundamentals (installation, chart structure, releases) then advances to Go templating, multi-chart dependencies, lifecycle hooks, OCI distribution, and library charts. By the end, you'll build production-grade charts that deploy your Task API (from Chapter 49) with a single command across any environment.

## Prerequisites

**Required from Chapter 50:**
- Kubernetes architecture (control plane, workers, declarative model)
- Core primitives: Pods, Deployments, Services, ConfigMaps, Secrets
- Security: RBAC, health checks
- Resource management: Requests/limits, HPA
- Docker Desktop Kubernetes cluster and kubectl operations

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

### Introduction Phase (Lesson 1): Helm Fundamentals

| Lesson | Title | Focus |
|--------|-------|-------|
| 1 | Introduction to Helm | Installation, chart structure, public charts, release management |

### Foundation Phase (Lessons 2-4): Templating Vocabulary

| Lesson | Title | Focus |
|--------|-------|-------|
| 2 | Advanced Go Templating | Variables, pipelines, conditionals, ranges |
| 3 | Named Templates and Helpers | `_helpers.tpl`, `include` vs `template`, scope rules |
| 4 | Values Deep Dive | Hierarchy, schema validation, multi-environment configs |

### Application Phase (Lessons 5-7): Real-World Patterns

| Lesson | Title | Focus |
|--------|-------|-------|
| 5 | Chart Dependencies | Subcharts, conditions, tags, import-values |
| 6 | Helm Hooks and Lifecycle | 9 hook types, weights, delete policies |
| 7 | Testing Your Charts | `helm lint`, `helm test`, template debugging |

### Distribution Phase (Lessons 8-9): Enterprise Scaling

| Lesson | Title | Focus |
|--------|-------|-------|
| 8 | OCI Registries and Distribution | `helm push`, `helm pull`, OCI URLs |
| 9 | Library Charts and Standardization | `type: library`, organizational patterns |

### Synthesis Phase (Lessons 10-12): Intelligence and Mastery

| Lesson | Title | Layer |
|--------|-------|-------|
| 10 | AI-Assisted Chart Development | Layer 2: AI Collaboration |
| 11 | Capstone: Production AI Agent Chart | Layer 4: Spec-Driven |
| 12 | Building a Helm Chart Skill | Layer 3: Intelligence Design |

## What Lesson 1 Covers

Lesson 1 (Introduction to Helm) teaches:
- Why Helm exists (repetitive YAML problem)
- Basic chart structure (Chart.yaml, values.yaml, templates/)
- Installing public charts (Bitnami Redis example)
- Creating simple custom charts (`helm create` scaffold)
- Basic release management (install, upgrade, rollback, uninstall)
- Environment-specific values files (dev vs prod)

**Lessons 2-12 build on this foundation** with:
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
