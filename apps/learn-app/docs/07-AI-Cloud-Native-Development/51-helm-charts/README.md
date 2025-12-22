---
sidebar_position: 51
title: "Chapter 51: Helm Charts for AI Services"
description: "Package Kubernetes deployments with Helm for repeatable, versioned releases"
---

# Chapter 51: Helm Charts for AI Services

kubectl applies individual YAML files. But production deployments have dozens of resources—deployments, services, configmaps, secrets, ingresses—that must be versioned together, configured per environment, and rolled back as a unit. Helm solves this as the "package manager for Kubernetes."

This chapter teaches you to create Helm charts that package your agent deployment. You'll learn templating, values files, releases, and how to share charts across teams. By the end, deploying your agent to any environment is a single `helm install` command.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Understand Helm concepts**: Charts, releases, repositories, and the templating engine
- **Create custom charts**: Chart structure, templates, and helpers
- **Use values files**: Environment-specific configuration (dev, staging, production)
- **Template effectively**: Go templating, built-in functions, and flow control
- **Manage releases**: Install, upgrade, rollback, and uninstall
- **Use chart dependencies**: Subcharts and importing community charts
- **Share charts**: Package and publish to chart repositories
- **Apply best practices**: Chart structure, documentation, and testing

## Chapter Structure

1. **Helm Fundamentals** — Charts, releases, and the Helm architecture
2. **Chart Structure** — Chart.yaml, values.yaml, and the templates directory
3. **Templating Basics** — Go templates, variables, and built-in functions
4. **Values & Configuration** — Default values, overrides, and environment configs
5. **Advanced Templating** — Helpers, includes, and flow control
6. **Release Management** — Install, upgrade, rollback, and history
7. **Dependencies & Subcharts** — Composing charts and using community charts
8. **Capstone: Agent Helm Chart** — Create a complete chart for your agent with dev/prod values

## Prerequisites

- Chapter 50: Kubernetes fundamentals and a running Minikube cluster
- Your agent deployed manually with kubectl

## Looking Ahead

This chapter packages your deployment. Chapter 52 (Kafka) adds event-driven capabilities, and Chapter 55 (CI/CD) automates chart deployments via GitOps.
