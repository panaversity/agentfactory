---
sidebar_position: 52
title: "Chapter 52: Event-Driven Architecture with Kafka"
description: "Build decoupled, scalable agent systems with Apache Kafka event streaming"
---

# Chapter 52: Event-Driven Architecture with Kafka

Request-response APIs work for simple interactions. But production agent systems need decoupling—when a task is created, a notification service should be triggered, an audit log should be written, and a recurring task engine should be notified. If these are direct API calls, one slow service blocks everything. If they're events on Kafka, each service consumes independently.

This chapter provides comprehensive Kafka coverage for AI agent developers. You'll progress from EDA fundamentals through production reliability patterns, learning to build event-driven systems that scale. The chapter uses Docker Compose for local development (Lessons 1-17), then deploys to Kubernetes with Strimzi (Lesson 18).

**Key Update (2025):** Kafka 4.0 removed ZooKeeper entirely. This chapter teaches KRaft-only deployment—the modern, simplified architecture.

## What You'll Learn

By the end of this chapter, you'll be able to:

- **Explain why events beat direct calls**: Coupling problems, async benefits, when to use EDA
- **Understand Kafka architecture**: Brokers, topics, partitions, consumer groups, offsets (KRaft mode)
- **Implement reliable producers**: acks semantics, retries, idempotent producer, error handling
- **Implement robust consumers**: Consumer groups, rebalancing, offset management, lag monitoring
- **Integrate with FastAPI**: Async producers/consumers, lifespan events, background tasks
- **Design event schemas**: Avro with Schema Registry, schema evolution, breaking change prevention
- **Apply delivery guarantees**: At-least-once, at-most-once, exactly-once semantics and trade-offs
- **Use transactions**: Consume-process-produce pattern, zombie fencing, read_committed isolation
- **Build data pipelines**: Kafka Connect, Debezium CDC, outbox pattern for microservices
- **Implement agent patterns**: Task events, notification fanout, audit logs, saga pattern
- **Deploy to Kubernetes**: Strimzi operator, Helm charts, production configuration
- **Debug production issues**: Consumer lag, under-replicated partitions, rebalancing storms

## Chapter Structure

### Part A: EDA Foundations (Lessons 1-3)

| # | Lesson | Focus |
|---|--------|-------|
| 1 | From Request-Response to Events | Why direct API calls fail at scale, coupling problems, async benefits |
| 2 | Event-Driven Architecture Concepts | Events vs commands, event sourcing intro, CQRS overview, when to use EDA |
| 3 | How Kafka Fits: The Mental Model | Topics, partitions, producers, consumers, brokers, offsets — visual + analogies |

### Part B: Kafka Core (Lessons 4-8)

| # | Lesson | Focus |
|---|--------|-------|
| 4 | Running Kafka Locally (KRaft Mode) | Docker Compose with Redpanda or Kafka KRaft, UI tools (Kafka UI, Redpanda Console) |
| 5 | Your First Producer (Python) | confluent-kafka-python, sync send, fire-and-forget vs sync vs async |
| 6 | Producer Deep Dive: Reliability | acks (0, 1, all), retries, delivery.timeout.ms, idempotent producer |
| 7 | Your First Consumer (Python) | Consumer groups, poll loop, auto-commit vs manual, offset management |
| 8 | Consumer Deep Dive: Groups & Rebalancing | Partition assignment, rebalance listeners, static membership, consumer lag |

### Part C: Production Patterns (Lessons 9-13)

| # | Lesson | Focus |
|---|--------|-------|
| 9 | Async Producers & Consumers in FastAPI | AIOProducer, async consumer patterns, lifespan events, background tasks |
| 10 | Message Schemas: Avro & Schema Registry | Why schemas, Avro basics, Schema Registry, evolution strategies |
| 11 | Delivery Semantics Deep Dive | At-most-once, at-least-once, exactly-once trade-offs, idempotent producer limits |
| 12 | Transactions for Stream Processing | Consume-process-produce, transactional.id, zombie fencing, read_committed |
| 13 | Reliability Configuration | Replication factor, min.insync.replicas, unclean leader election, ISR |

### Part D: Data Pipelines (Lessons 14-15)

| # | Lesson | Focus |
|---|--------|-------|
| 14 | Kafka Connect: Building Data Pipelines | Source vs sink connectors, REST API, when to use Connect vs client |
| 15 | Change Data Capture with Debezium | CDC vs polling, Debezium for Postgres, outbox pattern for atomicity |

### Part E: Agent Communication Patterns (Lessons 16-17)

| # | Lesson | Focus |
|---|--------|-------|
| 16 | Agent Event Patterns | Task lifecycle events, notification fanout, immutable audit log, naming conventions |
| 17 | Saga Pattern for Multi-Step Workflows | Choreography vs orchestration, compensation events, implementing saga |

### Part F: Deployment & Operations (Lessons 18-19)

| # | Lesson | Focus |
|---|--------|-------|
| 18 | Kafka on Kubernetes: Strimzi Operator | Strimzi CRDs, KafkaCluster, KafkaTopic, Helm chart deployment |
| 19 | Monitoring & Debugging Kafka | Consumer lag, under-replicated partitions, key metrics, tooling |

### Part G: AI Collaboration & Capstone (Lessons 20-22)

| # | Lesson | Focus |
|---|--------|-------|
| 20 | AI-Assisted Kafka Development | Use Claude to debug consumer lag, generate Avro schemas, optimize configs |
| 21 | Capstone: Event-Driven Agent Notifications | Spec-driven: add Kafka events to Part 6 agent (task.created, audit log) |
| 22 | Building the Kafka Event Schema Skill | Reusable skill for designing event schemas and topic structures |

## Prerequisites

- **Chapter 49**: Docker fundamentals (containers, Compose, volumes, networks)
- **Chapter 50**: Kubernetes basics (Pods, Deployments, Services)
- **Chapter 51**: Helm Charts (for Strimzi deployment)
- **Part 6**: Your FastAPI agent service
- **Python**: Async patterns (async/await, asyncio)

## Technology Choices

| Component | Choice | Rationale |
|-----------|--------|-----------|
| **Local Kafka** | Redpanda or Kafka KRaft | No ZooKeeper (Kafka 4.0+), simpler setup |
| **Python Client** | confluent-kafka-python | Best performance, native async, Schema Registry support |
| **Schemas** | Avro + Confluent Schema Registry | Industry standard, evolution support |
| **K8s Deployment** | Strimzi Operator | Standard for Kafka on Kubernetes |
| **CDC** | Debezium | Best-in-class change data capture |

## What's NOT Covered

This chapter focuses on **developer skills**, not SRE operations:

- Multi-datacenter replication (MirrorMaker 2)
- Security deep dive (SASL, SSL, ACLs) — covered at overview level only
- Kafka Streams framework — separate advanced topic
- Broker hardware sizing and tuning
- ZooKeeper — removed in Kafka 4.0

## Looking Ahead

This chapter teaches Kafka directly. Chapter 53 (Dapr) shows how to abstract pub/sub behind Dapr's API, making your code portable across message brokers while retaining the concepts you learned here.
