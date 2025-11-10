# Feature Specification: Usage Data Collection System

**Feature Branch**: `017-usage-data-collection`  
**Created**: 2025-01-10  
**Status**: Draft  
**Input**: User description: "How can I set this up so I can collect all my data https://code.claude.com/docs/en/monitoring-usage as andrew ng suggested about data silos for later interesting things https://www.deeplearning.ai/the-batch/tear-down-data-silos/ https://www.deeplearning.ai/the-batch/improve-agentic-performance-with-evals-and-error-analysis-part-2/. The idea is to setup data collection and a high level strategy to later use it. This component will be later available for all team working on this book"

## User Scenarios & Testing

### User Story 1 - Enable Usage Data Collection (Priority: P1)

**As a** book development team member  
**I want to** automatically collect Claude Code usage data during my work sessions  
**So that** we can analyze agent performance and improve content quality through evals-first development

**Why this priority**: This is the foundational capability that enables all downstream analysis. Without data collection, no evaluation or improvement workflow is possible. This establishes the data infrastructure that Andrew Ng emphasizes as critical for agentic AI improvement.

**Independent Test**: Can be fully tested by running Claude Code with telemetry enabled and verifying that usage data (prompts, tool calls, token counts, costs) is exported to the configured backend without manual intervention. Delivers immediate value by capturing workflow traces for manual inspection.

**Acceptance Scenarios**:

1. **Given** a team member starts a Claude Code session with telemetry enabled, **When** they execute any prompt or tool call, **Then** the system exports user prompt events, tool result events, API request events, and token usage metrics to the configured OpenTelemetry backend
2. **Given** telemetry is configured with multiple exporters (console + OTLP), **When** a Claude Code session runs for 30 minutes with 50+ interactions, **Then** all events and metrics are successfully exported to both backends without data loss or session interruption
3. **Given** a team member completes a typical chapter writing workflow (spec → plan → implement → validate), **When** they review the exported telemetry data, **Then** they can identify all major workflow steps (prompt submissions, file edits, test runs) with complete context (session ID, timestamps, token counts, costs)

---

### User Story 2 - Centralized Data Storage (Priority: P2)

**As a** book project coordinator  
**I want to** aggregate all team members' usage data into a centralized queryable database  
**So that** we can break down data silos and enable cross-team analysis as Andrew Ng recommends

**Why this priority**: Once individual data collection works (P1), the next critical step is centralization to prevent data silos. Without centralized storage, each team member's data remains isolated and unusable for comparative analysis or team-wide evals.

**Independent Test**: Can be tested by configuring multiple Claude Code instances to export to a shared backend (ClickHouse or similar), then querying that backend to retrieve aggregated metrics across sessions, users, and time periods. Delivers value by enabling team-wide visibility into agent usage patterns.

**Acceptance Scenarios**:

1. **Given** 5 team members are working on different chapters, **When** they all enable telemetry with the shared backend endpoint, **Then** a project coordinator can query the centralized database and retrieve aggregated metrics (total prompts, total tokens, cost by user, cost by day) without accessing individual machines
2. **Given** centralized storage contains 1 week of multi-user data, **When** a coordinator runs a query for "chapters with highest token usage", **Then** the system returns accurate results grouped by chapter spec directory, showing which content areas require the most AI assistance
3. **Given** the centralized database is running, **When** new team members join and enable telemetry, **Then** their data automatically flows into the shared system without manual configuration beyond initial backend endpoint setup

---

### User Story 3 - Error Analysis Workflow (Priority: P3)

**As a** content quality reviewer  
**I want to** identify workflow traces where AI output quality was poor  
**So that** I can analyze failures, improve prompts, and refine evals following Andrew Ng's error analysis methodology

**Why this priority**: This builds on P1 (data collection) and P2 (centralized storage) to enable the actual evals-first improvement loop. This is where data transforms into actionable insights for quality improvement.

**Independent Test**: Can be tested by querying the centralized database for sessions with specific failure indicators (high retry counts, validation failures, low evaluation scores), then reviewing those workflow traces to identify patterns. Delivers value by systematically surfacing improvement opportunities.

**Acceptance Scenarios**:

1. **Given** the centralized database contains 2 weeks of chapter development data, **When** a reviewer queries for "sessions where validation failed more than 3 times", **Then** the system returns complete workflow traces (prompts, tool calls, validation results) for manual review
2. **Given** a reviewer identifies 10 traces where code examples failed technical review, **When** they analyze those traces to find common patterns (vague specs, missing constraints, unclear acceptance criteria), **Then** they can document improvement recommendations (better spec templates, additional eval criteria, clearer prompting guidelines)
3. **Given** error analysis reveals that 30% of failures relate to unclear specification language, **When** the team updates spec templates based on this insight, **Then** subsequent chapters show measurably lower validation failure rates (tracked via the same telemetry system)

---

### User Story 4 - Team Documentation & Onboarding (Priority: P4)

**As a** new team member  
**I want to** access clear setup documentation for enabling telemetry  
**So that** I can start contributing usage data without requiring extensive support from existing team members

**Why this priority**: This is a enabler for adoption at scale. Without clear documentation, centralized data collection becomes a support burden rather than a team capability.

**Independent Test**: Can be tested by giving a new team member the documentation and measuring time to successful telemetry setup (target: <15 minutes). Delivers value by reducing onboarding friction and ensuring consistent telemetry adoption.

**Acceptance Scenarios**:

1. **Given** a new team member has access to setup documentation, **When** they follow the step-by-step instructions, **Then** they successfully enable telemetry and verify data export to the centralized backend within 15 minutes without assistance
2. **Given** setup documentation includes troubleshooting guidance, **When** a team member encounters a common error (wrong endpoint URL, missing environment variable), **Then** they can self-diagnose and resolve the issue using the documented solutions
3. **Given** documentation includes example queries for common use cases (cost by chapter, top error patterns, token usage trends), **When** a team member wants to analyze their own usage, **Then** they can copy/paste queries and get meaningful results without learning the query language from scratch

---

### Edge Cases

- **What happens when** a team member works offline or the telemetry backend is unreachable?
  - System MUST buffer telemetry data locally and retry export when connectivity is restored, OR gracefully degrade to local-only logging without disrupting Claude Code functionality

- **What happens when** a single session generates extremely large volumes of data (e.g., 1000+ tool calls in a long workflow)?
  - System MUST handle high-volume sessions without performance degradation or data loss, potentially through batching or streaming export mechanisms

- **What happens when** sensitive information (API keys, personal data, proprietary code) appears in telemetry events?
  - System MUST provide configurable data sanitization/filtering to prevent sensitive data from being exported to centralized storage

- **What happens when** the centralized database reaches storage limits?
  - System MUST provide data retention policies (e.g., archive data older than 90 days) and automated cleanup processes

- **What happens when** multiple team members work on the same feature simultaneously?
  - System MUST ensure session IDs and user identifiers clearly distinguish concurrent work to prevent data attribution errors in analysis

## Requirements

### Functional Requirements

- **FR-001**: System MUST export Claude Code telemetry using OpenTelemetry standard (metrics and events)
- **FR-002**: System MUST capture user prompt events including prompt text, length, and session context
- **FR-003**: System MUST capture tool result events including tool name, execution time, success/failure status, and parameters used
- **FR-004**: System MUST capture API request events including token counts (input/output/cache), cost in USD, and request duration
- **FR-005**: System MUST capture API error events including error type, status code, and retry attempt number
- **FR-006**: System MUST include session metadata in all events: session ID, user ID, organization ID, application version, timestamp
- **FR-007**: System MUST support multiple telemetry export backends concurrently (console for debugging, OTLP for production, Prometheus for metrics)
- **FR-008**: System MUST provide environment variable configuration for telemetry enable/disable (`CLAUDE_CODE_ENABLE_TELEMETRY`)
- **FR-009**: System MUST provide environment variable configuration for OTLP endpoint (`OTEL_EXPORTER_OTLP_ENDPOINT`)
- **FR-010**: System MUST buffer telemetry data locally if backend is unreachable and retry export when connectivity is restored
- **FR-011**: System MUST provide data sanitization options to filter sensitive information from exported events (configurable patterns for API keys, credentials, PII)
- **FR-012**: System MUST enable querying centralized telemetry data by session ID, user ID, date range, feature/chapter, and event type
- **FR-013**: System MUST persist centralized telemetry data in a time-series compatible database (ClickHouse, Prometheus, or equivalent)
- **FR-014**: System MUST support aggregation queries for cost analysis (total cost by user, by feature, by day, by chapter)
- **FR-015**: System MUST support aggregation queries for token usage analysis (total tokens by type, by session, by user)
- **FR-016**: System MUST support filtering queries for error analysis (sessions with validation failures, high retry counts, specific error types)
- **FR-017**: System MUST provide example query templates for common analysis patterns (cost by chapter, error rate trends, token efficiency)
- **FR-018**: System MUST provide setup documentation covering configuration, verification, and troubleshooting
- **FR-019**: System MUST provide architecture documentation explaining data flow, backend options, and integration points
- **FR-020**: System MUST provide usage documentation with example queries and analysis workflows

### Key Entities

- **Telemetry Event**: Represents a single recorded action during a Claude Code session
  - Attributes: Event type (prompt/tool/API/error/decision), timestamp, session ID, user ID, event-specific data (prompt text, tool name, token count, cost, etc.)
  - Relationships: Belongs to a Session, belongs to a User

- **Session**: Represents a single Claude Code CLI session
  - Attributes: Session ID (unique identifier), user ID, organization ID, start time, end time, application version, terminal type
  - Relationships: Contains multiple Telemetry Events

- **User**: Represents a team member using Claude Code
  - Attributes: User UUID, organization ID, role (author/reviewer/coordinator)
  - Relationships: Creates multiple Sessions

- **Feature/Chapter Context**: Represents the feature spec or chapter being worked on
  - Attributes: Feature number, feature name, chapter number (if applicable), spec directory path
  - Relationships: Associated with Sessions (derived from git branch or working directory)

- **Error Analysis Record**: Represents a identified failure pattern from workflow trace review
  - Attributes: Pattern description, affected sessions (IDs), root cause category, improvement recommendation, status (identified/addressed/validated)
  - Relationships: References multiple Telemetry Events and Sessions

- **Aggregated Metric**: Represents computed metrics for analysis dashboards
  - Attributes: Metric type (cost/tokens/errors), aggregation period (day/week/month), aggregation dimension (user/feature/chapter), value, timestamp
  - Relationships: Computed from multiple Telemetry Events

## Success Criteria

### Measurable Outcomes

- **SC-001**: All team members can enable telemetry and verify data export to centralized backend within 15 minutes of reading setup documentation
- **SC-002**: System captures 100% of Claude Code interactions (prompts, tool calls, API requests) without data loss during normal operation
- **SC-003**: Centralized database supports queries retrieving aggregated metrics (cost by chapter, token usage trends) with response time under 3 seconds for 30-day data ranges
- **SC-004**: Error analysis workflow identifies actionable failure patterns from telemetry data, leading to measurable quality improvements (e.g., 20% reduction in validation failures after prompt template updates based on error analysis)
- **SC-005**: System handles high-volume sessions (1000+ events) without performance degradation or Claude Code disruption
- **SC-006**: Team coordinator can generate monthly cost reports showing total spend by user, by chapter, and by feature within 5 minutes
- **SC-007**: New team members successfully self-onboard to telemetry system (enable + verify) without requiring support from existing team members in 90% of cases
- **SC-008**: Data retention and cleanup processes maintain centralized database size within acceptable limits (configurable threshold, default 100GB) without manual intervention

## Assumptions

- Team members have network connectivity during most work sessions (for telemetry export)
- Team members are comfortable using environment variables for configuration
- Centralized backend infrastructure (OTLP collector + time-series database) is managed separately from this feature (this spec focuses on client-side telemetry enablement and usage patterns)
- Team members have basic SQL or query language familiarity for running analysis queries (example queries provided as templates)
- Claude Code OpenTelemetry integration is stable enough for production use (currently marked as beta)
- Sensitive data sanitization patterns can be defined upfront and configured via environment variables or config files

## Out of Scope

- Backend infrastructure provisioning and management (OTLP collector setup, database deployment, backup strategies)
- Real-time alerting or monitoring dashboards (this spec focuses on data collection and batch analysis workflows)
- Integration with external BI tools or data warehouses (future enhancement)
- Automated evaluation scoring or quality metrics calculation (this spec provides raw data; eval frameworks are separate)
- Multi-organization or multi-project data isolation (assumes single-organization deployment for this book project)

## Dependencies

- Claude Code OpenTelemetry integration (currently in beta, details subject to change)
- OTLP collector backend (OpenTelemetry Protocol endpoint, e.g., Jaeger, Grafana, custom)
- Time-series database for centralized storage (ClickHouse, Prometheus, or equivalent)
- Git workflow integration for chapter/feature context attribution (uses branch names or working directory to tag telemetry events)

## Notes

This specification aligns with Andrew Ng's recommendations on:

1. **Breaking down data silos** — Centralized telemetry prevents individual team members from hoarding usage data in isolated environments
2. **Error analysis for agentic performance** — Workflow trace collection enables systematic identification of failure patterns and improvement opportunities
3. **Evals-first development** — Telemetry data provides the ground truth for measuring whether changes (spec templates, prompt guidelines, validation criteria) actually improve outcomes

The telemetry system is designed to be:

- **Non-blocking**: Failures in telemetry export MUST NOT disrupt Claude Code functionality
- **Privacy-aware**: Sensitive data sanitization is a first-class requirement
- **Team-first**: Documentation and onboarding are treated as critical features, not afterthoughts
- **Analysis-ready**: Data schema and query examples are designed for Andrew Ng's error analysis workflow (informal trace review → rigorous pattern identification → workflow redesign)

This feature establishes the data foundation for future enhancements:

- Automated evaluation scoring based on telemetry patterns
- Predictive models for estimating chapter complexity (token usage, iteration count)
- Real-time cost tracking and budget alerts
- Integration with content evaluation framework (linking telemetry to chapter quality scores)
