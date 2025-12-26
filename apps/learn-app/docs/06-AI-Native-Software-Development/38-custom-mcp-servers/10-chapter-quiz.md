---
sidebar_position: 10
title: "Chapter Quiz: Advanced MCP Server Development"
description: "Comprehensive assessment covering Context objects, sampling, notifications, roots, transport, scaling, error handling, and packaging patterns from Chapter 38"
keywords: ["MCP Assessment", "Context Objects", "Sampling", "Progress Notifications", "Roots", "StreamableHTTP", "Error Handling", "Packaging"]
chapter: 38
lesson: 10
duration_minutes: 45

# HIDDEN ASSESSMENT METADATA
assessment_type: "Chapter Mastery Quiz"
total_questions: 20
passing_score: 75
question_distribution:
  - category: "Context Object"
    count: 3
    bloom_levels: ["Understand", "Apply", "Apply"]
  - category: "Sampling"
    count: 4
    bloom_levels: ["Understand", "Apply", "Apply", "Analyze"]
  - category: "Notifications"
    count: 3
    bloom_levels: ["Understand", "Apply", "Analyze"]
  - category: "Roots"
    count: 3
    bloom_levels: ["Understand", "Apply", "Analyze"]
  - category: "Transport & Scaling"
    count: 4
    bloom_levels: ["Understand", "Apply", "Apply", "Analyze"]
  - category: "Error Handling & Packaging"
    count: 3
    bloom_levels: ["Understand", "Apply", "Apply"]

learning_objectives_assessed:
  - "Understand Context dependency injection patterns and when to use Context injection vs parameter passing"
  - "Implement sampling workflows where servers call LLMs through connected clients"
  - "Design progress notifications and logging strategies for long-running operations"
  - "Configure file system permissions using roots and is_path_allowed() validation"
  - "Select appropriate transport and scaling configurations for production deployments"
  - "Handle errors gracefully and package servers for distribution"

cognitive_load:
  assessment: "Cumulative assessment across 20 questions with mixed difficulty—students must synthesize knowledge from all 8 lessons"

success_criteria:
  - "75%+ correct (15 questions) indicates mastery of core patterns"
  - "90%+ correct (18 questions) indicates advanced production readiness"
---

# Chapter Quiz: Advanced MCP Server Development

This quiz assesses your mastery of advanced MCP patterns from Chapter 38. You should be able to answer these questions confidently after completing Lessons 1–8.

**Time**: ~45 minutes
**Passing Score**: 15/20 (75%)
**Advanced Score**: 18/20 (90%)

---

## Context Object Category

### Question 1: Dependency Injection Pattern

**What is the primary advantage of Context being injected into tool functions rather than passed as a parameter?**

A) It reduces the number of function parameters, making function signatures cleaner
B) FastMCP can automatically wire dependencies without requiring manual setup boilerplate
C) It allows multiple Context objects to exist simultaneously in different threads
D) It ensures Context is never shared between concurrent tool invocations

**Correct Answer**: B

**Explanation**:
Dependency injection enables the framework to automatically provide Context without explicit parameter passing. This is why FastMCP can wire complex tools without requiring setup boilerplate. While A is a nice side effect, it's not the primary advantage. C is false (Context is per-request, not per-thread). D describes a property of Context, but not why it's injected rather than passed.

---

### Question 2: Context Logging Levels

**You're building a tool that processes customer data. You want to log when processing starts (informational), when a non-critical issue occurs (e.g., missing optional field), and when data validation fails (problematic but recoverable). Which logging methods should you use?**

A) `info()` for all three cases
B) `info()` for start, `warning()` for missing field, `error()` for validation failure
C) `warning()` for all three cases
D) `error()` for all three cases

**Correct Answer**: B

**Explanation**:
Logging levels match severity: `info()` for expected operational events, `warning()` for non-critical issues that clients should be aware of, `error()` for failures that prevent normal operation. Using all one level loses diagnostic value—clients won't know which issues are serious.

---

### Question 3: Session-Based Context Usage

**When would you use `context.session.session_id` to cache results across multiple tool invocations within the same client connection?**

A) Every time you need to store any data persistently
B) When a user has executed multiple tools in sequence and you want to avoid recomputing shared data
C) When building stateless servers that must not retain any client-specific state
D) When implementing database connection pooling

**Correct Answer**: B

**Explanation**:
Session-based caching is appropriate when a single client connection includes multiple tool calls that could benefit from shared context (same user, same conversation). Stateless servers explicitly avoid this pattern. It's not for persistent storage or database pooling—those are different architectural concerns.

---

## Sampling Category

### Question 4: Problem Sampling Solves

**What fundamental problem does sampling (servers calling LLMs through clients) solve that servers couldn't do before?**

A) It allows servers to perform computation without using local CPU
B) It enables servers to access frontier LLM reasoning while staying focused on their domain expertise
C) It guarantees cheaper token costs because servers don't pay for LLM calls
D) It removes the need for error handling in servers

**Correct Answer**: B

**Explanation**:
Sampling shifts reasoning to frontier models (which clients have access to) while servers focus on domain operations. This is the conceptual insight: servers are good at tools/resources, frontier models are good at reasoning. Sampling bridges this gap. A is wrong (servers still compute). C is wrong (cost depends on client pricing). D is obviously false.

---

### Question 5: Implementing Sampling

**You want your server to ask an LLM for advice on which data processing strategy to use. Which method would you use to call the LLM through the connected client?**

A) `context.llm.generate(prompt)` — Call local LLM
B) `context.session.create_message([...])` — Call client's LLM
C) `await context.sample(model="claude-3-5-sonnet")` — Direct sampling
D) `context.request_reasoning(query)` — Special reasoning mode

**Correct Answer**: B

**Explanation**:
`context.session.create_message()` is the correct method for sampling. The client connection has access to its LLM; the server calls through that connection. A and C are made up. D doesn't exist. Sampling always goes through the session to the client's connected model.

---

### Question 6: Sampling Cost Structure

**Your server uses sampling to ask an LLM for advice during tool execution. Who bears the token cost of this LLM call?**

A) The server owner (you)
B) The client who called the server
C) Shared equally between server and client
D) Nobody—sampling is always free

**Correct Answer**: B

**Explanation**:
Cost shifting is the key insight: the client's LLM account pays for sampling. This is a feature, not a bug—it incentivizes servers to use sampling responsibly (reasoning doesn't happen on server's dime). This makes reasoning-intensive servers economically sustainable.

---

### Question 7: Client-Side Sampling Callback

**When a server invokes `context.session.create_message()` to perform sampling, what must happen on the client side?**

A) The client automatically routes the request to its configured LLM
B) The client must have a special `on_sampling_request()` handler registered
C) The client opens a new connection to request LLM access
D) Sampling cannot work—it's server-only

**Correct Answer**: A

**Explanation**:
MCP clients (like Claude Code) are designed to handle sampling requests transparently. When a server calls `create_message()`, the client automatically routes the request to its configured model. No special registration is needed. The architecture assumes clients will handle sampling; servers just call it.

---

## Notifications Category

### Question 8: Progress Notifications

**You're processing a file with 10,000 records. You want to report progress every time you process 100 records. Which method would you call?**

A) `await context.report_progress(100, 10000)` after each batch
B) `await context.progress_percentage(1)` to increment by 1%
C) `await context.log_progress("100 records processed")`
D) `context.session.notify_progress(100, 10000)`

**Correct Answer**: A

**Explanation**:
`report_progress(completed, total)` is the correct signature. You call it repeatedly with updated values as progress changes. B and C are made up. D is wrong (it's a context method, not session). Progress notifications keep clients informed during long operations.

---

### Question 9: Logging Levels in Notifications

**Your tool reports informational messages during processing, warnings when data quality is questionable, and errors when processing fails. How should clients distinguish between these severity levels?**

A) All notifications appear as plain text—clients can't distinguish severity
B) Use `info()`, `warning()`, and `error()` methods; clients interpret logging level in UI
C) Send all messages with the same method; severity is determined by message text
D) Use separate notification methods but don't expect client-side behavior to differ

**Correct Answer**: B

**Explanation**:
Logging levels communicate severity. Clients (like Claude Code) display these differently—errors might be highlighted, warnings might use different colors, info might be less prominent. This is intentional design. A and D are wrong because they miss the point of structured logging.

---

### Question 10: Client-Side Notification Handling

**When a server sends logging and progress notifications, what happens on the client if the client doesn't implement handlers for these notifications?**

A) The client crashes because it received unexpected messages
B) Notifications are silently ignored; processing continues
C) The client returns an error to the server
D) Notifications are buffered and displayed when processing completes

**Correct Answer**: B

**Explanation**:
MCP design assumes optional notification support. If a client doesn't implement handlers, notifications are safely ignored—the server continues normally. This is graceful degradation. A, C, and D would break compatibility with older clients.

---

## Roots Category

### Question 11: What Roots Solve

**Why is the roots feature necessary for MCP servers that access files?**

A) To improve file read/write performance
B) To define secure boundaries on which directories servers can access
C) To replace traditional file permissions in operating systems
D) To enable file caching across multiple server instances

**Correct Answer**: B

**Explanation**:
Roots establish a security boundary. Without roots, a tool could access `/etc/passwd` or other sensitive files outside its scope. Roots tell the client "this server can only see files under `/data/projects`" (for example). This is a security mechanism. A is wrong (roots don't improve performance). C is wrong (roots complement, not replace, OS permissions). D is wrong (roots don't handle caching).

---

### Question 12: Implementing Path Validation

**You're writing a tool that downloads files to a directory controlled by the user. Before allowing the download, how should you validate the target path?**

A) Trust the client-provided path—the client verified it already
B) Check if the path exists using `os.path.exists()`
C) Use `is_path_allowed()` to verify the path is within allowed roots
D) Validate against a hardcoded list of allowed directories

**Correct Answer**: C

**Explanation**:
`is_path_allowed()` checks against declared roots, preventing path traversal attacks. A is wrong (don't trust client input). B doesn't validate against roots. D is inflexible and doesn't use the roots mechanism. Proper validation combines roots declaration with `is_path_allowed()` enforcement.

---

### Question 13: Roots Security Implications

**A user declares a root as `/home/user/projects`. Your tool constructs a path using user input: `/home/user/projects/../../../etc/passwd`. What happens if you call `is_path_allowed()` on this path?**

A) It returns True because the path contains `/home/user/projects` as a substring
B) It returns False because the path resolves outside the declared root
C) It returns None (no answer provided)
D) It crashes the server because path traversal is detected

**Correct Answer**: B

**Explanation**:
`is_path_allowed()` canonicalizes paths (resolves `..` sequences) before checking against roots. The path traversal is detected and validation fails. A is wrong (substring matching is not how it works). C and D are wrong (proper validation returns False, doesn't crash).

---

## Transport & Scaling Category

### Question 14: StreamableHTTP vs Stdio

**When should you choose StreamableHTTP transport instead of stdio?**

A) When your server runs on the same machine as the client
B) When your server needs to handle multiple concurrent connections from different clients
C) When you're developing and testing locally
D) When your server is simple and processes requests sequentially

**Correct Answer**: B

**Explanation**:
StreamableHTTP handles concurrent clients (one connection per client). Stdio is designed for single-client scenarios (like Claude Code). If your server needs to serve multiple clients simultaneously, StreamableHTTP is required. A is wrong (same machine can use either). C and D suggest stdio is better, which is backwards.

---

### Question 15: Stateless vs Stateful Configuration

**Your MCP server processes documents and stores intermediate results in memory. You want to scale it horizontally behind a load balancer. What configuration choice must you make?**

A) Use `stateless_http=True` to ensure each request is independent
B) Use `stateless_http=False` to preserve state across requests
C) Use `json_response=True` to serialize state between instances
D) Add session affinity to the load balancer so requests route to the same server instance

**Correct Answer**: A and D (but if forced to choose one, A is the correct configuration choice; D is a load balancer setting that would be needed if you chose stateful)

**Explanation**:
For horizontal scaling, you should design stateless servers (`stateless_http=True`) where each request is fully independent. This allows load balancers to route requests to any instance. If you must maintain state, use session affinity so requests from the same client stick to one server. But stateless is the preferred pattern. B is wrong (mutable state breaks load balancing). C is wrong (it doesn't solve the problem).

---

### Question 16: json_response Implications

**You set `json_response=True` on your StreamableHTTP server. What does this affect?**

A) How the server stores data internally (all data becomes JSON)
B) Whether responses are wrapped in JSON envelope or sent as raw bytes
C) The programming language the server can be written in (must support JSON)
D) Whether clients must be written in JavaScript

**Correct Answer**: B

**Explanation**:
`json_response` determines the response format. When True, responses are JSON-wrapped (structured metadata + payload). When False, raw bytes are sent. This is a transport encoding choice. A is wrong (internal representation is unchanged). C and D are nonsense.

---

### Question 17: Scaling Configuration Decision

**You're deploying a server that processes real-time sensor data. Clients expect to subscribe to updates from previous requests. What configuration maximizes both scalability and update delivery?**

A) Stateless with no session support
B) Stateful with session affinity to one server instance
C) Stateless with WebSocket subscriptions
D) Stateful with database to share state between instances

**Correct Answer**: D (with caveats) or B as practical choice

**Explanation**:
This is a reasoning question without a perfect answer. Real-time subscriptions require some statefulness. Option B (stateful with affinity) is practical but limits scaling. Option D (shared database) enables true horizontal scaling but adds complexity. A is wrong (can't do subscriptions). C (WebSockets) isn't part of standard MCP transport.

---

## Error Handling & Packaging Category

### Question 18: JSON-RPC Error Structure

**Your tool encounters an error: "Database connection failed." How should you return this to the client in MCP?**

A) Raise a Python exception and let FastMCP convert it to JSON-RPC error
B) Return an error code as a string: `"error: database_connection_failed"`
C) Use structured JSON-RPC error with code and message
D) Log the error and return an empty response

**Correct Answer**: A (Python exception) or C (if using direct JSON-RPC)

**Explanation**:
FastMCP automatically converts Python exceptions to JSON-RPC error format. Alternatively, you can use the structured format directly. B is wrong (raw strings aren't structured errors). D is wrong (don't silently fail). The framework handles error translation.

---

### Question 19: Graceful Error Recovery

**Your server uses sampling to ask for LLM advice. The LLM call fails with a timeout. How should your server behave?**

A) Crash and return a critical error to the client
B) Retry the LLM call up to 3 times with exponential backoff
C) Proceed without LLM advice if it's non-essential to the tool, or return error if it's critical
D) Always retry forever until the LLM responds

**Correct Answer**: C

**Explanation**:
Graceful degradation depends on whether LLM input is essential. If the tool can proceed without it (e.g., advice on optimization), retry or log and continue. If it's essential (e.g., validation), return an error. B might work but don't blindly retry forever. D is wrong (infinite retries cause hangs).

---

### Question 20: Packaging for Distribution

**You've built a production MCP server. How should you define the entry point so it can be installed and discovered by clients?**

A) Add a `__main__.py` file and users manually run your server
B) Document the command to run in a README (e.g., `python -m my_server`)
C) Define the entry point in `pyproject.toml` under `[project.scripts]`
D) Create an executable shell script that users can run

**Correct Answer**: C

**Explanation**:
`pyproject.toml` entry points enable standard Python package discovery. Clients can install your server with `pip install my-server` and automatically have the entry point available. A and B are manual and don't integrate with the Python ecosystem. D might work locally but isn't standard packaging. C is the production-ready approach.

---

## Quiz Completion

**Congratulations on completing the Chapter 38 Quiz!**

### Next Steps

- **Passing (15+/20)**: You've demonstrated mastery of advanced MCP patterns. Move forward to building your own production server.
- **Advanced (18+/20)**: You're ready for the capstone (Lesson 9) and real-world deployment scenarios.
- **Review Needed**: Review the lesson(s) corresponding to lower-scoring categories and retry this quiz.

### Answer Key Summary

| Question | Category | Correct | Difficulty |
|----------|----------|---------|------------|
| 1 | Context | B | Apply |
| 2 | Context | B | Apply |
| 3 | Context | B | Analyze |
| 4 | Sampling | B | Understand |
| 5 | Sampling | B | Apply |
| 6 | Sampling | B | Apply |
| 7 | Sampling | A | Analyze |
| 8 | Notifications | A | Understand |
| 9 | Notifications | B | Apply |
| 10 | Notifications | B | Analyze |
| 11 | Roots | B | Understand |
| 12 | Roots | C | Apply |
| 13 | Roots | B | Analyze |
| 14 | Transport & Scaling | B | Understand |
| 15 | Transport & Scaling | A | Apply |
| 16 | Transport & Scaling | B | Apply |
| 17 | Transport & Scaling | D | Analyze |
| 18 | Error Handling & Packaging | A | Understand |
| 19 | Error Handling & Packaging | C | Apply |
| 20 | Error Handling & Packaging | C | Apply |

---

## Try With AI: Quiz Review & Clarification

Use this section to deepen your understanding of challenging concepts with AI assistance.

**Setup**: Open Claude Code or your preferred MCP client

### Part 1: Identify Weak Areas

Review your quiz answers. Note which categories or specific questions you found challenging:

```
Questions I struggled with: [Q#, Q#, ...]
Topics I want to clarify: [topic, topic, ...]
```

### Part 2: Deep Dive on Sampling

**Prompt to Claude Code**:

```
I'm struggling with sampling in MCP servers. Explain:
1) Why sampling shifts costs to clients (why is this good?)
2) What code pattern I'd use to call an LLM from within an MCP tool
3) A real scenario where sampling is essential
```

**What you're learning**:
- The economic incentive structure of sampling
- Implementation pattern for `context.session.create_message()`
- When sampling adds value vs adds complexity

### Part 3: Scaling Tradeoffs

**Prompt to Claude Code**:

```
I want to build a server that handles multiple clients and maintains some state
(like caches). Should I use stateless_http=True or False? What are the tradeoffs?

Walk me through an example where state must be preserved vs where it can be stateless.
```

**What you're learning**:
- Tradeoff between scalability and feature requirements
- When session affinity is necessary
- How database-backed state enables true scaling

### Part 4: Roots Security

**Prompt to Claude Code**:

```
Show me a vulnerable example where a tool doesn't properly validate file paths
using is_path_allowed(). Then show the fixed version. Explain how path traversal
attacks work and why roots + canonicalization prevents them.
```

**What you're learning**:
- Common path traversal vulnerabilities
- How is_path_allowed() protects against them
- Why you can't trust client-provided paths

### Part 5: Self-Assessment

Ask yourself:
- Can I explain sampling to someone who hasn't taken this chapter?
- Could I design a server that uses roots to control file access?
- Would I know which transport (stdio vs StreamableHTTP) to choose for a given scenario?
- Can I recognize when error handling is insufficient and add graceful recovery?

**If you answered "yes" to all four**, you've mastered this chapter's core concepts.

---

## Additional Resources

- **Chapter 38 Lessons 1–9**: Review specific lessons for deeper practice
- **MCP Official Docs**: Context patterns, sampling examples, roots configuration
- **Capstone Project (Lesson 9)**: Build a full production server that demonstrates all patterns

**Final Note**: Advanced MCP server development isn't just about knowing patterns—it's about making tradeoff decisions. You've learned WHAT each pattern does. Now practice deciding WHEN and WHY to apply each one in production scenarios.
