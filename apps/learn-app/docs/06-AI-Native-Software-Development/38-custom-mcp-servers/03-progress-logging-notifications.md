---
sidebar_position: 3
title: "Progress & Logging Notifications"
chapter: 38
lesson: 3
duration_minutes: 35

# HIDDEN SKILLS METADATA
skills:
  - name: "Using Context for Logging in MCP Servers"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can inject Context into tool handlers, call context.info()/warning()/error() methods, and verify logging output in Claude Desktop logs"

  - name: "Implementing Progress Notifications"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can call context.report_progress(current, total) with correct arguments and observe progress bar updates in client UI"

  - name: "Handling Notifications on the Client Side"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can write client-side callback handlers for logging_callback and print_progress_callback, test them with streaming operations"

  - name: "Designing UX for Long-Running Operations"
    proficiency_level: "B1"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Problem-Solving"
    measurable_at_this_level: "Student can identify long-running tools that need progress feedback, plan notification strategy, evaluate whether progress or logging is appropriate"

learning_objectives:
  - objective: "Understand why long-running operations require progress feedback and how frozen UX damages user experience"
    proficiency_level: "B1"
    bloom_level: "Understand"
    assessment_method: "Explanation of frozen client state and how notifications solve this"

  - objective: "Implement progress notifications using context.report_progress() with correct percentage calculation"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code implementation passing validation: correct argument types, percentage math, update frequency"

  - objective: "Implement logging notifications using context.info/warning/error with appropriate severity levels"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Code implementation with mixed severity levels, tested in Claude Desktop"

  - objective: "Write client-side callbacks to handle progress and logging notifications"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "Handler code receiving notification parameters, performing appropriate actions (display, log, etc.)"

  - objective: "Design notification strategy for various operation types (downloads, analysis, generation)"
    proficiency_level: "B1"
    bloom_level: "Analyze"
    assessment_method: "Decision framework for when to use progress vs logging, notification frequency optimization"

cognitive_load:
  new_concepts: 6
  assessment: "6 concepts (frozen UX problem, report_progress, info/warning/error, callback pattern, notification parameters, UX patterns) within B1 limit (7-10 concepts) ✓"

differentiation:
  extension_for_advanced: "Implement adaptive progress updates that adjust frequency based on operation duration; build client dashboard that aggregates notifications from multiple concurrent operations"
  remedial_for_struggling: "Focus on single notification type (progress OR logging) first; practice percentage calculation separately before integrating into tool"
---

# Progress & Logging Notifications

Imagine using Claude for a research task. You submit a tool call that will take 30 seconds to complete. The UI freezes. No feedback. No indication of progress. 30 seconds feels like forever.

This is the core problem that notifications solve: **When Claude calls your server tool and that tool takes time to run, the client needs continuous feedback that something is happening.**

Without notifications:
- Client UI appears frozen
- User thinks the tool crashed or hung
- User cancels the operation
- Claude restarts the tool from scratch

With notifications:
- Client UI shows progress bar or activity indicator
- User sees operational milestones ("Downloaded 50 of 100 files...")
- User has confidence that work is proceeding
- Long operations complete successfully

Notifications serve two purposes:

1. **Progress notifications** (quantified): "50% complete" — for operations where you know the total work units
2. **Logging notifications** (qualitative): "Processing manifest file..." — for operations where you report activity status

Let's implement both patterns.

## The Frozen UX Problem

Before we see notifications in action, understand why they matter. When Claude calls your server tool:

```python
@mcp.tool(name="research", description="Research a given topic")
async def research(topic: str):
    # This takes 45 seconds to complete
    sources = await fetch_sources(topic)  # 15 seconds
    articles = await fetch_articles(sources)  # 20 seconds
    summary = await generate_summary(articles)  # 10 seconds
    return summary
```

From the client perspective, the timeline looks like:

```
t=0s   Claude calls research("machine learning")
t=0s   Client waits for response (UI frozen, no feedback)
       ...
t=30s  Still waiting (user thinks tool crashed)
t=45s  Tool completes, returns response
t=45s  Client UI unfreezes, displays result
```

The entire interaction feels broken because the client has **zero visibility into progress**.

Notifications fix this by injecting context updates back to the client:

```
t=0s   Claude calls research("machine learning")
t=0s   Tool receives Context object
t=2s   Tool calls context.info("Fetching sources...")
       → Client receives logging notification
       → Client updates activity indicator
t=15s  Tool calls context.report_progress(15, 45)
       → Client receives progress notification (33%)
       → Client updates progress bar
t=20s  Tool calls context.info("Analyzing articles...")
       → Client receives logging notification
t=35s  Tool calls context.report_progress(35, 45)
       → Client updates progress bar (77%)
t=45s  Tool completes, returns response
       → Client receives final result
```

The user now has **continuous feedback that work is progressing**.

## Progress Notifications: Quantified Feedback

Progress notifications work when you know the total units of work upfront.

### Pattern: report_progress(current, total)

```python
@mcp.tool(
    name="batch_download",
    description="Download multiple files"
)
async def batch_download(
    urls: list[str] = Field(description="URLs to download"),
    *,
    context: Context
):
    total_files = len(urls)
    downloaded = []

    for i, url in enumerate(urls):
        file_content = await download_file(url)
        downloaded.append(file_content)

        # Update progress: current=i+1, total=total_files
        await context.report_progress(i + 1, total_files)

    return {"downloaded": len(downloaded), "files": downloaded}
```

**Output:**

Client sees:
- Progress bar: `████░░░░░░░░░░░░░░░░ 20%` (after 1 of 5 files)
- Progress bar: `██████████████████░░░░ 80%` (after 4 of 5 files)
- Progress bar: `████████████████████████ 100%` (complete)

### Key points about progress updates:

1. **Argument semantics**: `report_progress(current, total)` where:
   - `current`: How many units completed (0 to total)
   - `total`: Total units to complete
   - Client calculates percentage as: `(current / total) * 100`

2. **Update frequency matters**:
   - Too frequent (every 1% change): Floods client with notifications, degrades performance
   - Too infrequent (every 50%): Feels stuck between updates
   - Sweet spot: Update every 5-10% or every 1-2 seconds

3. **Progress semantics**: Report what's COMPLETED, not what remains:
   - ✓ Correct: `report_progress(3, 10)` → 30% done
   - ✗ Wrong: `report_progress(7, 10)` → Appears to be 70%, but means 30%

Let's see progress in context with other notifications:

```python
@mcp.tool(
    name="research_papers",
    description="Download and analyze research papers"
)
async def research_papers(
    keywords: str = Field(description="Research keywords"),
    *,
    context: Context
):
    await context.info("Searching academic database...")
    papers = await search_papers(keywords)

    total_papers = len(papers)
    analyzed = []

    for i, paper in enumerate(papers):
        await context.info(f"Analyzing {paper['title']}...")
        analysis = await analyze_paper(paper)
        analyzed.append(analysis)

        # After analyzing each paper, update progress
        await context.report_progress(i + 1, total_papers)

    summary = await generate_summary(analyzed)
    await context.info("Analysis complete")

    return {"count": total_papers, "summary": summary}
```

**Output:**

Client sees:
1. Logging: "Searching academic database..."
2. Logging: "Analyzing paper 1..."
3. Progress: `██░░░░░░░░░░░░░░░░░░░░░░ 10%`
4. Logging: "Analyzing paper 2..."
5. Progress: `████░░░░░░░░░░░░░░░░░░░░ 20%`
... (pattern continues) ...
10. Logging: "Analysis complete"
11. Progress: `████████████████████████ 100%`

## Logging Notifications: Qualitative Feedback

Logging notifications work when you want to report activity status without quantified progress. Three severity levels exist:

- `context.info()`: Regular activity milestones
- `context.warning()`: Recoverable issues (skipped files, missing data)
- `context.error()`: Errors that don't stop execution

### Pattern: Three Severity Levels

```python
@mcp.tool(
    name="process_documents",
    description="Process documents and extract metadata"
)
async def process_documents(
    directory: str = Field(description="Directory containing documents"),
    *,
    context: Context
):
    files = list_files(directory)
    results = []

    for filename in files:
        try:
            await context.info(f"Processing {filename}")
            content = read_file(f"{directory}/{filename}")

            if not is_valid_format(content):
                await context.warning(f"Skipping {filename}: Invalid format")
                continue

            metadata = extract_metadata(content)
            results.append(metadata)

        except Exception as e:
            await context.error(f"Error processing {filename}: {str(e)}")
            continue

    return {"processed": len(results), "results": results}
```

**Output:**

Client sees in activity log:
```
[info]    Processing document-1.pdf
[info]    Processing document-2.docx
[warning] Skipping document-3.txt: Invalid format
[info]    Processing document-4.pdf
[error]   Error processing document-5.xlsx: Missing sheet header
[info]    Processing document-6.pdf
```

### Severity level guidance:

| Level | When to use | Example |
|-------|------------|---------|
| `info()` | Normal operation milestones | "Downloading chunk 5 of 20", "Parsing config file" |
| `warning()` | Recoverable issues that don't prevent progress | "Skipping malformed entry", "Using default value for missing field" |
| `error()` | Errors that occurred but execution continues | "Failed to fetch metadata from API (continuing with cached data)" |

**Anti-pattern**: Using `error()` for expected conditions:
```python
# Wrong
await context.error("File not found in cache")  # Expected, use info() instead

# Right
await context.info("Cache miss for query_id=abc123")  # Normal operation detail
```

## Client-Side Callback Handling

The server sends notifications, but the client needs to handle them. When using Claude Desktop or Claude Code, these callbacks are automatically set up. However, if you're testing with a custom client (like MCP Inspector or a CLI tool), you'll need to implement handlers.

### Pattern: Notification Callbacks

When your tool sends notifications, the client receives them through two callback types:

```python
# Handler for logging notifications
async def logging_callback(params: LoggingMessageNotificationParams):
    # params.data contains the message
    # params.level contains the severity (info/warning/error)
    print(params.data)  # e.g., "Processing file 5"

# Handler for progress notifications
async def print_progress_callback(
    progress: float,
    total: float | None,
    message: str | None
):
    if total is not None:
        percentage = (progress / total) * 100
        print(f"Progress: {progress}/{total} ({percentage:.1f}%)")
    else:
        print(f"Progress: {progress}")
```

These callbacks are connected to your MCP client. In Claude Desktop, this happens automatically. When you test with custom tools, you register them:

```python
# For MCP Inbox (testing tool)
client = Client()

# Register callbacks
client.on_logging_message += logging_callback
client.on_progress_update += print_progress_callback

# Now when your tool calls context.info() or context.report_progress(),
# these callbacks receive the notifications
response = await client.call_tool("research", {"topic": "AI"})
```

### Real-world callback pattern: Tracking multiple concurrent operations

```python
class ProgressTracker:
    def __init__(self):
        self.operations = {}  # operation_id → progress_data

    async def handle_logging(self, params: LoggingMessageNotificationParams):
        timestamp = datetime.now().isoformat()
        operation_id = params.get("operation_id")  # Custom field in your notification

        self.operations[operation_id] = {
            "status": "active",
            "last_message": params.data,
            "timestamp": timestamp
        }

        # Update UI dashboard or log to file
        print(f"[{operation_id}] {params.data}")

    async def handle_progress(
        self, progress: float, total: float | None, message: str | None
    ):
        if total:
            percentage = (progress / total) * 100
            print(f"Progress: {percentage:.1f}% - {message}")

# Usage
tracker = ProgressTracker()
client.on_logging_message += tracker.handle_logging
client.on_progress_update += tracker.handle_progress
```

## UX Patterns for Notifications

Not every tool needs notifications. Match the strategy to the operation:

### Pattern 1: Deterministic Progress (known total work units)

**When**: Batch operations with known count
- Downloading 50 files
- Processing 100 rows
- Analyzing 20 articles

**Strategy**: Use `report_progress(current, total)`

```python
async def batch_operation(
    items: list,
    *,
    context: Context
):
    total = len(items)

    for i, item in enumerate(items):
        await process(item)
        await context.report_progress(i + 1, total)

    return "Complete"
```

### Pattern 2: Indeterminate Progress (unknown total work)

**When**: Operations where total work is unknown upfront
- Searching database with unpredictable result count
- Processing recursive structures with variable depth
- Streaming data with unknown end

**Strategy**: Use `context.info()` to report checkpoints instead of percentages

```python
async def indeterminate_operation(
    query: str,
    *,
    context: Context
):
    await context.info("Starting search...")

    checkpoint = 0
    async for result in search_stream(query):
        checkpoint += 1

        if checkpoint % 10 == 0:  # Report every 10 results
            await context.info(f"Found {checkpoint} results...")

    await context.info(f"Search complete: {checkpoint} total results")
    return {"count": checkpoint}
```

### Pattern 3: Multi-Phase Operations (distinct stages)

**When**: Operations with distinct phases
- Download → Parse → Analyze → Summarize
- Compile → Test → Deploy
- Fetch → Transform → Load

**Strategy**: Combine logging (phase transitions) with progress (within each phase)

```python
async def multi_phase_operation(
    dataset: str,
    *,
    context: Context
):
    # Phase 1: Download
    await context.info("Phase 1: Downloading dataset...")
    files = await download(dataset)
    await context.report_progress(25, 100)

    # Phase 2: Parse
    await context.info("Phase 2: Parsing files...")
    parsed = await parse(files)
    await context.report_progress(50, 100)

    # Phase 3: Analyze
    await context.info("Phase 3: Analyzing data...")
    analysis = await analyze(parsed)
    await context.report_progress(75, 100)

    # Phase 4: Summarize
    await context.info("Phase 4: Generating summary...")
    summary = await summarize(analysis)
    await context.report_progress(100, 100)

    return summary
```

### Pattern 4: Short Operations (< 2 seconds)

**When**: Operations that complete very quickly
- Simple API calls
- Small calculations
- Quick lookups

**Strategy**: Skip notifications entirely—overhead is higher than benefit

```python
async def quick_lookup(
    query: str,
    *,
    context: Context
):
    # Don't report progress for sub-second operations
    result = await fast_lookup(query)
    return result
```

## Try With AI

Use your AI companion (Claude, ChatGPT, Gemini, or similar).

### Prompt 1: Understand the Client UX

```
I'm building an MCP server tool that downloads and processes files.
Without notifications, what would the user experience look like from
Claude Desktop's perspective? Walk me through the timeline second-by-second
for a 45-second operation. What problems does this create?
```

**What you're learning**: Understanding the frozen UX problem that notifications solve—why user feedback matters for client-side experience.

### Prompt 2: Implement a Progress Tool

```
Here's my download tool stub:

async def download_files(
    urls: list[str],
    *,
    context: Context
):
    files = []
    for url in urls:
        content = await fetch(url)
        files.append(content)
    return files

Add progress notifications. I have 10 URLs to download. Show me:
1. Where to get the total count
2. When to call report_progress()
3. How to calculate the percentage argument correctly
4. What the client will see

Test your implementation step-by-step.
```

**What you're learning**: Implementing progress notifications with correct semantics—understanding total/current arguments and when to update.

### Prompt 3: Design Notification Strategy

```
I'm building an MCP server with these tools:

1. fetch_api(endpoint) - Returns data in 50ms
2. search_database(query) - Returns variable number of results (50-500), takes 2 seconds
3. process_csv(filepath) - Opens CSV, transforms rows, exports. Variable file size, 30 seconds typical
4. cache_update() - Updates internal cache from S3. Unknown total files, takes 10-60 seconds

For each tool, help me decide:
- Should it use progress notifications, logging, both, or neither?
- If progress: how do I know the total?
- If logging: what checkpoints should I report?
- What's the notification frequency that balances feedback with performance?

Build a decision framework for each tool.
```

**What you're learning**: Designing notification strategies appropriate to operation characteristics—matching notification type to operational semantics and operation duration.

---

**Safety Note for Long-Running Tools**: Long operations that timeout or fail should always report error-level logging before failing. This prevents frozen UX where client receives neither progress nor failure notification. Example: `await context.error("Timeout after 60s, cancelling operation")` before raising exception.
