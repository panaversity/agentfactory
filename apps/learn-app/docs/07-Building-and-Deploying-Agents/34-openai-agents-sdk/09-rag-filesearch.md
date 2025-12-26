---
sidebar_position: 9
title: "RAG with FileSearchTool: Knowledge Base Integration"
description: "Build agentic RAG systems where agents decide when to retrieve from knowledge bases. Use OpenAI hosted vector stores with FileSearchTool for grounded, context-aware responses."
keywords: ["RAG", "FileSearchTool", "vector stores", "retrieval-augmented generation", "knowledge base", "vs_xxx IDs", "max_num_results", "agentic RAG", "grounded responses", "TaskManager"]
chapter: 34
lesson: 9
duration_minutes: 110

# HIDDEN SKILLS METADATA
skills:
  - name: "OpenAI Hosted Vector Store Creation"
    proficiency_level: "B2"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Cloud Integration"
    measurable_at_this_level: "Student can create vector stores via OpenAI dashboard and API, manage file uploads, and reference stores by vs_xxx IDs"

  - name: "FileSearchTool Configuration"
    proficiency_level: "B1"
    category: "Technical"
    bloom_level: "Apply"
    digcomp_area: "Software Development"
    measurable_at_this_level: "Student can configure FileSearchTool with vector_store_ids and max_num_results parameters"

  - name: "Agentic RAG Pattern Design"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Analyze"
    digcomp_area: "System Architecture"
    measurable_at_this_level: "Student can design workflows where agent autonomously decides when retrieval is necessary vs using existing knowledge"

  - name: "Grounded Response Generation"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Apply"
    digcomp_area: "Data Management"
    measurable_at_this_level: "Student can implement responses that cite retrieved documents and maintain traceability to source material"

  - name: "Knowledge Base Management for Digital FTEs"
    proficiency_level: "B2"
    category: "Applied"
    bloom_level: "Evaluate"
    digcomp_area: "Information Architecture"
    measurable_at_this_level: "Student can evaluate knowledge base design choices and tradeoffs for domain-specific agent products"

learning_objectives:
  - objective: "Create and manage OpenAI hosted vector stores for agent knowledge bases"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Student successfully creates vector store, uploads files, and references by vs_xxx ID"

  - objective: "Configure FileSearchTool to retrieve relevant documents with controlled result limits"
    proficiency_level: "B1"
    bloom_level: "Apply"
    assessment_method: "FileSearchTool configured with proper vector_store_ids and max_num_results working in agent"

  - objective: "Design agentic RAG where agent autonomously decides when to search knowledge base"
    proficiency_level: "B2"
    bloom_level: "Analyze"
    assessment_method: "Agent makes retrieval decisions based on context rather than retrieving for every prompt"

  - objective: "Implement grounded responses that cite source documents and maintain traceability"
    proficiency_level: "B2"
    bloom_level: "Apply"
    assessment_method: "Agent responses include citations and references to specific retrieved documents"

cognitive_load:
  new_concepts: 8
  assessment: "8 concepts (vector stores, FileSearchTool, vs_xxx IDs, max_num_results, agentic RAG, retrieval logic, grounding, citation) - within B2 limit of 10 ✓"

differentiation:
  extension_for_advanced: "Implement multi-vector-store orchestration; design retrieval confidence thresholds; analyze retrieval patterns to optimize knowledge base organization; build feedback loops to improve document relevance"
  remedial_for_struggling: "Start with pre-created vector store; focus on FileSearchTool configuration first; practice agentic retrieval decision-making with simple yes/no logic before complex confidence-based decisions"

generated_by: content-implementer
source_spec: specs/047-ch34-openai-agents-sdk
created: 2025-12-26
last_modified: 2025-12-26
version: 1.0.0
---

# RAG with FileSearchTool: Knowledge Base Integration

**Scenario**: Your TaskManager Digital FTE handles project work, but it only knows what you tell it in each conversation. A customer asks about your company's coding standards—the agent doesn't know them. Another asks about project governance—the agent hasn't seen those documents. You need the agent to access a knowledge base: documentation, standards, processes, templates—and ground its responses in those documents.

This is where Retrieval-Augmented Generation (RAG) becomes critical. Instead of relying on the model's training data (which is outdated) or asking you to copy-paste entire docs into every prompt, the agent searches a knowledge base and generates responses grounded in current information.

The key innovation in agentic RAG: **the agent decides when to search**. It doesn't retrieve for every prompt. It reasons about what it knows, recognizes gaps, and retrieves only when necessary. This reduces latency, tokens, and cost while improving accuracy.

## Understanding RAG and Vector Stores

Traditional applications search databases. RAG-enabled applications search **semantic meaning**.

### The RAG Pipeline

When a user asks your agent a question:

1. **User Query** → "What are our code review standards?"
2. **Semantic Search** → The system converts the query to embeddings and finds similar documents
3. **Context Window** → Relevant document chunks are inserted into the prompt
4. **Generation** → The model generates response grounded in retrieved documents
5. **Output** → User sees response + citations to source documents

### Vector Stores in OpenAI

OpenAI provides **hosted vector stores**—no infrastructure to manage. You upload documents, OpenAI handles embedding, chunking, and indexing.

**Key concepts**:

- **Vector Store ID** (format: `vs_xxx...`) → Reference your knowledge base
- **File IDs** → Individual documents in the vector store
- **Embeddings** → Mathematical representations of document meaning
- **Retrieval** → Semantic search finding documents relevant to query

Vector stores are like databases, but instead of searching for exact text matches, they search for semantic similarity. "What are our coding standards?" matches documents about "code review guidelines" and "development process requirements" even though the exact phrases don't appear.

## Creating Vector Stores

Vector stores can be created two ways: via dashboard or API.

### Method 1: OpenAI Dashboard (Recommended for First Setup)

1. Visit https://platform.openai.com
2. Navigate to **Files** section
3. Click **Create Vector Store**
4. Name it: "TaskManager Knowledge Base"
5. Upload your documents:
   - Click **Add Files**
   - Select documents (.pdf, .txt, .md, .docx)
   - Recommended: Start with 5-10 key documents
6. OpenAI automatically processes and embeds documents
7. Copy your **Vector Store ID** (format: `vs_abc123...`)

**Documentation you might upload**:
- Coding standards guide
- Project governance documents
- API documentation
- Team processes
- Decision frameworks
- Architecture diagrams (as PDFs with text)

### Method 2: API-Based Creation

```python
from openai import OpenAI

client = OpenAI()

# Create a vector store
vector_store = client.beta.vector_stores.create(name="TaskManager Knowledge Base")

print(f"Vector Store ID: {vector_store.id}")

# Upload files to the vector store
file_paths = [
    "docs/coding-standards.md",
    "docs/project-governance.md",
    "docs/api-reference.pdf"
]

# Read and upload each file
for file_path in file_paths:
    with open(file_path, "rb") as f:
        file_response = client.beta.vector_stores.files.upload(
            vector_store_id=vector_store.id,
            file=f
        )
        print(f"Uploaded {file_path}: {file_response.id}")
```

**Output:**
```
Vector Store ID: vs_5b6j7k8l9m0n1p2q3r4s
Uploaded docs/coding-standards.md: file-abc123...
Uploaded docs/project-governance.md: file-def456...
Uploaded docs/api-reference.pdf: file-ghi789...
```

This creates a vector store containing all three documents. OpenAI automatically:
- Extracts text from PDFs
- Chunks documents into retrievable pieces
- Generates embeddings for semantic search
- Indexes for fast retrieval

**Vector Store Lifecycle**:
- Created once, reused across agents
- Add/remove files as knowledge evolves
- Retention: Documents persist until explicitly deleted
- Scale: Handle thousands of documents efficiently

## Configuring FileSearchTool

Once you have a vector store, you configure **FileSearchTool** to let agents search it.

### Basic FileSearchTool Setup

```python
from agents import Agent, Runner, FileSearchTool

# Initialize the file search tool with your vector store ID
file_search = FileSearchTool(
    vector_store_ids=["vs_5b6j7k8l9m0n1p2q3r4s"],
    max_num_results=5  # Return top 5 most relevant documents
)

# Create an agent with the search tool
knowledge_agent = Agent(
    name="TaskManager Knowledge Expert",
    instructions="""You are an expert on company processes and standards.
    When users ask about coding standards, governance, or processes,
    search the knowledge base for current information.
    Always cite the document you retrieved from.""",
    tools=[file_search]
)

# Test the agent
result = Runner.run_sync(
    knowledge_agent,
    "What are our code review standards?"
)

print(result.final_output)
```

**Output:**
```
Based on our Code Review Standards document, our review process requires:

1. **All PRs reviewed by 2+ team members** - Ensures quality and knowledge sharing
2. **Automated checks pass first** - Linting, tests, security scans
3. **Comments addressed before merge** - All feedback incorporated or discussed
4. **Approval in 24 hours** - Targets fast iteration without rushing

The standards emphasize that code review is about improving code AND building team knowledge.
```

### FileSearchTool Parameters

| Parameter | Purpose | Example |
|-----------|---------|---------|
| **vector_store_ids** | References to knowledge bases | `["vs_abc123...", "vs_def456..."]` |
| **max_num_results** | Limit retrieved documents | `5` (return top 5) |

**Parameter Tuning**:

- **max_num_results**: Trade context window vs retrieval relevance
  - `3-5`: High precision, focused context
  - `5-10`: Balanced approach
  - `10+`: Comprehensive but may exceed context limits

For production TaskManager agents, start with `max_num_results=5` and observe whether agents have sufficient context.

### Multiple Vector Stores

For large organizations, you might have separate knowledge bases for different domains:

```python
file_search_multi = FileSearchTool(
    vector_store_ids=[
        "vs_coding_standards_abc123...",      # Coding practices
        "vs_project_governance_def456...",    # Governance
        "vs_api_reference_ghi789..."          # API docs
    ],
    max_num_results=3  # From each store
)

# Agent searches across all three stores
expert_agent = Agent(
    name="Expert",
    instructions="Search knowledge bases to answer questions",
    tools=[file_search_multi]
)
```

OpenAI performs semantic search across all stores and returns top results by relevance.

## The Agentic RAG Pattern

Here's what makes agentic RAG different from traditional RAG:

**Traditional RAG**: Search for every prompt
```
User: "Hello"
→ Vector search: [irrelevant results]
→ Generate: "Here's what I found about..." [wastes tokens]
```

**Agentic RAG**: Agent decides when to search
```
User: "Hello"
→ Agent thinks: "This is a greeting. I don't need to search."
→ Agent responds: "Hello! How can I help?"

User: "What are our code review standards?"
→ Agent thinks: "This requires domain knowledge. I should search."
→ Vector search: [relevant documents]
→ Agent generates grounded response with citations
```

The agent has the tool available but uses reasoning to decide when it's necessary.

### Implementing Agentic Retrieval Logic

```python
from agents import Agent, Runner, FileSearchTool

file_search = FileSearchTool(
    vector_store_ids=["vs_knowledge_base_id"],
    max_num_results=5
)

agentic_rag_agent = Agent(
    name="TaskManager RAG Expert",
    instructions="""You manage a project using company knowledge.

You have access to a knowledge base containing:
- Coding standards and best practices
- Project governance documents
- API reference materials
- Team processes

CRITICAL: Only search the knowledge base when necessary:
- YES, search if: "What are our standards?", "How do we handle X?", "Policy on Y?"
- NO, don't search if: "What's 2+2?", "How does Python work?", casual conversation

Searching wastes tokens. Think before you search.

When you retrieve documents, ALWAYS cite them:
- Reference the source document name
- Quote specific requirements
- Explain how the retrieved info answers the question
""",
    tools=[file_search]
)

# Test 1: Query requiring retrieval
result1 = Runner.run_sync(
    agentic_rag_agent,
    "What's our pull request approval process?"
)
print("Query 1 (needs retrieval):")
print(result1.final_output)

# Test 2: Query not requiring retrieval
result2 = Runner.run_sync(
    agentic_rag_agent,
    "Explain what a Git branch is"
)
print("\nQuery 2 (doesn't need retrieval):")
print(result2.final_output)
```

**Output:**
```
Query 1 (needs retrieval):
According to our Code Review Standards document:

**PR Approval Process**:
1. Create pull request with description
2. Run automated checks (linting, tests) - must pass
3. Request review from 2+ team members
4. Reviewers test and evaluate code
5. Address all feedback
6. Maintainer approves and merges

Typical timeline: 24 hours from submission to merge.

---

Query 2 (doesn't need retrieval):
A Git branch is an independent line of development. Think of it as a parallel universe for your code. You can work on feature branches without affecting main code. Once tested, you merge the branch back into main.

[No retrieval needed - this is general knowledge]
```

### Measuring Retrieval Effectiveness

In production, monitor whether the agent is making good retrieval decisions:

```python
from agents import Agent, Runner, RunHooks

class RetrievalAnalyzer(RunHooks):
    def __init__(self):
        self.retrievals = []

    async def on_tool_end(self, context, agent, tool, result):
        # Track when FileSearchTool was called
        if tool.name == "file_search":
            self.retrievals.append({
                "timestamp": datetime.now(),
                "query": str(tool.args),
                "result_count": len(result) if result else 0
            })

analyzer = RetrievalAnalyzer()
result = Runner.run_sync(agent, user_input, hooks=analyzer)

print(f"Total retrievals: {len(analyzer.retrievals)}")
for retrieval in analyzer.retrievals:
    print(f"  Query: {retrieval['query']}")
    print(f"  Results: {retrieval['result_count']}")
```

Agentic RAG is working well when:
- Retrieval-heavy queries trigger searches
- Casual/general queries don't retrieve
- Retrieved documents directly answer the question

## Practical Example: TaskManager with Knowledge Base

Let's build a TaskManager agent that uses company knowledge for better project guidance.

### Knowledge Base Documents

Your knowledge base contains:

**doc1: coding-standards.md**
```
# Coding Standards

## Python
- Use type hints on all functions
- Maximum line length: 100 characters
- Use Black formatter
- Docstrings required for public functions
```

**doc2: governance.md**
```
# Project Governance

## Decision Making
- Major decisions require consensus
- Changes affecting 3+ people need team discussion
- 24-hour feedback window minimum
```

**doc3: api-design.md**
```
# API Design Patterns

## REST Endpoints
- Use nouns for resources: /projects, /tasks, /users
- Use verbs for actions: POST to create, DELETE to remove
- All responses include metadata
```

### Building the Agent

```python
from agents import Agent, Runner, FileSearchTool

# Create knowledge-aware TaskManager
file_search = FileSearchTool(
    vector_store_ids=["vs_company_knowledge_abc123"],
    max_num_results=4
)

task_manager_agent = Agent(
    name="TaskManager Expert",
    instructions="""You manage a software project while following company standards.

When helping with code reviews, suggest improvements based on our standards.
When discussing governance, reference our decision-making framework.
When designing APIs, follow our design patterns.

Use the knowledge base to ground recommendations in our actual practices.
Always cite the relevant document when referencing standards.

Think carefully: only search when you need current standards or processes.
""",
    tools=[file_search]
)

# Example conversations
conversations = [
    "Review this Python function signature: def process_data(x, y, z): return x+y+z",
    "We need to decide on a new deployment strategy. How should we approach this?",
    "Design a REST API for managing project tasks"
]

for user_input in conversations:
    print(f"\nUser: {user_input}")
    result = Runner.run_sync(task_manager_agent, user_input)
    print(f"Agent: {result.final_output}")
```

**Output:**
```
User: Review this Python function signature: def process_data(x, y, z): return x+y+z
Agent: Based on our Coding Standards, this needs improvements:

1. **Add type hints** - Our standards require type hints on all functions:
   ```python
   def process_data(x: float, y: float, z: float) -> float:
   ```

2. **Add docstring** - Public functions need documentation:
   ```python
   def process_data(x: float, y: float, z: float) -> float:
       """Calculate sum of three numeric values."""
   ```

3. **Format with Black** - Ensure your code passes Black formatter

This function is currently non-compliant with our standards.

---

User: We need to decide on a new deployment strategy. How should we approach this?
Agent: According to our Project Governance document, major decisions follow this process:

1. **Present the proposal** - Document the deployment strategy options
2. **Team discussion** - Requires involvement from people affected (3+ people minimum discussion)
3. **Feedback window** - Allow 24 hours minimum for team feedback
4. **Consensus building** - Major decisions require consensus, not just majority vote
5. **Documentation** - Record the decision and rationale

Since deployment affects operations, development, and customer experience, this definitely requires team input. I'd recommend: write a short proposal outlining the new strategy, post it for feedback, and schedule a discussion.

---

User: Design a REST API for managing project tasks
Agent: Following our API Design Patterns document:

**Endpoints structure**:
- `GET /projects/{id}/tasks` - List tasks in project
- `POST /projects/{id}/tasks` - Create new task
- `GET /projects/{id}/tasks/{task_id}` - Get specific task
- `PUT /projects/{id}/tasks/{task_id}` - Update task
- `DELETE /projects/{id}/tasks/{task_id}` - Delete task

**Response format** (includes metadata):
```json
{
  "data": {
    "id": "task-123",
    "title": "Implement RAG",
    "status": "in_progress"
  },
  "metadata": {
    "timestamp": "2025-12-26T10:30:00Z",
    "version": "1.0"
  }
}
```

This design uses nouns for resources (/tasks, /projects) and maintains our metadata standard.
```

## Grounding Responses in Retrieved Content

The power of RAG is **traceability**: users know your answers come from actual company documents, not hallucination.

### Citation Pattern

```python
agent_instructions = """
When you retrieve documents, ALWAYS cite them in your response.

Format citations as:
- "According to [Document Name]: [quote]"
- "Our [Document Type] specifies: [requirement]"

Example good citation:
"According to our Code Review Standards document: 'All PRs require 2+ reviewers before merging. This ensures quality and knowledge sharing.'"

Example bad citation (vague):
"We probably review code. I think it's important."
"""
```

### Confidence Thresholds

For sensitive topics (security, compliance), implement retrieval confidence:

```python
file_search_strict = FileSearchTool(
    vector_store_ids=["vs_security_policies"],
    max_num_results=3  # Return only most relevant documents
)

security_agent = Agent(
    name="Security Advisor",
    instructions="""For security and compliance questions, only provide answers
    based on retrieved documents. If retrieval returns insufficient results,
    explicitly state 'This requires consultation with our security team.'

    Never guess about security policies. Always cite the document.""",
    tools=[file_search_strict]
)
```

## Try With AI

### Setup

You'll build a knowledge-aware TaskManager with retrieval capability.

**Prerequisites**:
- OpenAI account with API access
- Created a vector store (via dashboard or API) with 3-5 sample documents
- Vector Store ID available (format: `vs_xxx...`)

**Required documents to upload** (create these as .txt or .md files):
1. Coding standards (5-10 lines)
2. Project governance (5-10 lines)
3. API design patterns (5-10 lines)

### Prompt 1: Create Your Vector Store

**What you're learning**: How to bootstrap a knowledge base with documents.

Ask AI:
```
I need to create a Python script that:
1. Creates a new vector store named "TaskManager Knowledge"
2. Uploads three .txt files to it:
   - "coding-standards.txt" with sample coding guidelines
   - "governance.txt" with sample governance rules
   - "api-design.txt" with sample API patterns
3. Prints the vector store ID

Use the OpenAI Python client. Show me the complete script with sample document content.
```

Review the response:

- Does the script use `client.beta.vector_stores.create()`?
- Are files uploaded with `client.beta.vector_stores.files.upload()`?
- Does the output include the vector store ID you'll need next?

### Prompt 2: Configure FileSearchTool in Your Agent

**What you're learning**: How to connect an agent to a knowledge base.

Based on the vector store ID from Prompt 1, ask AI:
```
I have a vector store with ID: vs_xxx... (from previous step)

Create an agent that:
1. Has FileSearchTool configured with my vector store
2. Sets max_num_results=5
3. Has instructions to search for company knowledge when needed
4. Does NOT search for general knowledge questions

Show me:
- The FileSearchTool configuration
- The Agent creation
- A test with 2 queries: one that should search, one that shouldn't
```

Review:

- Does FileSearchTool have correct vector_store_ids?
- Does the agent distinguish between queries needing retrieval vs general knowledge?
- Which query triggered a retrieval search?

### Prompt 3: Implement Agentic Decision-Making

**What you're learning**: How to make retrieval decisions intelligently.

Ask AI:
```
Modify the agent from the previous step so it:
1. Explains its reasoning about whether to search
2. Only searches when the question specifically asks about company policies/standards
3. For general questions, responds without searching
4. Always cites the document source when it retrieves

Test with these 3 queries:
- "What are our coding standards?"
- "Explain what REST APIs are"
- "How should we make project decisions?"

Show me the agent output for each, indicating which ones triggered retrieval.
```

Review:

- Did the agent correctly identify which queries needed retrieval?
- Did responses include citations to document sources?
- Did the agent avoid unnecessary searches for general knowledge?

### Prompt 4: Build Citation Analysis

**What you're learning**: How to ensure responses are grounded in documents.

Ask AI:
```
Create a citation tracker that:
1. Runs your agent with 5 different queries
2. After each response, checks if the response includes a citation
3. Counts how many responses were properly cited
4. For uncited responses, identifies if they should have been

Show me a report listing:
- Query
- Whether retrieval was triggered
- Whether response included citation
- Assessment (correct/incorrect) of citation presence
```

Review:

- How many responses should have been cited?
- How many actually were cited?
- Which topics most need citations (standards, policies)?

### Reflection: Knowledge as Product

Ask yourself:

1. **Completeness**: Are your three documents sufficient for real agent needs? What's missing?
2. **Retrieval Quality**: Did the agent retrieve relevant documents for all queries?
3. **Citation Trust**: Would a user trust recommendations backed by cited documents?
4. **Digital FTE Product**: If you were selling this TaskManager as a Digital FTE, what documents would customers need to upload?

This is foundation of knowledge-enabled Digital FTE products—every response is grounded in customer-provided documentation.

**Expected outcome**: A working agent that searches knowledge bases intelligently, retrieves relevant documents, and grounds responses in current company information.

