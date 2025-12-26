# Chapter 35: Google ADK TaskManager

Starter project for building a reliable TaskManager agent.

## Setup

```bash
cd examples/chapter-35-google-adk
uv sync
export GOOGLE_API_KEY="your-key"
```

## Structure

Students generate these files using prompts from the lessons:

```
taskmanager-agent/
├── pyproject.toml      # ✅ Provided
├── evals/              # ✅ Provided (starter cases)
├── agent.py            # Generate in Lesson 2
├── tools.py            # Generate in Lesson 2
├── callbacks.py        # Generate in Lesson 5
├── session.py          # Generate in Lesson 6
└── tests/              # Generate in Lesson 7
```

## Run

```bash
# Run agent
adk run agent.py

# Run evals
adk eval evals/taskmanager_evals.json

# Deploy
adk deploy agent_engine
```

See lessons for "Generate with AI" prompts to create each component.
