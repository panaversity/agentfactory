### Core Concept
Hybrid workloads combine asyncio (concurrent I/O) with InterpreterPoolExecutor (parallel CPU), enabling pipelines where fetch and process overlap—turning 1,000 items in 35 minutes (sequential) into 5 minutes (pipelined) through intelligent batching and resource coordination.

### Key Mental Models
- **Pipeline Overlap**: Fetch item N while processing item N-1 and storing item N-2; stages run concurrently through queues
- **Batch Processing**: Fetch in groups (e.g., 10 items) to manage memory and allow safe resource constraints
- **Bottleneck Identification**: Longest-running stage limits throughput; optimize by increasing workers for that stage
- **Semaphore as Limiter**: Control concurrency without runaway resource exhaustion (API rate limits, database connections)

### Critical Patterns
- **Queue-Based Coordination**: asyncio.Queue buffers between stages, implements backpressure naturally
- **Batch Loop Pattern**: for batch in batches: fetch, process, store—simple but sequential
- **3-Stage Pipeline**: Fetch → Transform → Store running concurrently via queues and executors
- **Resource Limiting**: Semaphore(N) ensures max N concurrent operations, preventing system overload

### AI Collaboration Keys
- Ask AI to design batch size for 1,000 items given network/CPU/storage latencies
- Have AI identify bottleneck by analyzing per-stage timing metrics
- Collaborate on queue depths and backpressure to prevent memory exhaustion

### Common Mistakes
- Sequential batching (fetch all, then process all, then store) instead of overlapping stages
- Unbounded queues causing memory exhaustion when one stage is much slower
- Not measuring bottleneck before optimizing (premature optimization of wrong stage)

### Connections
- **Builds on**: Asyncio concurrency, CPU parallelism with executors, task coordination
- **Leads to**: Production AI systems, data pipelines, streaming architectures
