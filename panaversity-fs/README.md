# PanaversityFS - Agent-Native Multi-Book Storage System

**Status**: Phase 4 - MCP Server Implementation (In Progress)

PanaversityFS is a Python MCP server providing unified CRUD operations for educational content across multiple storage backends using OpenDAL abstraction.

## Architecture

- **Runtime**: Python 3.13+ with FastMCP framework
- **MCP Transport**: Stateless Streamable HTTP
- **Storage**: OpenDAL (fs/s3/supabase)
- **Authentication**: API key for MVP
- **Audit**: Direct JSONL writes

## MCP Tools (14 Total)

**Content** (3 tools): read_content, write_content, delete_content ✅
**Assets** (3 tools): upload_asset, get_asset, list_assets ✅
**Summaries** (4 tools): add/update/get/list_summaries ✅
**Bulk** (1 tool): get_book_archive ⏳
**Registry** (1 tool): list_books ⏳
**Search** (2 tools): glob/grep_search ⏳

**Progress**: 10/14 tools implemented (71%)

## Quick Start

```bash
# Install dependencies
cd panaversity-fs && uv sync

# Configure (local filesystem)
export PANAVERSITY_STORAGE_BACKEND=fs
export PANAVERSITY_STORAGE_ROOT=/tmp/panaversity-fs-data

# Start server
python -m panaversity_fs.server
# Server runs at http://0.0.0.0:8000/mcp

# Test with MCP Inspector
npx @modelcontextprotocol/inspector http://localhost:8000/mcp
```

## Implementation Status

- ✅ Phase 0: Learning & architecture
- ✅ Phase 1: Core infrastructure (869 lines)
- ✅ Phase 2: Content tools (3/14 tools)
- ✅ Phase 3: Asset tools (6/14 tools)
- ✅ Phase 4: Summary tools (10/14 tools)

## Links

- **Draft PR**: https://github.com/panaversity/ai-native-software-development/pull/299
- **Specification**: [specs/030-panaversity-fs/spec.md](../specs/030-panaversity-fs/spec.md)
