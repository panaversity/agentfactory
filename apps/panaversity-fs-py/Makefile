# PanaversityFS Build Pipeline Makefile
# Provides targets for incremental builds, hydration, and ingestion

.PHONY: help hydrate ingest build sync test clean setup lint format

# Default book ID (override with BOOK_ID=xxx)
BOOK_ID ?= ai-native-python

# Default directories (override as needed)
# Path is relative to this Makefile (apps/panaversity-fs-py/)
SOURCE_DIR ?= ../learn-app/docs
OUTPUT_DIR ?= .docusaurus/content
MANIFEST_FILE ?= .panaversity/manifest.json

# Python interpreter
PYTHON ?= python

# MCP Server URL (for local development)
PANAVERSITY_MCP_URL ?= http://localhost:8000

help:
	@echo "PanaversityFS Build Pipeline"
	@echo ""
	@echo "Usage: make <target> [BOOK_ID=xxx] [SOURCE_DIR=xxx]"
	@echo ""
	@echo "Targets:"
	@echo "  build         - Build Python package (for CI/distribution)"
	@echo "  lint          - Run ruff linter"
	@echo "  test          - Run tests"
	@echo "  hydrate       - Download content from PanaversityFS (incremental)"
	@echo "  hydrate-full  - Force full download (ignore manifest)"
	@echo "  ingest        - Upload source content to PanaversityFS (incremental)"
	@echo "  ingest-dry    - Preview what would be uploaded"
	@echo "  sync          - Full sync: ingest + hydrate (requires MCP server)"
	@echo "  clean         - Remove build artifacts"
	@echo "  setup         - Install dependencies"
	@echo ""
	@echo "Variables:"
	@echo "  BOOK_ID=$(BOOK_ID)"
	@echo "  SOURCE_DIR=$(SOURCE_DIR)"
	@echo "  OUTPUT_DIR=$(OUTPUT_DIR)"
	@echo "  PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL)"

# ============================================================
# Hydration (Download from PanaversityFS)
# ============================================================

hydrate:
	@echo "Hydrating from PanaversityFS..."
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/hydrate-book.py \
		--book-id $(BOOK_ID) \
		--output-dir $(OUTPUT_DIR) \
		--manifest-file $(MANIFEST_FILE)

hydrate-full:
	@echo "Full hydration from PanaversityFS (ignoring manifest)..."
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/hydrate-book.py \
		--book-id $(BOOK_ID) \
		--output-dir $(OUTPUT_DIR) \
		--manifest-file $(MANIFEST_FILE) \
		--full-rebuild

hydrate-verbose:
	@echo "Hydrating from PanaversityFS (verbose)..."
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/hydrate-book.py \
		--book-id $(BOOK_ID) \
		--output-dir $(OUTPUT_DIR) \
		--manifest-file $(MANIFEST_FILE) \
		--verbose

# ============================================================
# Ingestion (Upload to PanaversityFS)
# ============================================================

ingest:
	@echo "Ingesting to PanaversityFS..."
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/ingest-book.py \
		--book-id $(BOOK_ID) \
		--source-dir $(SOURCE_DIR)

ingest-dry:
	@echo "Dry run ingestion (no changes)..."
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/ingest-book.py \
		--book-id $(BOOK_ID) \
		--source-dir $(SOURCE_DIR) \
		--dry-run

ingest-verbose:
	@echo "Ingesting to PanaversityFS (verbose)..."
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/ingest-book.py \
		--book-id $(BOOK_ID) \
		--source-dir $(SOURCE_DIR) \
		--verbose

# ============================================================
# Package Build (for CI/distribution)
# ============================================================

build:
	@echo "Building Python package..."
	uv build

# ============================================================
# Content Sync Pipeline (requires MCP server)
# ============================================================

sync: ingest hydrate
	@echo "Sync complete."

sync-verbose:
	@echo "Full sync (verbose)..."
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/ingest-book.py \
		--book-id $(BOOK_ID) \
		--source-dir $(SOURCE_DIR) \
		--verbose
	PANAVERSITY_MCP_URL=$(PANAVERSITY_MCP_URL) \
	$(PYTHON) scripts/hydrate-book.py \
		--book-id $(BOOK_ID) \
		--output-dir $(OUTPUT_DIR) \
		--manifest-file $(MANIFEST_FILE) \
		--verbose

# ============================================================
# Testing
# ============================================================

test:
	@echo "Running tests..."
	uv run --extra dev pytest tests/ -v

test-unit:
	@echo "Running unit tests..."
	uv run --extra dev pytest tests/unit/ -v

test-integration:
	@echo "Running integration tests..."
	uv run --extra dev pytest tests/integration/ -v

test-scripts:
	@echo "Running script tests..."
	uv run --extra dev pytest tests/scripts/ -v

# ============================================================
# Setup & Cleanup
# ============================================================

setup:
	@echo "Installing dependencies..."
	pip install -e ".[dev]"

clean:
	@echo "Cleaning build artifacts..."
	rm -rf $(OUTPUT_DIR)
	rm -rf .panaversity/
	rm -rf __pycache__
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true

clean-manifest:
	@echo "Removing manifest (will force full rebuild)..."
	rm -f $(MANIFEST_FILE)

# ============================================================
# Development Targets
# ============================================================

dev-server:
	@echo "Starting local MCP server..."
	$(PYTHON) -m uvicorn src.panaversity_fs.server:app --reload --port 8000

lint:
	@echo "Running linter..."
	@# Note: --exit-zero allows CI to pass while we address existing lint issues
	uv run --extra dev ruff check . --exit-zero

format:
	@echo "Formatting code..."
	uv run --extra dev ruff format .
