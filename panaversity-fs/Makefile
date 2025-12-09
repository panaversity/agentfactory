# PanaversityFS Build Pipeline Makefile
# Provides targets for incremental builds, hydration, and ingestion

.PHONY: help hydrate ingest build test clean setup

# Default book ID (override with BOOK_ID=xxx)
BOOK_ID ?= ai-native-python

# Default directories (override as needed)
SOURCE_DIR ?= ./book-source/docs
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
	@echo "  hydrate       - Download content from PanaversityFS (incremental)"
	@echo "  hydrate-full  - Force full download (ignore manifest)"
	@echo "  ingest        - Upload source content to PanaversityFS (incremental)"
	@echo "  ingest-dry    - Preview what would be uploaded"
	@echo "  build         - Full build: ingest + hydrate"
	@echo "  test          - Run tests"
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
# Combined Build Pipeline
# ============================================================

build: ingest hydrate
	@echo "Build complete."

build-verbose:
	@echo "Full build (verbose)..."
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
	$(PYTHON) -m pytest tests/ -v

test-unit:
	@echo "Running unit tests..."
	$(PYTHON) -m pytest tests/unit/ -v

test-integration:
	@echo "Running integration tests..."
	$(PYTHON) -m pytest tests/integration/ -v

test-scripts:
	@echo "Running script tests..."
	$(PYTHON) -m pytest tests/scripts/ -v

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
	$(PYTHON) -m ruff check .

format:
	@echo "Formatting code..."
	$(PYTHON) -m ruff format .
