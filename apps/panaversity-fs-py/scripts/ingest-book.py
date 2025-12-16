#!/usr/bin/env python
"""Entry point for ingest-book CLI.

Ingests book source content to PanaversityFS storage.
Handles path mapping, change detection, and incremental uploads.

Usage:
    python scripts/ingest-book.py --book-id ai-native-python --source-dir ./book-source/docs
    python scripts/ingest-book.py --book-id ai-native-python --source-dir ./book-source/docs --dry-run
    python scripts/ingest-book.py --help
"""

import sys
from pathlib import Path

# Add the app root directory to path for imports
# This handles both local runs and CI environments
app_root = Path(__file__).parent.parent.resolve()
if str(app_root) not in sys.path:
    sys.path.insert(0, str(app_root))

from scripts.ingest.cli import main

if __name__ == "__main__":
    main()
