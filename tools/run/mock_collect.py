"""Backward-compatible entry point for one automatic Compose mock collection.

The supervisor lives in ``collect_run.py``; this module only keeps the historical
``bash tools/run/mock_collect.sh`` and ``python3 tools/run/mock_collect.py``
invocations working.  ``--output`` now names one raw run directory
(``<raw-root>/<run-id>``); run logs always stay under the separate runs root.
"""

from __future__ import annotations

import sys
from pathlib import Path

_TOOLS_RUN = Path(__file__).resolve().parent
if str(_TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(_TOOLS_RUN))

from collect_run import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
