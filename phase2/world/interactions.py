"""Unified interaction event log - the piece Phase I's audit (docs/experiment_inventory.md)
flagged as entirely missing: `claims.csv`/`negotiations.csv` exist in
Phase I but use different, non-unified schemas. Phase II logs every robot-to-robot event
through one JSONL schema so role/peer-preference/network analysis can read one file.

Interaction-type mapping, applied to what this world
actually does (no interaction type is emitted that doesn't correspond to a real event):

  RESOURCE_REQUEST / RESOURCE_YIELD  - task-station claim broadcast / losing or releasing one
  CHARGER_REQUEST / CHARGER_YIELD    - same, for the single shared charger specifically
                                        (kept distinct from RESOURCE_* so charger-specific
                                        convention analysis doesn't have
                                        to filter task-station noise out of charger events)
  HELP_REQUEST                       - a robot with a low battery mid-task offers its task
                                        to peers (OFFER performative)
  HELP_OFFER                         - a peer bids to take over that task (BID performative)
  TASK_TRANSFER                      - the offer resolved to a winner (COMMIT)
  TASK_REJECT                        - a bidder that didn't win (REJECT)

HELP_REQUEST/TASK_TRANSFER are recorded once per negotiation with `result` set to
"SUCCESS" or "TIMEOUT" (no bids) rather than as several redundant rows, matching how
Phase I's `negotiations.csv` already logs one row per performative rather than per
message hop.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import List, Optional, TextIO

RESOURCE_REQUEST = "RESOURCE_REQUEST"
RESOURCE_YIELD = "RESOURCE_YIELD"
CHARGER_REQUEST = "CHARGER_REQUEST"
CHARGER_YIELD = "CHARGER_YIELD"
HELP_REQUEST = "HELP_REQUEST"
HELP_OFFER = "HELP_OFFER"
TASK_TRANSFER = "TASK_TRANSFER"
TASK_REJECT = "TASK_REJECT"

ALL_INTERACTION_TYPES = frozenset({
    RESOURCE_REQUEST, RESOURCE_YIELD, CHARGER_REQUEST, CHARGER_YIELD,
    HELP_REQUEST, HELP_OFFER, TASK_TRANSFER, TASK_REJECT,
})


class InteractionLog:
    """Append-only JSONL writer. `events` (in-memory) is what tests and in-process
    analysis read directly; `path` (optional) is what a real run persists to disk -
    the same "log everything, analyze from saved data" discipline as Phase I's
    event_logger_node.py, just without needing a ROS subscription to do it."""

    def __init__(self, path: Optional[Path] = None) -> None:
        self.events: List[dict] = []
        self._file: Optional[TextIO] = None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = open(path, "w", encoding="utf-8")

    def record(
        self,
        *,
        simulation_time: float,
        from_robot: str,
        to_robot: Optional[str],
        interaction_type: str,
        resource: Optional[str] = None,
        task_id: Optional[str] = None,
        accepted: Optional[bool] = None,
        response_latency: Optional[float] = None,
        result: Optional[str] = None,
    ) -> None:
        if interaction_type not in ALL_INTERACTION_TYPES:
            raise ValueError(f"unknown interaction_type: {interaction_type!r}")
        event = {
            "simulation_time": round(simulation_time, 3),
            "from": from_robot,
            "to": to_robot,
            "interaction_type": interaction_type,
            "resource": resource,
            "task_id": task_id,
            "accepted": accepted,
            "response_latency": response_latency,
            "result": result,
        }
        self.events.append(event)
        if self._file is not None:
            self._file.write(json.dumps(event) + "\n")
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
