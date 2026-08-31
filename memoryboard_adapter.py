"""Memoryboard + RAG adapter for the Governed Optimus Swarm.

Integrates the swarm with the Jarvis Memoryboard (EMR persistence ledger) and
the AMUL RAG service, so a governed swarm can:

- **Remember** significant events (law violations, task assignment/completion,
  lock events) as *governed draft* memories through the EMR write gateway.
- **Recall** governed memory (EMR retrieval) into swarm decisions/context.
- **Query** the RAG knowledge base for governed policy/knowledge lookups.

Constitutional posture (mirrors the Memoryboard lawbook):
- Writes are DRAFT-ONLY and require explicit `user_requested`/operator opt-in;
  the swarm never auto-verifies truth in the ledger.
- Reads never mutate the ledger.
- Fail-closed: any network/config error surfaces as a structured
  ``MemoryboardError`` (or a soft ``offline_ok`` mode) rather than crashing the
  swarm or silently fabricating recall.
- Hot-swappable: ``attach(url)`` / ``detach()`` dock or undock the instrument
  at runtime. The swarm keeps ticking. Two boards are never merged: a new URL
  mints a new ``session_id`` so continuity does not silently jump ledgers.
"""

from __future__ import annotations

import json
import logging
import os
import time
import uuid
from datetime import datetime, timezone
from typing import Any, Optional
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

log = logging.getLogger("memoryboard_adapter")
log.addHandler(logging.NullHandler())

# Default base URL of the Jarvis Memoryboard service.
DEFAULT_MEMORYBOARD_URL = os.getenv("JARVIS_MEMORYBOARD_URL", "http://127.0.0.1:8001")


def _norm_url(url: str) -> str:
    """Identity of an instrument is its normalized base URL, not its session."""
    return (url or "").rstrip("/").lower()


class MemoryboardError(RuntimeError):
    """Raised on a hard (non-offline) memoryboard failure."""


class MemoryboardOffline(RuntimeError):
    """Raised when the memoryboard is unreachable and offline_ok=False."""


class MemoryboardUndocked(MemoryboardOffline):
    """Raised when the operator has detached the instrument and offline_ok=False."""


class MemoryboardAdapter:
    """Thin HTTP client over the Memoryboard/EMR/RAG API."""

    def __init__(
        self,
        base_url: str | None = None,
        *,
        session_id: str = "optimus-swarm",
        source_agent: str = "governed-optimus-swarm",
        offline_ok: bool = True,
        timeout: float = 5.0,
    ) -> None:
        self.base_url = (base_url or os.getenv("JARVIS_MEMORYBOARD_URL") or DEFAULT_MEMORYBOARD_URL).rstrip("/")
        self.session_id = session_id
        self.source_agent = source_agent
        self.offline_ok = offline_ok
        self.timeout = timeout
        self.docked = True
        self._instrument_url = self.base_url
        self.dock_log: list[dict[str, Any]] = []
        self._stamp(
            "dock",
            reason="constructed",
            from_url=None,
            to_url=self.base_url,
            from_session_id=None,
            to_session_id=self.session_id,
            continuity="same_instrument",
            note="adapter constructed already docked; swarm may detach without restart",
        )

    # ------------------------------------------------------------------ #
    # Low-level HTTP                                                     #
    # ------------------------------------------------------------------ #

    def _request(
        self,
        method: str,
        path: str,
        *,
        body: Optional[dict[str, Any]] = None,
        headers: Optional[dict[str, str]] = None,
    ) -> dict[str, Any]:
        if not self.docked:
            return self._undocked_response()
        url = f"{self.base_url}{path}"
        data = json.dumps(body).encode() if body is not None else None
        req = Request(
            url,
            data=data,
            method=method,
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
                **(headers or {}),
            },
        )
        try:
            with urlopen(req, timeout=self.timeout) as resp:
                payload = resp.read().decode("utf-8")
                return json.loads(payload) if payload else {}
        except HTTPError as exc:
            detail = exc.read().decode("utf-8", "ignore")
            if self.offline_ok:
                return {"_offline": True, "_http_status": exc.code, "_detail": detail}
            raise MemoryboardError(f"memoryboard HTTP {exc.code}: {detail}") from exc
        except (URLError, TimeoutError, OSError) as exc:
            if self.offline_ok:
                return {"_offline": True, "_error": str(exc)}
            raise MemoryboardOffline(f"memoryboard unreachable: {exc}") from exc

    # ------------------------------------------------------------------ #
    # Health                                                             #
    # ------------------------------------------------------------------ #

    def status(self) -> dict[str, Any]:
        """Probe /health. Returns server schema or an offline/undocked marker."""
        return self._request("GET", "/health")

    # ------------------------------------------------------------------ #
    # Hot-swap: dock / undock the instrument without restarting the swarm #
    # ------------------------------------------------------------------ #

    def instrument_state(self) -> dict[str, Any]:
        """Snapshot of which board is docked. Safe to log; no HTTP."""
        return {
            "docked": self.docked,
            "base_url": self.base_url,
            "session_id": self.session_id,
            "instrument_url": self._instrument_url,
            "events": len(self.dock_log),
        }

    def detach(self, *, reason: str = "operator_detach") -> dict[str, Any]:
        """Undock the Memoryboard. The swarm keeps ticking; recall will not fabricate.

        Subsequent remember/recall/rag hit no network. They return an undocked
        marker (or raise MemoryboardUndocked if offline_ok=False).
        """
        from_url = self.base_url
        from_session = self.session_id
        if not self.docked:
            return self._stamp(
                "undock",
                reason=reason,
                from_url=from_url,
                to_url=None,
                from_session_id=from_session,
                to_session_id=from_session,
                continuity="already_undocked",
                note="already undocked; no HTTP",
            )
        self.docked = False
        log.info("memoryboard undocked url=%s session=%s reason=%s", from_url, from_session, reason)
        return self._stamp(
            "undock",
            reason=reason,
            from_url=from_url,
            to_url=None,
            from_session_id=from_session,
            to_session_id=from_session,
            continuity="undocked",
            note="instrument undocked; swarm continues; recall will not fabricate",
        )

    def attach(
        self,
        base_url: str,
        *,
        session_id: Optional[str] = None,
        reason: str = "operator_attach",
    ) -> dict[str, Any]:
        """Dock a Memoryboard at runtime.

        Same URL as the last instrument: session continuity is preserved
        (re-seat of the same board). A different URL is a new instrument:
        a new session_id is minted so two ledgers are not merged. Passing
        the previous session_id for a new URL is rejected, not reused.
        """
        new_url = (base_url or "").strip().rstrip("/")
        if not new_url:
            raise MemoryboardError("attach requires a non-empty base_url")

        from_url = self.base_url
        from_session = self.session_id
        from_docked = self.docked
        same_instrument = _norm_url(new_url) == _norm_url(self._instrument_url)

        rejected_session_reuse = False
        if same_instrument:
            to_session = session_id or from_session
            continuity = "same_instrument"
        else:
            continuity = "new_instrument"
            if session_id is None or session_id == from_session:
                rejected_session_reuse = session_id == from_session
                to_session = self._mint_session()
            else:
                to_session = session_id

        self.base_url = new_url
        self.session_id = to_session
        self.docked = True
        self._instrument_url = new_url

        health = self.status()
        event = "swap" if from_docked and not same_instrument else "dock"
        note = (
            "new instrument: session rotated so two ledgers are not merged"
            if continuity == "new_instrument"
            else "same instrument re-docked; session continuity preserved"
        )
        log.info(
            "memoryboard %s to=%s session=%s continuity=%s reason=%s",
            event, new_url, to_session, continuity, reason,
        )
        return self._stamp(
            event,
            reason=reason,
            from_url=from_url,
            to_url=new_url,
            from_session_id=from_session,
            to_session_id=to_session,
            from_docked=from_docked,
            continuity=continuity,
            rejected_session_reuse=rejected_session_reuse,
            health=health,
            note=note,
        )

    def _mint_session(self) -> str:
        return f"{self.source_agent}-dock-{uuid.uuid4().hex[:12]}"

    def _undocked_response(self) -> dict[str, Any]:
        # memories/conflicts are None (not []) so undocked recall cannot be
        # mistaken for "live board, empty ledger".
        marker = {
            "_offline": True,
            "_undocked": True,
            "_detail": "memoryboard undocked; no ledger I/O until attach()",
            "memories": None,
            "conflicts": None,
        }
        if self.offline_ok:
            return marker
        raise MemoryboardUndocked("memoryboard undocked")

    def _stamp(self, event: str, **fields: Any) -> dict[str, Any]:
        rec = {
            "event": event,
            "wall_time": time.time(),
            "wall_time_iso": datetime.now(timezone.utc).isoformat(),
            "docked": self.docked,
            "base_url": self.base_url,
            "session_id": self.session_id,
            **fields,
        }
        self.dock_log.append(rec)
        return rec

    # ------------------------------------------------------------------ #
    # Write: govern a swarm event into durable (draft) memory            #
    # ------------------------------------------------------------------ #

    def remember(
        self,
        content: str,
        *,
        subject: Optional[str] = None,
        memory_type: str = "decision",
        tags: Optional[list[str]] = None,
        confidence: float = 0.5,
        user_requested: bool = False,
        user_statement: Optional[str] = None,
    ) -> dict[str, Any]:
        """Persist a swarm event as a governed DRAFT memory via EMR.

        ``user_requested`` must be True for the write gateway to accept the
        record (constitutional user-intent gate). The swarm's autonomous
        writes default to False and are therefore rejected (fail-closed).
        """
        return self._request(
            "POST",
            "/api/jarvis/tools/emr_remember",
            body={
                "content": content,
                "source_agent": self.source_agent,
                "session_id": self.session_id,
                "type": memory_type,
                "subject": subject,
                "tags": tags or [],
                "confidence": confidence,
                "user_requested": user_requested,
                "user_statement": user_statement,
            },
        )

    # ------------------------------------------------------------------ #
    # Recall: EMR governed retrieval                                     #
    # ------------------------------------------------------------------ #

    def recall(self, query: str, *, limit: int = 8) -> dict[str, Any]:
        """EMR governed recall: retrieve relevant memories + conflicts."""
        return self._request(
            "POST",
            "/api/jarvis/tools/emr_recall",
            body={"intent": "code", "query": query, "limit": limit},
        )

    def retrieve(self, query: str, *, truth_scope: str = "live") -> dict[str, Any]:
        """Raw ledger retrieval (no EMR scoring) — memories + conflicts."""
        qs = urlencode({"query": query, "truth_scope": truth_scope})
        return self._request("GET", f"/api/jarvis/memory/retrieve?{qs}")

    # ------------------------------------------------------------------ #
    # RAG: knowledge lookup                                              #
    # ------------------------------------------------------------------ #

    def rag_query(self, query: str, *, rag_key: Optional[str] = None) -> dict[str, Any]:
        """Query the AMUL RAG knowledge base for governed facts."""
        headers = {"X-Jarvis-RAG-Key": rag_key} if rag_key else {}
        return self._request(
            "POST",
            "/api/jarvis/rag/query",
            body={"query": query},
            headers=headers,
        )

    def rag_status(self) -> dict[str, Any]:
        return self._request("GET", "/api/jarvis/rag/status")

    # ------------------------------------------------------------------ #
    # Convenience: swarm log -> memory digest                            #
    # ------------------------------------------------------------------ #

    def ingest_swarm_log(
        self,
        log: list[dict[str, Any]],
        *,
        user_requested: bool = False,
        memory_type: str = "decision",
        subject: str = "optimus-swarm-step",
    ) -> list[dict[str, Any]]:
        """Optionally persist a compressed swarm log digest as governed memory.

        Returns the per-write memoryboard responses (accept or refuse).
        Autonomous (user_requested=False) writes are refused by the gateway —
        this is intended fail-closed behavior.
        """
        if not log:
            return []
        digest = "\n".join(
            json.dumps(e, sort_keys=True)
            for e in log[-64:]  # bounded digest; never dump the whole transcript
        )
        resp = self.remember(
            digest,
            subject=subject,
            memory_type=memory_type,
            user_requested=user_requested,
            user_statement=(
                "Optim uses the Optimus Swarm ingestion of a bounded per-tick "
                "log digest, pending operator verification."
            ) if user_requested else None,
        )
        return [resp]
