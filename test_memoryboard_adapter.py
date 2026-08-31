"""Tests for the Memoryboard + RAG adapter in the Governed Optimus Swarm.

Uses a local HTTP stubs (http.server) so tests never depend on a running
Jarvis Memoryboard. Covers:
- fail-closed offline behavior (offline_ok=True degrades)
- hard failure when offline_ok=False
- remember() builds the correct governed EMR write payload
- recall()/rag_query() hit the right endpoints
- ingest_swarm_log() bounds the digest and refuses autonomous (non-user) writes
- attach()/detach() hot-swap the instrument; new URL mints a new session
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from memoryboard_adapter import MemoryboardAdapter, MemoryboardOffline, MemoryboardUndocked


class _StubHandler(BaseHTTPRequestHandler):
    """Records requests and serves canned responses per path."""

    requests: list = []

    def log_message(self, *args):
        pass

    def _handle(self):
        length = int(self.headers.get("Content-Length") or 0)
        body = self.rfile.read(length).decode() if length else ""
        record = {
            "method": self.command,
            "path": self.path,
            "headers": {k: v for k, v in self.headers.items()},
            "body": json.loads(body) if body else None,
        }
        type(self).requests.append(record)

        if self.path.startswith("/api/jarvis/tools/emr_remember"):
            payload = record["body"]
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({
                "accepted": payload.get("user_requested", False),
                "refused": not payload.get("user_requested", False),
                "refuse_reason": None if payload.get("user_requested") else "user-intent-required",
                "memory": ({"id": "mem-stub-1"} if payload.get("user_requested") else None),
            }).encode())
            return

        if self.path.startswith("/api/jarvis/tools/emr_recall"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({"memories": [], "conflicts": []}).encode())
            return

        if self.path.startswith("/api/jarvis/rag/query"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps({
                "status": "answered",
                "answer": "governed policy answer",
                "docs_used": ["d-1"],
            }).encode())
            return

        if self.path == "/health":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'{"schema": "continuity-ledger-v1"}')
            return

        self.send_response(404)
        self.end_headers()
        self.wfile.write(b'{"detail":"Not Found"}')

    do_GET = _handle
    do_POST = _handle
    do_PATCH = _handle
    do_DELETE = _handle


@pytest.fixture()
def stub_server():
    _StubHandler.requests = []
    server = HTTPServer(("127.0.0.1", 0), _StubHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield f"http://127.0.0.1:{server.server_address[1]}"
    server.shutdown()
    thread.join(timeout=2)


def test_adapter_health(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False)
    assert adapter.status() == {"schema": "continuity-ledger-v1"}


def test_remember_user_requested_true_accepted(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False, session_id="sx")
    resp = adapter.remember(
        "r1 locked by law violation",
        subject="governance",
        memory_type="decision",
        user_requested=True,
    )
    assert resp["accepted"] is True
    assert _StubHandler.requests[-1]["body"]["session_id"] == "sx"
    assert _StubHandler.requests[-1]["body"]["user_requested"] is True


def test_remember_autonomous_refused_fail_closed(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False)
    resp = adapter.remember("autonomous event", user_requested=False)
    assert resp["accepted"] is False
    assert resp["refuse_reason"] == "user-intent-required"


def test_recall_hits_emr_endpoint(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False)
    out = adapter.recall("where does r2 go")
    assert out == {"memories": [], "conflicts": []}
    assert _StubHandler.requests[-1]["path"].startswith("/api/jarvis/tools/emr_recall")


def test_rag_query_hits_rag_endpoint_and_passes_key(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False)
    out = adapter.rag_query("governed lane policy", rag_key="secret")
    assert out["status"] == "answered"
    req = _StubHandler.requests[-1]
    assert req["path"].startswith("/api/jarvis/rag/query")
    lowered = {k.lower(): v for k, v in req["headers"].items()}
    assert lowered.get("x-jarvis-rag-key") == "secret"


def test_offline_ok_degrades_gracefully():
    adapter = MemoryboardAdapter(base_url="http://127.0.0.1:1", offline_ok=True)  # port 1: refused
    out = adapter.remember("x", user_requested=True)
    assert out["_offline"] is True


def test_offline_fails_hard_when_not_offline_ok():
    adapter = MemoryboardAdapter(base_url="http://127.0.0.1:1", offline_ok=False)
    with pytest.raises(MemoryboardOffline):
        adapter.remember("x", user_requested=True)


def test_ingest_swarm_log_bounded_and_refuses_autonomous(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False)
    log = [{"robot": i, "event": "x"} for i in range(200)]
    resp = adapter.ingest_swarm_log(log, user_requested=False)
    assert len(resp) == 1
    assert resp[0]["refused"] is True
    # The digest sent must be bounded (last 64) not the full 200.
    body = _StubHandler.requests[-1]["body"]["content"]
    assert body.count("\n") <= 64


def test_detach_blocks_http_and_does_not_fabricate_recall(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=True, session_id="board-a")
    adapter.status()
    hits_before = len(_StubHandler.requests)

    rec = adapter.detach(reason="test_undock")
    assert rec["event"] == "undock"
    assert rec["continuity"] == "undocked"
    assert adapter.docked is False

    out = adapter.recall("anything")
    assert out["_undocked"] is True
    assert out["memories"] is None  # not [] — that would look like an empty live ledger
    assert out["conflicts"] is None
    assert len(_StubHandler.requests) == hits_before

    remember = adapter.remember("should not land", user_requested=True)
    assert remember["_undocked"] is True
    assert len(_StubHandler.requests) == hits_before


def test_redock_same_url_keeps_session(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False, session_id="board-a")
    adapter.detach(reason="brief_unplug")
    rec = adapter.attach(stub_server, reason="replug")
    assert rec["continuity"] == "same_instrument"
    assert rec["event"] == "dock"
    assert adapter.session_id == "board-a"
    assert adapter.docked is True
    assert adapter.recall("x") == {"memories": [], "conflicts": []}


def test_attach_new_url_mints_session_and_refuses_reuse(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=True, session_id="board-a")
    rec = adapter.attach("http://127.0.0.1:9", session_id="board-a", reason="swap_boards")
    assert rec["event"] == "swap"
    assert rec["continuity"] == "new_instrument"
    assert rec["rejected_session_reuse"] is True
    assert adapter.session_id != "board-a"
    assert adapter.session_id.startswith("governed-optimus-swarm-dock-")
    assert rec["from_session_id"] == "board-a"
    assert rec["to_session_id"] == adapter.session_id


def test_undocked_hard_fail_when_not_offline_ok(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=False)
    adapter.detach(reason="hard_undock")
    with pytest.raises(MemoryboardUndocked):
        adapter.remember("x", user_requested=True)


def test_dock_log_is_timestamped_and_complete(stub_server):
    adapter = MemoryboardAdapter(base_url=stub_server, offline_ok=True, session_id="board-a")
    adapter.detach(reason="r1")
    adapter.attach(stub_server, reason="r2")
    events = [e["event"] for e in adapter.dock_log]
    assert events[0] == "dock"  # constructed
    assert "undock" in events
    assert events[-1] == "dock"
    for e in adapter.dock_log:
        assert "wall_time_iso" in e
        assert e["reason"]
    state = adapter.instrument_state()
    assert state["docked"] is True
    assert state["events"] == len(adapter.dock_log)
