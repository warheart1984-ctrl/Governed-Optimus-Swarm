"""Governed Runtime Control Plane (prototype).

This is independent verification for the OmniSim seam and the swarm law
gate. It is not PKI, not a certified product, and not a production
zero-trust platform. Digests are HMAC-SHA256 over canonical JSON plus a
run-scoped secret/nonce from RunManifest.

Responsibilities:
  * Bind an Assignment into a validated AdmissionRecord before any
    OmniSim HTTP or motion command.
  * Freeze robot roles in an immutable RunManifest; role changes go
    through rebind_role() with a hashed ticket.
  * Mint one-use operation_ids and resolve dispatch signatures *before*
    the first call (no TypeError retry).
"""

from __future__ import annotations

import hashlib
import hmac
import inspect
import json
import secrets
import uuid
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from types import MappingProxyType
from typing import Any, Dict, Mapping, Optional, Tuple


DEFAULT_ROLE = "carrier"


def canonical_json(obj: Any) -> bytes:
    """Stable JSON for HMAC/SHA256. Tuples become lists; keys sorted."""

    def _default(value: Any) -> Any:
        if isinstance(value, tuple):
            return list(value)
        raise TypeError(f"not JSON-serializable: {type(value).__name__}")

    return json.dumps(
        obj,
        sort_keys=True,
        separators=(",", ":"),
        default=_default,
    ).encode("utf-8")


def hmac_hex(secret: str, payload: Any) -> str:
    return hmac.new(
        secret.encode("utf-8"),
        canonical_json(payload),
        hashlib.sha256,
    ).hexdigest()


def sha256_hex(payload: Any) -> str:
    return hashlib.sha256(canonical_json(payload)).hexdigest()


@dataclass(frozen=True)
class RunManifest:
    """Immutable per-run authority: roles, verification secret, identity."""

    run_id: str
    verification_secret: str
    roles: Mapping[str, str]
    issued_at: str
    generation: int = 0

    @classmethod
    def issue(
        cls,
        roles: Mapping[str, str],
        verification_secret: Optional[str] = None,
        run_id: Optional[str] = None,
    ) -> "RunManifest":
        frozen_roles = MappingProxyType(dict(roles))
        return cls(
            run_id=run_id or str(uuid.uuid4()),
            verification_secret=verification_secret or secrets.token_hex(32),
            roles=frozen_roles,
            issued_at=datetime.now(timezone.utc).isoformat(),
            generation=0,
        )

    def role_of(self, robot_id: str) -> Optional[str]:
        return self.roles.get(robot_id)

    def with_role(self, robot_id: str, new_role: str) -> "RunManifest":
        updated = dict(self.roles)
        updated[str(robot_id)] = new_role
        return RunManifest(
            run_id=self.run_id,
            verification_secret=self.verification_secret,
            roles=MappingProxyType(updated),
            issued_at=self.issued_at,
            generation=self.generation + 1,
        )


def roles_from_robots(
    robots: Mapping[str, Any],
    explicit: Optional[Mapping[str, str]] = None,
) -> Dict[str, str]:
    if explicit is not None:
        return {str(k): str(v) for k, v in explicit.items()}
    out: Dict[str, str] = {}
    for robot_id, robot in robots.items():
        out[str(robot_id)] = str(getattr(robot, "role", None) or DEFAULT_ROLE)
    return out


@dataclass(frozen=True)
class AdmissionRecord:
    """Bound, validated admission. Raw swarm dicts are not authority."""

    robot_id: str
    task_id: str
    target: str
    policy: str
    request_id: str
    source_log: Dict[str, Any]
    state_hash: str
    role: str
    admission_id: str
    issued_at: str
    digest: str

    def payload_for_digest(self) -> Dict[str, Any]:
        return {
            "admission_id": self.admission_id,
            "issued_at": self.issued_at,
            "policy": self.policy,
            "request_id": self.request_id,
            "robot_id": self.robot_id,
            "role": self.role,
            "source_log": self.source_log,
            "state_hash": self.state_hash,
            "target": self.target,
            "task_id": self.task_id,
        }

    def compute_digest(self, secret: str) -> str:
        return hmac_hex(secret, self.payload_for_digest())

    def verify(self, secret: str) -> bool:
        if not self.digest or not self.state_hash:
            return False
        claimed = self.source_log.get("state_hash")
        if claimed != self.state_hash:
            return False
        expected = self.compute_digest(secret)
        return hmac.compare_digest(expected, self.digest)

    def to_json(self) -> Dict[str, Any]:
        return {
            "admission_id": self.admission_id,
            "digest": self.digest,
            "issued_at": self.issued_at,
            "policy": self.policy,
            "request_id": self.request_id,
            "robot_id": self.robot_id,
            "role": self.role,
            "state_hash": self.state_hash,
            "target": self.target,
            "task_id": self.task_id,
        }


def admit_assignment(
    assignment: Any,
    manifest: RunManifest,
) -> Tuple[Optional[AdmissionRecord], Optional[str]]:
    """Build and verify an AdmissionRecord. Fail closed on any defect.

    Returns (record, None) on success or (None, reason) on reject.
    Prototype HMAC only — not a PKI signature.
    """
    source_log = getattr(assignment, "source_log", None)
    if not isinstance(source_log, dict):
        return None, "missing_source_log"
    try:
        source_log = json.loads(canonical_json(source_log).decode("utf-8"))
    except (TypeError, ValueError):
        return None, "malformed_source_log"
    state_hash = source_log.get("state_hash")
    if not isinstance(state_hash, str) or not state_hash.strip():
        return None, "missing_state_hash"

    robot_id = str(assignment.robot_id)
    if robot_id not in manifest.roles:
        return None, "unknown_robot"

    role = manifest.roles[robot_id]
    claimed_role = source_log.get("role")
    if claimed_role is not None and claimed_role != role:
        return None, "role_mismatch"

    issued_at = datetime.now(timezone.utc).isoformat()
    unsigned = AdmissionRecord(
        robot_id=robot_id,
        task_id=str(assignment.task_id),
        target=str(assignment.target),
        policy=str(assignment.policy),
        request_id=str(assignment.request_id),
        source_log=source_log,
        state_hash=state_hash,
        role=role,
        admission_id=str(uuid.uuid4()),
        issued_at=issued_at,
        digest="",
    )
    record = replace(unsigned, digest=unsigned.compute_digest(manifest.verification_secret))
    if not record.verify(manifest.verification_secret):
        return None, "unverified"
    return record, None


def verify_admission(record: AdmissionRecord, secret: str) -> bool:
    return record.verify(secret)


class OperationLedger:
    """One-use operation_id set. Replay is rejected before any HTTP."""

    def __init__(self) -> None:
        self._used: set[str] = set()

    def mint(self) -> str:
        return str(uuid.uuid4())

    def already_used(self, operation_id: str) -> bool:
        return bool(operation_id) and operation_id in self._used

    def consume(self, operation_id: str) -> bool:
        if not operation_id or operation_id in self._used:
            return False
        self._used.add(operation_id)
        return True


def dispatch_call_kwargs(
    dispatch_fn: Any,
    start_pose: Any,
    operation_id: Optional[str],
) -> Dict[str, Any]:
    """Resolve the dispatch signature *before* the first call.

    Never used as a TypeError retry. Missing parameters are omitted;
    extra unknown names are not passed.
    """
    try:
        params = inspect.signature(dispatch_fn).parameters
    except (TypeError, ValueError):
        return {}
    names = set(params)
    accepts_var_kw = any(
        p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values()
    )
    kwargs: Dict[str, Any] = {}
    if accepts_var_kw or "start_pose" in names:
        kwargs["start_pose"] = start_pose
    if operation_id is not None and (accepts_var_kw or "operation_id" in names):
        kwargs["operation_id"] = operation_id
    return kwargs


def rebind_ticket_payload(run_id: str, robot_id: str, new_role: str) -> Dict[str, str]:
    return {
        "new_role": new_role,
        "purpose": "rebind_role",
        "robot_id": robot_id,
        "run_id": run_id,
    }


def make_rebind_ticket(
    manifest: RunManifest,
    robot_id: str,
    new_role: str,
) -> Dict[str, str]:
    payload = rebind_ticket_payload(manifest.run_id, robot_id, new_role)
    return {**payload, "digest": hmac_hex(manifest.verification_secret, payload)}


def verify_rebind_authorization(
    manifest: RunManifest,
    robot_id: str,
    new_role: str,
    authorization: Any,
) -> bool:
    expected = hmac_hex(
        manifest.verification_secret,
        rebind_ticket_payload(manifest.run_id, robot_id, new_role),
    )
    if authorization is None:
        return False
    if isinstance(authorization, str):
        provided = authorization
    elif isinstance(authorization, dict):
        provided = str(
            authorization.get("digest") or authorization.get("signature") or ""
        )
        if authorization.get("robot_id") not in (None, robot_id):
            return False
        if authorization.get("new_role") not in (None, new_role):
            return False
        if authorization.get("run_id") not in (None, manifest.run_id):
            return False
    else:
        return False
    if not provided:
        return False
    return hmac.compare_digest(expected, provided)
