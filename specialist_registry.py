from dataclasses import dataclass
from typing import Dict, FrozenSet, Optional, Set
from copy import deepcopy


@dataclass(frozen=True)
class SpecialistDef:
    role: str
    permitted_tasks: FrozenSet[str]
    description: str = ""


class SpecialistRegistry:
    """
    Central registry of robot roles.
    Roles are registered at startup and are read-only after lock.
    No new roles can be added after the registry is locked.
    """

    def __init__(self) -> None:
        self._roles: Dict[str, SpecialistDef] = {}
        self._locked: bool = False

    def register(self, role: str, permitted_tasks: Set[str], description: str = "") -> None:
        if self._locked:
            raise PermissionError("Registry is locked. No new roles can be registered.")
        if role in self._roles:
            raise ValueError(f"Role '{role}' already registered.")
        self._roles[role] = SpecialistDef(
            role=role,
            permitted_tasks=frozenset(permitted_tasks),
            description=description,
        )

    def lock(self) -> None:
        self._locked = True

    def is_locked(self) -> bool:
        return self._locked

    def get(self, role: str) -> Optional[SpecialistDef]:
        return self._roles.get(role)

    def is_permitted(self, role: str, task_type: str) -> bool:
        spec = self.get(role)
        if spec is None:
            return False
        return task_type in spec.permitted_tasks

    def all_roles(self) -> Dict[str, SpecialistDef]:
        return deepcopy(self._roles)


def build_default_registry() -> SpecialistRegistry:
    """
    Default specialist set for a governed warehouse / factory floor.
    Extend before calling lock() if the deployment needs additional roles.
    """
    reg = SpecialistRegistry()
    reg.register(
        role="assembler",
        permitted_tasks={"assemble", "idle", "returning", "moving", "working"},
        description="Performs assembly tasks at designated nodes.",
    )
    reg.register(
        role="carrier",
        permitted_tasks={"carry", "idle", "returning", "moving", "working"},
        description="Transports materials between zones.",
    )
    reg.register(
        role="inspector",
        permitted_tasks={"inspect", "idle", "returning", "moving", "working"},
        description="Performs quality inspection at task nodes.",
    )
    reg.register(
        role="charger",
        permitted_tasks={"charge", "idle", "moving"},
        description="Stays near charging zones; limited locomotion.",
    )
    reg.lock()
    return reg
