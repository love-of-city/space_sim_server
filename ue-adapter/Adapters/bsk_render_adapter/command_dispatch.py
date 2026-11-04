"""Allow-listed renderer command registration and simulation-thread dispatch."""
from __future__ import annotations

from typing import Any, Callable, Mapping

from .scene_description import normalize_identifier


class CommandDispatch:
    """Bounded command handling and the unchanged command-result event contract."""

    def register_command_handler(
        self,
        command: str,
        handler: Callable[[Mapping[str, Any], int], Mapping[str, Any] | None],
        *,
        label: str = "",
        target_id: str = "",
        payload: Mapping[str, Any] | None = None,
        requires_confirmation: bool = False,
        show_in_ui: bool = True,
    ) -> None:
        """Register an allow-listed command and optionally expose it in the task UI."""

        normalized = command.strip()
        if not normalized or any(char.isspace() for char in normalized):
            raise ValueError("command must be a non-empty identifier without whitespace")
        if normalized in self._command_handlers:
            raise ValueError(f"duplicate command handler: {normalized}")
        if not callable(handler):
            raise TypeError("command handler must be callable")
        self._command_handlers[normalized] = handler
        if show_in_ui:
            self._ui_settings["commands"].append(
                {
                    "command": normalized,
                    "label": label or normalized,
                    "target_id": normalize_identifier(target_id) if target_id else "",
                    "payload": dict(payload or {}),
                    "requires_confirmation": bool(requires_confirmation),
                }
            )
        self._manifest_revision += 1

    def _handle_manifest_request(self, payload: Mapping[str, Any], sim_time_ns: int) -> Mapping[str, Any]:
        self._record_and_retain_static()
        return {"manifest_revision": str(self._manifest_revision), "sim_time_ns": str(sim_time_ns)}

    def _dispatch_commands(self, current_sim_ns: int) -> None:
        consume = getattr(self.publisher, "consume_command", None)
        if not callable(consume):
            return
        for _ in range(16):
            message = consume()
            if message is None:
                break
            command_id = str(message.get("command_id", ""))
            command = str(message.get("command", ""))
            target_id = str(message.get("target_id", ""))
            payload = message.get("payload", {})
            status = "accepted"
            severity = "info"
            result: Mapping[str, Any] = {}
            error = ""
            if message.get("session_id") != self.session_id:
                status, severity, error = "rejected", "error", "command session does not match the active BSK session"
            elif not command_id:
                status, severity, error = "rejected", "error", "command_id is required"
            elif not isinstance(payload, Mapping):
                status, severity, error = "rejected", "error", "command payload must be an object"
            elif command not in self._command_handlers:
                status, severity, error = "rejected", "error", f"unknown or unregistered command: {command}"
            else:
                try:
                    result = dict(self._command_handlers[command](payload, current_sim_ns) or {})
                except Exception as exception:  # command failures are reported, never hidden
                    status, severity, error = "failed", "error", f"{type(exception).__name__}: {exception}"
            self.publish_event(
                "command_result",
                {
                    "command_id": command_id,
                    "command": command,
                    "target_id": target_id,
                    "status": status,
                    "severity": severity,
                    # ``status`` reports whether the command was accepted, rejected, or
                    # failed.  Do not use "completed" here: for query commands such as
                    # mission.status it is easily confused with the mission phase.
                    "message": error or f"{command} {status}",
                    "sim_time_ns": str(current_sim_ns),
                    "result": dict(result),
                },
            )

    def process_commands(self, current_sim_ns: int) -> None:
        """Consume pending UI commands on the caller's simulation thread."""

        self._dispatch_commands(int(current_sim_ns))
