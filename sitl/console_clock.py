"""Operator clock synchronisation and the clock-error budget for synchronized starts.

Part of `IndependentSwarmConsole` (independent_swarm_console.py); split out for size, behaviour unchanged."""
from __future__ import annotations
import time
import uuid
from sitl.clock_budget import compute_budget
from sitl.clock_sync import best_clock_sample, estimate_clock_offset
from sitl.operator_udp import send_to_nodes

_CLOCK_RESYNC_INTERVAL_S = 60.0


class ClockSyncMixin:
    def _record_clock_probe(self, agent_id: int, message: dict) -> None:
        probe_id = message.get("probe_id")
        if not isinstance(probe_id, str):
            return
        with self.lock:
            pending = self.clock_probe_pending.pop(probe_id, None)
        if pending is None or pending[0] != agent_id or message.get("drone_id") != agent_id:
            return
        t0, t0_monotonic = pending[1], pending[2]
        t3 = time.time()
        t3_monotonic = time.monotonic()
        operator_wall_elapsed = t3 - t0
        operator_monotonic_elapsed = t3_monotonic - t0_monotonic
        if abs(operator_wall_elapsed - operator_monotonic_elapsed) > 0.005:
            self.event("clock_probe_rejected", drone_id=agent_id,
                       reason="operator_wall_clock_changed_during_probe",
                       wall_elapsed_s=operator_wall_elapsed,
                       monotonic_elapsed_s=operator_monotonic_elapsed)
            return
        try:
            offset, round_trip = estimate_clock_offset(
                t0, message.get("agent_received_at"), message.get("agent_sent_at"), t3,
                agent_processing_s=message.get("agent_processing_s"),
            )
        except (TypeError, ValueError):
            self.event("clock_probe_rejected", drone_id=agent_id,
                       reason="invalid_timestamps_or_agent_clock_step")
            return
        with self.lock:
            self.clock_probe_samples[agent_id].append((offset, round_trip))
        self.event("clock_probe_sample", drone_id=agent_id, offset_s=offset,
                   round_trip_s=round_trip)

    def _synchronize_clocks(self, rounds: int = 5, max_round_trip_s: float = 0.25,
                            ack_timeout_s: float = 6.0) -> None:
        """Measure every agent clock, then distribute offsets for peer/intent timestamps."""
        with self.lock:
            self.clock_probe_pending.clear()
            self.clock_probe_samples = {index: [] for index in range(self.count)}
        for _round in range(rounds):
            for agent_id, target in self.operator_targets.items():
                probe_id = uuid.uuid4().hex
                t0 = time.time()
                t0_monotonic = time.monotonic()
                with self.lock:
                    self.clock_probe_pending[probe_id] = (agent_id, t0, t0_monotonic)
                send_to_nodes(
                    {"version": 1, "type": "clock_probe", "probe_id": probe_id},
                    endpoints={agent_id: target}, auth_key=self.operator_auth_key,
                )
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                with self.lock:
                    enough = all(len(self.clock_probe_samples[index]) >= _round + 1
                                 for index in range(self.count))
                if enough:
                    break
                time.sleep(0.01)

        offsets = {}
        uncertainty = {}
        for agent_id in range(self.count):
            try:
                offsets[agent_id], uncertainty[agent_id] = best_clock_sample(
                    self.clock_probe_samples[agent_id], max_round_trip_s,
                )
            except ValueError as exc:
                raise TimeoutError(
                    f"clock synchronization failed for D{agent_id + 1}: {exc}"
                ) from exc
        with self.lock:
            self.clock_probe_pending.clear()
        self.clock_offsets_s = offsets
        self.clock_uncertainty_s = uncertainty
        self.clock_sync_sequence += 1
        epoch = uuid.uuid4().hex
        sync_message = {
            "version": 1, "type": "clock_sync", "sequence": self.clock_sync_sequence,
            "epoch": epoch,
            "offsets_s": {str(index): value for index, value in offsets.items()},
        }
        self.clock_sync_message = sync_message
        self.last_clock_sync_publish = time.monotonic()
        self._send_to_agents(sync_message)
        deadline = time.monotonic() + ack_timeout_s
        next_retry = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            with self.lock:
                ready = all(self.latest.get(index, {}).get("clock_sync_epoch") == epoch
                            for index in range(self.count))
            if ready:
                self.event("clock_sync_complete", epoch=epoch,
                           offsets_s={str(index): value for index, value in offsets.items()},
                           uncertainty_s={str(index): value
                                          for index, value in uncertainty.items()},
                           resynchronized=self.clock_sync_sequence > 1,
                           budget=self._clock_budget_dict())
                self.last_clock_sync_measurement = time.monotonic()
                return
            if time.monotonic() >= next_retry:
                self._send_to_agents(sync_message)
                next_retry = time.monotonic() + 0.5
            time.sleep(0.05)
        raise TimeoutError("agents did not acknowledge the measured clock offsets")

    def _clock_budget(self):
        """Worst-case start skew and minimum safe lead from the measured offset uncertainties (None if unknown)."""
        if not self.clock_uncertainty_s:
            return None
        age = max(0.0, time.monotonic() - self.last_clock_sync_measurement) if self.last_clock_sync_measurement else \
            _CLOCK_RESYNC_INTERVAL_S
        try:
            return compute_budget(self.clock_uncertainty_s, age_s=min(age, _CLOCK_RESYNC_INTERVAL_S * 2))
        except ValueError:
            return None

    def _clock_budget_dict(self):
        budget = self._clock_budget()
        return None if budget is None else budget.as_dict()

    def _lead_too_short(self, lead_s):
        """Message if the announcement lead cannot cover the clock-error budget, else None."""
        budget = self._clock_budget()
        if budget is not None and lead_s < budget.required_lead_s:
            return (f"The {lead_s:.1f}s start lead is shorter than the clock budget needs "
                    f"({budget.required_lead_s:.2f}s: skew bound {budget.skew_bound_s * 1000:.0f} ms).")
        return None
