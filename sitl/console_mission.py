"""Mission commands: patrol routes, goto, transit with the tower-gap manoeuvre, camera capture, geofence checks.

Part of `IndependentSwarmConsole` (independent_swarm_console.py); split out for size, behaviour unchanged."""
from __future__ import annotations
from datetime import datetime, timezone
import json
import math
import threading
import time
import numpy as np
from sitl.formation_library import slots_for
from sitl import gap_manoeuvre
from sitl.route import Route, square_patrol


class MissionMixin:
    def _slot_points(self, goal_ne, formation=None, slots=None) -> np.ndarray:
        """North/East positions every drone would hold for a goal (current or given formation slots)."""
        if slots is None:
            slots = (self.formation_slots if getattr(self, "formation_slots", None) is not None
                     and formation is None else slots_for(formation or self.formation, self.count))
        return np.asarray(goal_ne, dtype=float) + np.asarray(slots, dtype=float)[:, :2]

    def _fence_problem(self, points_ne, altitude_m=None) -> str | None:
        """None if every point fits inside the fence (margin-shrunk), else a message for the operator."""
        fence = self.geofence
        if fence is None:
            return None
        for index, point in enumerate(np.atleast_2d(points_ne)):
            if not fence.contains(point, inner=True):
                return (f"D{index + 1} would be at N {point[0]:.1f} E {point[1]:.1f}, outside the geofence "
                        f"(kept {fence.margin_m:g} m inside).")
        if altitude_m is not None and fence.max_alt_m is not None and \
                altitude_m > fence.clamp_altitude(fence.max_alt_m):
            return f"{altitude_m:.1f} m is above the geofence ceiling ({fence.max_alt_m:g} m)."
        return None

    def start_patrol(self, args) -> dict:
        """patrol square <side_m> [speed_m_s]: every drone carries the route and keeps flying it."""
        def result(accepted, message, **details):
            self.last_command_result = {"accepted": accepted, "message": message, **details}
            print((">> " if accepted else ">> Patrol blocked: ") + message, flush=True)
            return self.last_command_result
        if len(args) < 2 or args[0] != "square":
            return result(False, "usage: patrol square <side_m> [speed_m_s]")
        if self.route is not None:
            return result(False, "A route is already active; use 'route stop' first.", code="route_active")
        if self.mode_switch is not None and self.mode_switch.busy:
            return result(False, "A control-mode switch is in progress.", code="mode_switch_active")
        if (getattr(self, "formation_transition_deadline", None) is not None or
                (self.transition_protocol and (self.transition.busy or self.formation_hold))):
            return result(False, "Finish or abort the formation transition first.", code="transition_active")
        try:
            side = float(args[1])
            speed = (float(args[2]) if len(args) > 2 else 0.3) * self.time_scale   # given in simulated m/s
            route = square_patrol(self.goal_ne, side, time.time() + 2.0, speed)
        except ValueError as exc:
            return result(False, str(exc))
        for index, waypoint in enumerate(route.waypoints):
            problem = self._fence_problem(self._slot_points(waypoint), self.target_alt_m)
            if problem:
                return result(False, f"waypoint {index + 1} does not fit the geofence: {problem}", code="geofence")
        self.route = route
        self._note_metric("route_start", side_m=side, speed_m_s=speed)
        self.event("route_start", waypoints=[list(p) for p in route.waypoints], speed_m_s=speed,
                   start_at=route.start_at)
        self._publish_intent()
        return result(True, f"Patrol square {side:g} m at {speed:g} m/s starts in 2 s. Every drone carries the "
                      "route, so decentralized and hybrid swarms keep patrolling even if the coordinator is lost.",
                      side_m=side, speed_m_s=speed)

    def goto(self, north: str, east: str) -> None:
        """goto <north_m> <east_m>: move the swarm's goal (the formation centre) to a place on the shared map."""
        try:
            target = np.array([float(north), float(east)])
            if not np.isfinite(target).all():
                raise ValueError
        except ValueError:
            print(">> usage: goto <north_m> <east_m>")
            return
        if self.route is not None:
            print(">> Move blocked: a route is active; use 'route stop' first.")
            return
        if (self.mode_switch is not None and self.mode_switch.busy) or \
                (self.transition_protocol and (self.transition.busy or self.formation_hold)):
            print(">> Move blocked: a mode switch or formation transition is in progress.")
            return
        problem = self._fence_problem(self._slot_points(target), self.target_alt_m)
        if problem:
            print(f">> Move blocked by the geofence: {problem}")
            return
        self.goal_ne = target
        self._publish_intent()
        self.event("goto", north=float(target[0]), east=float(target[1]))
        print(f">> Swarm goal moved to N {target[0]:.1f} E {target[1]:.1f}; the drones follow at the slew speed.", flush=True)

    def capture(self, label: str = "capture") -> dict:
        """capture [label]: every drone saves one camera frame (needs a Gazebo world generated with --camera).

        Operator-triggered, not onboard: the console process reads the cameras through gz-transport. An index with each
        drone's reported position at capture time is written next to the images.
        """
        from sitl.gz_camera import capture_all
        from sitl.generate_gazebo_world import camera_topic
        safe_label = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label)[:40] or "capture"
        directory = self.run_dir / "captures"
        stamp = f"{int(time.time())}_{safe_label}"
        try:
            saved = capture_all({n: camera_topic(n) for n in range(1, self.count + 1)}, directory, stamp, timeout_s=15.0)
        except Exception as exc:                                  # no bindings, no Gazebo, no cameras
            print(f">> Capture failed: {exc}", flush=True)
            self.event("capture_failed", error=str(exc))
            return {"accepted": False, "message": str(exc)}
        with self.lock:
            states = {index: state.copy() for index, state in self.latest.items()}
        frames = []
        for number, path in saved.items():
            position = (states.get(number - 1) or {}).get("position")
            frames.append({"drone": number, "file": None if path is None else path.name, "position": position})
        index_path = directory / "index.jsonl"
        directory.mkdir(parents=True, exist_ok=True)
        with open(index_path, "a") as handle:
            handle.write(json.dumps({"utc": datetime.now(timezone.utc).isoformat(), "label": safe_label,
                                     "frames": frames}) + "\n")
        got = sum(1 for frame in frames if frame["file"])
        self._note_metric("capture", label=safe_label, frames=got, expected=self.count)
        self.event("capture", label=safe_label, frames=got, expected=self.count)
        print(f">> Captured {got}/{self.count} camera frames ({safe_label}) -> {directory}", flush=True)
        return {"accepted": got == self.count, "message": f"{got}/{self.count} frames", "frames": frames}

    # ------------------------------------------------------------------ gap manoeuvre (V -> column -> V)
    _gap = None

    def transit(self, north: str, east: str, speed: str = "0.3") -> None:
        """transit <north> <east> [speed]: fly straight there; squeeze through gaps between sensed obstacles."""
        try:
            target = np.array([float(north), float(east)])
            speed_sim = float(speed)
            heading = gap_manoeuvre.heading_to(self.goal_ne, target)
            route = Route(((float(self.goal_ne[0]), float(self.goal_ne[1])), (float(target[0]), float(target[1]))),
                          time.time() + 2.0, speed_sim * self.time_scale)
        except ValueError as exc:
            print(f">> Transit refused: {exc}")
            return
        if self.route is not None:
            print(">> Transit refused: a route is active; use 'route stop' first.")
            return
        self.route = route
        self._gap = {"state": "cruise", "target": target, "heading": heading, "speed": speed_sim, "passed": []}
        self._note_metric("transit_start", north=float(target[0]), east=float(target[1]))
        self.event("transit_start", target=[float(target[0]), float(target[1])], heading_deg=math.degrees(heading))
        self._publish_intent()
        print(f">> Transit to N {target[0]:.1f} E {target[1]:.1f} at {speed_sim:g} m/s; watching for gaps.", flush=True)

    GAP_COLUMN_SPACING_M = 8.0

    GAP_PASS_SPEED_FACTOR = 0.6

    def _resume_transit(self, speed_factor: float = 1.0) -> None:
        g = self._gap
        try:
            self.route = Route(((float(self.goal_ne[0]), float(self.goal_ne[1])),
                                (float(g["target"][0]), float(g["target"][1]))),
                               time.time() + 2.0, max(0.1, g["speed"] * speed_factor) * self.time_scale)
        except ValueError:                                    # already at the target
            self.route, g["state"] = None, "done"
            return
        self._publish_intent()

    def _pause_transit(self, at=None) -> None:
        if self.route is not None:
            self.goal_ne = np.asarray(self.route.position(time.time()) if at is None else at, dtype=float)
            self.route = None
            self._publish_intent()

    def _gap_formation(self, style: str, heading: float, spacing) -> None:
        """Request a gap-manoeuvre formation; if it is refused outright (not merely cancelled and retried), retry once,
        then give up the manoeuvre and hold, so the swarm never waits forever (live finding: a refused column left the
        state machine stuck)."""
        for attempt in range(2):
            self.last_formation_result = None
            self._set_formation(style, heading, spacing)
            result = getattr(self, "last_formation_result", None) or {}
            if result.get("accepted", True):
                return
            if attempt == 0:
                time.sleep(3.0 / self.time_scale)
        g = self._gap
        if g is not None:
            g["state"] = "done"
        self.event("gap_manoeuvre_abandoned", formation=style, reason=str(result.get("message")))
        print(f"!! Gap manoeuvre stopped: {style.upper()} could not be planned ({result.get('message')}); holding.",
              flush=True)

    def _known_obstacles(self) -> list:
        from sitl.operator_dashboard import _sensed_obstacles
        with self.lock:
            latest = {index: state.copy() for index, state in self.latest.items()}
        known = _sensed_obstacles(latest)
        for raw in getattr(self.args, "obstacle", ()):
            try:
                known.append([float(v) for v in str(raw).split(",")])
            except ValueError:
                pass
        return known

    def _drone_positions(self) -> np.ndarray:
        with self.lock:
            return np.asarray([self.latest.get(i, {}).get("position", [np.nan] * 3)[:2] for i in range(self.count)],
                              dtype=float)

    def _tick_gap(self) -> None:
        """Gap state machine: cruise -> squeeze (column) -> through -> reform (V) -> cruise ... -> done."""
        g = self._gap
        if g is None or g["state"] == "done":
            return
        heading, sigma = g["heading"], self.position_sigma_m
        busy = self.transition_protocol and (self.transition.busy or self.transition_pending is not None)
        if g["state"] == "cruise":
            if self.route is None or self.route.finished(time.time()):
                g["state"] = "done"
                self._pause_transit()
                print(">> Transit complete.", flush=True)
                return
            here = self.route.position(time.time())
            gap = gap_manoeuvre.find_gap(self._known_obstacles(), here, heading, lookahead_m=40.0)
            if gap is None or any(np.allclose(gap.centre, c, atol=1.0) for c in g["passed"]):
                return
            positions = self._drone_positions()
            if not np.isfinite(positions).all():
                return
            relative = positions - positions.mean(axis=0)
            if not gap_manoeuvre.needs_squeeze(relative, heading, gap, sigma):
                return
            stop = gap_manoeuvre.stop_point(gap, here, heading, gap_manoeuvre.formation_length(relative, heading) * 2)
            if float((stop - here) @ gap_manoeuvre.unit(heading)) > 0.5:
                return                                         # not there yet: keep flying towards the stop point
            self._pause_transit()
            g["gap"] = gap
            if not gap_manoeuvre.passable_single_file(gap, sigma):
                g["state"] = "done"
                self.event("gap_too_narrow", free_width_m=gap.free_width_m)
                print(f"!! Gap ahead is {gap.free_width_m:.1f} m wide: too narrow even in single file; holding.",
                      flush=True)
                return
            g["state"] = "squeeze"
            self.event("gap_squeeze", centre=list(gap.centre), free_width_m=gap.free_width_m)
            self._note_metric("gap_squeeze", free_width_m=round(gap.free_width_m, 2))
            print(f">> Gap of {gap.free_width_m:.1f} m ahead (formation needs more): changing to a COLUMN.", flush=True)
            # A moving single file under GPS error "breathes" along the track (live: 0.85 m at ~5.5 m spacing).
            threading.Thread(target=self._gap_formation, args=("column", heading, self.GAP_COLUMN_SPACING_M),
                             daemon=True).start()
        elif g["state"] == "squeeze" and not busy:
            if self.formation == "column":
                g["state"] = "through"
                print(">> Column formed: passing through the gap.", flush=True)
                self._resume_transit(self.GAP_PASS_SPEED_FACTOR)
            elif self.transition_pending is None and not getattr(self, "_transition_retries", {}).get("column"):
                pass                                          # still planning / settling inside _set_formation
        elif g["state"] == "through":
            positions = self._drone_positions()
            if np.isfinite(positions).all() and gap_manoeuvre.all_past(positions, g["gap"], heading):
                self._pause_transit()
                g["passed"].append(g["gap"].centre)
                g["state"] = "reform"
                print(">> Every drone is past the gap: back to the V.", flush=True)
                threading.Thread(target=self._gap_formation, args=("v", heading, None), daemon=True).start()
        elif g["state"] == "reform" and not busy and self.formation == "v":
            g["state"] = "cruise"
            self.event("gap_passed", centre=list(g["gap"].centre))
            self._note_metric("gap_passed")
            print(">> V restored: continuing the transit.", flush=True)
            self._resume_transit()

    def route_command(self, args) -> None:
        if not args or args[0] not in ("stop", "status"):
            print(">> usage: route stop | route status")
            return
        if self.route is None:
            print(">> No route is active.")
            return
        now = time.time()
        position = self.route.position(now)
        if args[0] == "status":
            print(f">> Route: {len(self.route.waypoints)} waypoints, {self.route.speed_m_s:g} m/s, "
                  f"goal now N {position[0]:.1f} E {position[1]:.1f}")
            return
        self.goal_ne = np.asarray(position, dtype=float)    # hold where the route is: no jump
        self.route = None
        self._note_metric("route_stop")
        self.event("route_stop", goal_ne=[float(self.goal_ne[0]), float(self.goal_ne[1])])
        self._publish_intent()
        print(f">> Route stopped; holding N {self.goal_ne[0]:.1f} E {self.goal_ne[1]:.1f}.")

    def _movement_blocked(self, action: str, step: float) -> bool:
        """Refuse a move (or altitude change) that would take any drone outside the geofence."""
        if self.route is not None and action in ("w", "north", "s", "south", "a", "west", "d", "east"):
            print(">> Move blocked: a route is active; use 'route stop' first.")
            self.last_command_result = {"accepted": False, "message": "A route is active."}
            return True
        if self.geofence is None:
            return False
        goal = self.goal_ne.copy()
        altitude = self.target_alt_m
        if action in ("w", "north"):
            goal[0] += step
        elif action in ("s", "south"):
            goal[0] -= step
        elif action in ("a", "west"):
            goal[1] -= step
        elif action in ("d", "east"):
            goal[1] += step
        elif action in ("u", "up"):
            altitude += step
        elif action in ("j", "down"):
            altitude -= step
        problem = self._fence_problem(self._slot_points(goal), altitude)
        if problem:
            print(f">> Move blocked by the geofence: {problem}")
            self.last_command_result = {"accepted": False, "message": f"Geofence: {problem}"}
            return True
        return False
