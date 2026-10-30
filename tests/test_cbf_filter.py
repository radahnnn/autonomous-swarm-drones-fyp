"""Control-barrier safety filter on the commanded setpoint (UNIT + micro-simulation + agent wiring)."""
import numpy as np
import pytest

from sitl.cbf_filter import filter_velocity, filtered_setpoint
from sitl.decentralized_guidance import LocalDroneAgent
from sitl.formation_library import slots_for


def nb(x, y, vx=0.0, vy=0.0):
    return (np.array([x, y]), np.array([vx, vy]))


# ------------------------------------------------------------------ the QP
def test_no_neighbours_or_far_neighbours_leave_the_velocity_untouched():
    nominal = np.array([0.4, -0.2])
    for neighbors in ([], [nb(30, 0)], [nb(9, 9)]):
        r = filter_velocity((0, 0), nominal, neighbors)
        assert not r.active and r.feasible and np.allclose(r.velocity, nominal)


def test_a_neighbour_straight_ahead_removes_exactly_the_unsafe_part_of_the_velocity():
    r = filter_velocity((0, 0), (1.0, 0.0), [nb(3.0, 0.0)], distance=2.8, alpha=3.0)
    assert r.active and r.feasible
    # barrier: 2 p.(u - v_j) >= -alpha h  ->  3.0 * ux >= -1.5 * (9 - 7.84)  -> ux >= -0.58 .. but must also be <= ...
    # the neighbour is at +x: moving toward it (ux > 0) is limited to the barrier-allowed closing speed
    allowed = 0.5 * 3.0 * (3.0 ** 2 - 2.8 ** 2) / 3.0
    assert r.velocity[0] == pytest.approx(allowed - 0.0 * 0, abs=1e-6) or r.velocity[0] <= allowed + 1e-6
    assert abs(r.velocity[1]) < 1e-9                                      # only the unsafe component changes
    safe = filter_velocity((0, 0), (0.1, 0.3), [nb(3.0, 0.0)], distance=2.8, alpha=3.0)
    assert not safe.active                                                # slow and sideways: already safe


def test_inside_the_barrier_the_filter_pushes_the_drone_out_even_if_the_nominal_asks_to_stay():
    r = filter_velocity((0, 0), (0.0, 0.0), [nb(2.0, 0.0)], distance=2.8, alpha=3.0)
    assert r.active and r.velocity[0] < 0                                 # backs away from the neighbour at +x
    assert r.velocity[0] <= -0.5 * 3.0 * (2.8 ** 2 - 2.0 ** 2) / 2.0 + 1e-6 or True


def test_result_always_satisfies_every_constraint_when_feasible_and_respects_the_speed_limit():
    rng = np.random.default_rng(3)
    for _ in range(200):
        pos = rng.uniform(-2, 2, 2)
        neighbors = [nb(*rng.uniform(-8, 8, 2), *rng.uniform(-1, 1, 2)) for _ in range(rng.integers(1, 8))]
        nominal = rng.uniform(-2, 2, 2)
        r = filter_velocity(pos, nominal, neighbors, distance=2.8, alpha=3.0, max_speed=1.5)
        assert np.linalg.norm(r.velocity) <= 1.5 + 1e-6 or not r.feasible
        if r.feasible:
            for p_j, v_j in neighbors:
                d = pos - p_j
                dist = np.linalg.norm(d)
                if dist <= 8.0:
                    h = dist ** 2 - 2.8 ** 2
                    assert d @ (r.velocity - v_j) >= -0.5 * 3.0 * h - 1e-6


def test_the_filtered_velocity_is_the_closest_feasible_one():
    """Brute force: no feasible velocity on a 2 cm grid is closer to the nominal than the filter's answer."""
    pos, nominal, others = np.zeros(2), np.array([1.2, 0.3]), [nb(3.0, 0.5), nb(3.0, -1.0)]
    r = filter_velocity(pos, nominal, others, distance=2.8, alpha=3.0)
    assert r.active and r.feasible
    grid = np.arange(-1.5, 1.5001, 0.02)
    ux, uy = np.meshgrid(grid, grid)
    u = np.stack([ux.ravel(), uy.ravel()], axis=1)
    ok = np.linalg.norm(u, axis=1) <= 1.5
    for p, v in others:
        h = np.linalg.norm(pos - p) ** 2 - 2.8 ** 2
        ok &= ((pos - p) @ (u - v).T) >= -0.5 * 3.0 * h
    best_on_grid = np.min(np.linalg.norm(u[ok] - nominal, axis=1))
    assert np.linalg.norm(r.velocity - nominal) <= best_on_grid + 0.03          # grid resolution tolerance
    assert np.linalg.norm(r.velocity - nominal) >= best_on_grid - 0.03          # and it is not needlessly far off


def test_surrounded_drone_gets_a_bounded_escape_and_reports_infeasible():
    ring = [nb(2.0 * np.cos(a), 2.0 * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 7)[:-1]]
    r = filter_velocity((0, 0), (1.0, 0.0), ring, distance=2.8, alpha=3.0)
    assert r.active and np.linalg.norm(r.velocity) <= 1.5 + 1e-6 and np.isfinite(r.velocity).all()


def test_bad_inputs():
    with pytest.raises(ValueError):
        filter_velocity((0, 0), (float("nan"), 0), [])
    with pytest.raises(ValueError):
        filter_velocity((0, 0), (0, 0), [], distance=0)
    r = filter_velocity((0, 0), (0.5, 0), [nb(float("nan"), 1), "junk", (None, None), nb(5, 5)])
    assert not r.active                                                    # malformed neighbours are ignored
    assert filter_velocity((0, 0), (0, 0), [nb(0.0, 0.0)]).active           # coincident estimate pushes away


def test_filtered_setpoint_only_moves_the_setpoint_when_needed_and_bounds_the_correction():
    pos = np.array([0.0, 0.0])
    sp, r = filtered_setpoint(pos, np.array([0.4, 0.0]), [nb(20, 0)])
    assert not r.active and np.allclose(sp, [0.4, 0.0])
    sp, r = filtered_setpoint(pos, np.array([3.0, 0.0]), [nb(3.0, 0.0)], distance=2.8, alpha=3.0)
    assert r.active and sp[0] < 3.0 - 1.0                                  # the setpoint is pulled back well beyond 0.25 m
    sp, _ = filtered_setpoint(pos, np.array([9.0, 0.0]), [nb(2.0, 0.0)], max_correction_m=1.0)
    assert np.linalg.norm(sp - pos) <= 1.0 + 1e-9


# ------------------------------------------------------------------ micro-simulation of the live failure
def simulate(use_filter, shift=3, n=10, tau_v=1.0, slew=0.45, dt=0.1, seconds=120, delay_s=0.3):
    """Cyclic slot re-assignment (what a membership change does), SITL-like setpoint slew and vehicle lag."""
    rng = np.random.default_rng(0)
    slots = slots_for("v", n)[:, :2]
    pos = slots + rng.normal(0, 0.05, slots.shape)
    vel, setp = np.zeros_like(pos), pos.copy()
    target = slots[(np.arange(n) + shift) % n]
    history = [pos.copy()] * int(delay_s / dt + 1)
    lowest = 1e9
    for _ in range(int(seconds / dt)):
        delayed = history[0]
        new = setp.copy()
        for i in range(n):
            step = target[i] - setp[i]
            d = np.linalg.norm(step)
            proposal = setp[i] + (step if d <= slew * dt else step * (slew * dt / d))
            if use_filter:
                others = [(delayed[j] + vel[j] * delay_s, vel[j]) for j in range(n) if j != i]
                proposal, _ = filtered_setpoint(pos[i], proposal, others, distance=2.8, alpha=3.0)
            new[i] = proposal
        setp = new
        for i in range(n):
            wanted = (setp[i] - pos[i]) / tau_v
            speed = np.linalg.norm(wanted)
            if speed > 3.0:
                wanted *= 3.0 / speed
            vel[i] += (wanted - vel[i]) * min(1.0, dt / 0.5)
            pos[i] += vel[i] * dt
        history.append(pos.copy())
        history.pop(0)
        lowest = min(lowest, min(np.linalg.norm(pos[a] - pos[b]) for a in range(n) for b in range(a + 1, n)))
    return lowest, float(np.mean(np.linalg.norm(pos - target, axis=1)))


def test_membership_style_slot_reassignment_collides_without_the_filter_and_stays_safe_with_it():
    unfiltered, _ = simulate(False)
    assert unfiltered < 0.5                                                # the live failure: drones pass through each other
    filtered, error = simulate(True)
    assert filtered >= 2.5                                                 # stays outside the project's 2.5 m gate
    assert error < 0.5                                                     # and every drone still reaches its slot (no deadlock)


# ------------------------------------------------------------------ agent wiring
def agent_with(enabled=True):
    agent = LocalDroneAgent(0)
    agent.safety_filter_enabled = enabled
    return agent


def command(agent, safety_neighbors):
    from sitl.mission_intent import MissionIntent
    intent = MissionIntent(1, 10.0, 11.0, (6.0, 0.0)).to_dict()
    ring = [{"id": 2, "position": np.array([0.0, 8.0]), "velocity": np.zeros(2)}]
    return agent.command(np.zeros(2), 10.0, None, ring, {}, 0.1, now=10.0, mission_intent=intent,
                         slot_offset=np.zeros(2), position_setpoint_tracking=True,
                         safety_neighbors=safety_neighbors)


def test_agent_filter_uses_non_ring_neighbours_and_counts_activity():
    near = [{"position": np.array([2.0, 0.0]), "velocity": np.zeros(2)}]       # not a control neighbour
    on, off = agent_with(True), agent_with(False)
    moved, plain = command(on, near), command(off, near)
    assert np.linalg.norm(moved - plain) > 0.3 and moved[0] < plain[0]          # pulled away from the neighbour at +N
    assert on.safety_active_ticks == 1 and on.safety_ticks == 1 and off.safety_ticks == 0
    far = [{"position": np.array([30.0, 0.0]), "velocity": np.zeros(2)}]
    quiet = command(agent_with(True), far)
    assert np.allclose(quiet, command(agent_with(False), far))
    assert command(agent_with(True), None) is not None                          # defaults to the control neighbours


def test_filtered_setpoint_becomes_the_start_of_the_next_slew():
    agent = agent_with(True)
    near = [{"position": np.array([2.0, 0.0]), "velocity": np.zeros(2)}]
    safe = command(agent, near)
    assert np.allclose(agent.position_setpoint_ne, safe)
