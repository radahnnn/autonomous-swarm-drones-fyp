import pytest

from sitl.estimate_health import EstimateHealth


def feed(health, samples):
    return [health.update(p, t) for t, p in samples]


def test_smooth_flight_stays_ok():
    h = EstimateHealth()
    states = feed(h, [(0.1 * i, (0.3 * 0.1 * i, 0.0)) for i in range(200)])        # 0.3 m/s
    assert set(states) == {"ok"} and h.glitches == 0


def test_a_ten_metre_per_second_jump_degrades_then_recovers():
    h = EstimateHealth(recover_s=5.0)
    feed(h, [(0.0, (0.0, 0.0)), (1.0, (0.2, 0.0))])
    assert h.update((10.2, 0.0), 2.0) == "degraded" and h.glitches == 1            # 10 m in 1 s
    assert h.update((10.3, 0.0), 4.0) == "degraded"
    assert h.update((10.4, 0.0), 7.1) == "ok"                                        # 5 s without a glitch


def test_persistent_glitches_end_in_land():
    h = EstimateHealth(recover_s=5.0, land_after_s=15.0)
    t, n = 0.0, 0.0
    h.update((n, 0.0), t)
    states = []
    for step in range(20):
        t += 1.0
        n += 8.0 if step % 2 == 0 else -8.0                                          # jumping back and forth
        states.append(h.update((n, 0.0), t))
    assert states[-1] == "land" and h.glitches >= 15


def test_samples_too_close_in_time_and_bad_values_are_not_judged():
    h = EstimateHealth()
    h.update((0.0, 0.0), 0.0)
    assert h.update((1.0, 0.0), 0.01) == "ok"                    # 1 cm of time: not judged
    assert h.update((float("nan"), 0.0), 1.0) == "ok"
    assert h.update((0.2, 0.0), 1.0) == "ok"


def test_validation():
    with pytest.raises(ValueError):
        EstimateHealth(max_speed_m_s=0)
    with pytest.raises(ValueError):
        EstimateHealth(recover_s=10.0, land_after_s=5.0)


def test_barrier_keeps_extra_distance_from_a_degraded_neighbour():
    import numpy as np
    from sitl.cbf_filter import filter_velocity
    me, other = np.zeros(2), np.array([3.5, 0.0])
    toward = np.array([1.0, 0.0])
    normal = filter_velocity(me, toward, [(other, np.zeros(2))], distance=2.8, alpha=3.0)
    wary = filter_velocity(me, toward, [(other, np.zeros(2), 2.0)], distance=2.8, alpha=3.0)
    assert wary.velocity[0] < normal.velocity[0] and wary.active
    assert filter_velocity(me, toward, [(other, np.zeros(2), -1.0)]).constraints == 0     # malformed extra ignored


def test_degraded_flag_round_trips_over_the_peer_link():
    import os, time
    from tests.test_coordinator_election import make_pair
    a, b = make_pair(41000 + os.getpid() % 3000)
    try:
        now = time.time()
        a.publish([1.0, 2.0], [0.0, 0.0], now, [1], now, landing=False, degraded=True)
        got = []
        for _ in range(60):
            got = b.receive(now, [0])
            if got:
                break
            time.sleep(0.02)
        assert got and got[0]["degraded"] is True
        with pytest.raises(ValueError):
            a.publish([1.0, 2.0], [0.0, 0.0], now, [1], now, degraded="yes")
    finally:
        a.close(); b.close()


def _agent():
    import numpy as np
    from sitl.independent_agent_node import IndependentAgentNode
    node = object.__new__(IndependentAgentNode)
    node.drone_id, node.airborne, node.landing = 0, True, False
    node.estimate_health = EstimateHealth(recover_s=5.0, land_after_s=15.0)
    node._frozen_target = None
    node.last_position_target_ne = np.array([1.0, 1.0])
    node.position, node.position_at = np.array([0.0, 0.0, -5.0]), 100.0
    node.landed_reason = None
    node._command_land = lambda reason: setattr(node, "landed_reason", reason)
    return node


def test_agent_freezes_on_a_glitch_and_lands_if_it_persists(capsys):
    import numpy as np
    node = _agent()
    node._tick_estimate_health(100.0)
    node.position, node.position_at = np.array([10.0, 0.0, -5.0]), 101.0               # 10 m in 1 s
    node._tick_estimate_health(101.0)
    assert node.estimate_health.degraded and np.allclose(node._frozen_target, [1.0, 1.0])
    assert "estimate ok -> degraded" in capsys.readouterr().out
    n = 10.0
    for step in range(2, 20):                                                           # keeps glitching
        n = -n
        node.position, node.position_at = np.array([n, 0.0, -5.0]), 100.0 + step
        node._tick_estimate_health(100.0 + step)
    assert node.landed_reason == "position_estimate_or_flyaway"


def test_peers_marked_degraded_get_extra_distance():
    import numpy as np
    from sitl.independent_agent_node import IndependentAgentNode
    node = object.__new__(IndependentAgentNode)
    node._peer_memory = {1: (np.array([3.0, 0.0]), np.zeros(2), 0.0, 10.0), 2: (np.array([9.0, 0.0]), np.zeros(2), 0.0, 10.0)}
    node._peer_degraded = {1: True, 2: False}
    extras = sorted(p["extra_m"] for p in node._safety_neighbors(10.0))
    assert extras == [0.0, IndependentAgentNode.DEGRADED_PEER_EXTRA_M]


def test_high_rate_jitter_is_not_a_glitch():
    import random
    random.seed(1)
    h = EstimateHealth()
    states = feed(h, [(0.1 * i, (0.03 * i + random.uniform(-0.3, 0.3), random.uniform(-0.3, 0.3))) for i in range(300)])
    assert set(states) == {"ok"}


def test_flyaway_from_a_steady_target_degrades_then_lands_if_extreme():
    h = EstimateHealth(flyaway_m=4.0, flyaway_s=2.0, flyaway_land_m=10.0)
    target = (0.0, 0.0)
    assert h.check_tracking((1.0, 0.0), target, 0.0) == "ok"            # normal tracking error
    assert h.check_tracking((6.0, 0.0), target, 1.0) == "ok"            # far, but not for long yet
    assert h.check_tracking((7.0, 0.0), target, 3.1) == "degraded" and h.flyaways == 1
    assert h.check_tracking((12.0, 0.0), target, 5.2) == "land"         # still far and beyond 10 m
    calm = EstimateHealth()
    assert all(calm.check_tracking((3.5, 0.0), target, float(t)) == "ok" for t in range(30))   # within 4 m: fine
