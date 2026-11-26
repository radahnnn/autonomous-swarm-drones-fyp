# Teammate Quickstart — Hardware Branch (`hw/integration`)

Welcome! This branch is dedicated to physical hardware integration on your Windows PC so you and Ubuntu/SITL can work in parallel with **zero Git conflicts**.

---

## 1. First-Time Setup on Windows

```powershell
git clone https://github.com/radahnnn/autonomous-swarm-drones-fyp.git
cd autonomous-swarm-drones-fyp
git checkout hw/integration
```

If you already cloned the repo earlier:

```powershell
git fetch origin
git checkout hw/integration
git pull origin hw/integration
```

When you open **Claude Code** inside this folder, start with:
> `Read hardware/CLAUDE.md first. Only edit files inside hardware/ on branch hw/integration.`

---

## 2. How to Push Your Updates (and Pull Ours)

**After finishing a bench session or editing ESP32 code / bench notes:**

```powershell
git status
git add hardware/
git commit -m "hardware: describe what you tested or updated"
git push origin hw/integration
```

**To pull latest software updates from Ubuntu (optional):**

```powershell
git fetch origin
git merge origin/wip/decentralized-sitl-handoff-20261029
```

*(Never edit files outside `hardware/`, and never push to `master` or `wip/decentralized-sitl-handoff-20261029`.)*

---

## 3. Your 4-Step Bench Checklist (Props OFF First!)

1. **Step 1 — Prove CAN RX & TX Alive (`hardware/esp32/twai_diagnostic/`):**
   - **Take all 4 propellers off physically** before plugging in the LiPo (`VBAT`). *(Matek CAN1 5 V only turns on when `VBAT` is connected, not from USB!)*
   - Fill in your exact `CAN_TX_PIN` and `CAN_RX_PIN` at the top of `twai_diagnostic.ino`.
   - Test `LISTEN_ONLY = 1` → look for `NodeStatus ... from node 10` every 1 s with `rx_err=0`, `bus_err=0`.
   - Test `LISTEN_ONLY = 0` → confirm `tx_ok` climbs and `tx_err=0`. *(If TX fails while RX works, check the `74HCT08` / `BSS138` / `1N4007` breadboard notes in `hardware/CLAUDE.md`.)*
2. **Step 2 — MAVLink2 over DroneCAN ↔ Wi-Fi UDP (`hardware/esp32/dronecan_mavlink_bridge/`):**
   - Set the 5 Matek DroneCAN tunnel parameters in QGroundControl (`CAN_D1_UC_SER_EN=1`, reboot, then `S1_NOD=50`, `S1_IDX=0`, `S1_PRO=2`, `S1_BD=115200`).
   - Put your Wi-Fi SSID/password + laptop IP into `dronecan_mavlink_bridge.ino` and flash the ESP32.
   - Verify QGroundControl connects wirelessly over UDP (`14551` for Drone 1).
3. **Step 3 — Gate 1 Pre-Flight Checks (Props OFF):**
   - Bind RC receiver (`SERIAL7`), run 3D accel + compass calibration, insert a FAT32 8–32 GB MicroSD card under the Matek, enable `ARMING_CHECK=1`, `FENCE_ENABLE=1`, battery failsafe, and GCS failsafe, and save the `.param` export into `hardware/params/`.
4. **Step 4 — Replicate on Drone 2:**
   - Repeat with `SYSID_THISMAV=2` and UDP port `14552`.

---

## 4. Open Hardware Details Still Needed (Fill In When You Can)

| Item | Drone 1 | Drone 2 |
|---|---|---|
| ESP32 `CTX` / `CRX` GPIO pins | `TBD` | `TBD` |
| `MCU-2551` 120 ohm termination jumper closed? | `TBD` | `TBD` |
| LiPo battery (cell count S & mAh) | `TBD` | `TBD` |
| RC transmitter & receiver model | `TBD` | `TBD` |
| FAT32 MicroSD card installed under Matek? | `TBD` | `TBD` |
| ESP32 + `MCU-2551` harness built on Drone 2? | `N/A` | `TBD` |
