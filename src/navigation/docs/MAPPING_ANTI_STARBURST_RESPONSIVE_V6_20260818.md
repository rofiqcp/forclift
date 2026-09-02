# Mapping Anti-Starburst + Responsive Map V6 — 2026-08-18

## Evidence from `log(20260818-102818)`

Latest mapping runtime (`mapping_gui_20260818_172549.txt`) shows:

- verified 2-byte parser is active (`mode=2B`); keep it;
- physical complete-scan rate is about **6.1 Hz** in this run;
- CT ring markers track completed revolutions almost one-for-one
  (`complete=627`, `ring=629` near the end);
- the old V5 filter removed only about **7.6 bins/scan** on average, while red
  anomalous rays were still visible;
- Hector still rejected repeated impossible scan matches and suppressed false yaw;
- RViz also reported LaserScan message-filter queue overflow / future TF lookup;
- the saved PGM itself contains the radial fan, so this is not only an RViz display issue.

## V6 changes

1. **Physical revolution boundary is CT ring-start, not a 350° software cut.**
   - startup partial data is discarded;
   - a scan is closed only by the next physical ring marker;
   - minimum coverage/point sanity is checked;
   - an invalid/unfinished revolution is dropped instead of mixed with the next one;
   - LaserScan timestamp is the first packet time of that physical revolution.

2. **Two-stage anti-starburst filter.**
   - Stage A: local same-surface support within +/-3 bins OR repeat evidence;
   - two previous raw revolutions are used after warm-up, preventing one bad scan
     from immediately validating itself;
   - near foreground returns are protected;
   - Stage B keeps the existing two-sided median FAR-shadow test;
   - **raw-scan fallback is removed**. If filtering leaves too little safe
     geometry, the revolution is dropped rather than injected into the map.

3. **Hector motion latency reduced without removing hard safety gates.**
   - motion confirmation: 3 -> 2 scans;
   - median/change thresholds returned closer to the responsive master;
   - local matcher-map update threshold: 3 cm / 1° -> 1.5 cm / ~0.5°;
   - max scan gap: 0.40 s, so one intentionally dropped noisy frame at ~6 Hz
     does not force a re-seed, while longer stream stalls still do.
   - IMU yaw plausibility gate and physical velocity rejection remain enabled.

4. **SLAM Toolbox responsiveness restored to the source's own master reference.**
   - `map_update_interval: 0.1`;
   - `minimum_time_interval: 0.05`;
   - `minimum_travel_distance: 0.01`;
   - `minimum_travel_heading: 0.0087`;
   - `correlation_search_space_dimension: 0.50`;
   - bounded scan queue: 3;
   - occupancy evidence remains hardened (`min_pass_through: 3`,
     `occupancy_threshold: 0.35`).

5. **RViz LaserScan is latest-scan oriented.**
   - decay 0;
   - larger display queue/depth to tolerate short TF timing bursts.

## What V6 intentionally does NOT change

- USB physical role resolver;
- LiDAR port / baudrate;
- IMU port / behavior;
- 2-byte parser mode;
- motor STOP safety added in V5;
- mapping ownership: LiDAR Hector odometry + async SLAM Toolbox;
- IMU is not integrated into occupancy-map X/Y.

## Runtime acceptance targets

Start from a **new mapping session / clean map**. An old map already containing
starburst cells cannot be cleaned retroactively by the filter.

Expected startup signature:

```text
[LIDAR-PARSER] mode=VERIFIED_2BYTE ...
[ANTI-STARBURST-V6] ... temporal=on two_frame=true ...
```

During a slow mapping lap:

- `/scan` must continue without long stalls;
- `complete` should remain approximately one behind `ring` after startup;
- `incomplete_rev` should stay near zero;
- occasional `[LIDAR-FILTER-DROP]` is acceptable; continuous drops are not;
- Hector `jumps` should grow much more slowly than in the 17:25 run;
- false yaw suppression while physically stationary should become rare;
- `/map` should visibly update at about the configured 0.1 s publication cadence
  whenever accepted motion/geometry is available;
- walls should accumulate as compact edges rather than radial fans.

The actual physical scan rate in the supplied run is ~6.1 Hz even though the
configuration target says 10 Hz. V6 does not send undocumented motor-frequency
commands; it makes mapping consume each physical revolution cleanly and promptly.
