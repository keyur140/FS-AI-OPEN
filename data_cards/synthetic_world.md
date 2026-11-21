# Data card: synthetic tracks, sensors and runs (stage 2)

Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X). Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science, Brunel University London, UK; (2) Independent researcher. Conducted independently; not an official Brunel University London project. Drafted with an AI assistant (Claude, Anthropic); decisions by the author.

**In plain words.** Every track, sensor reading and lap in `runs/` was made by this software from a seed number. Nothing was recorded on a real car or a real track. The same seed always gives the same track and, in the desk runs, the same result to the byte.

| What | How it is made | Sources for the numbers | Where |
|---|---|---|---|
| Tracks | Seeded generator per event: Acceleration lane, Skidpad figure-of-eight, closed Autocross/Trackdrive loops. Checked by `validate()` (width, cone spacing, longest straight, hairpins, staging) | FS-AI 2026 Rules chapter D (D4 to D8), IMechE FS-AI cones specification 2021 (DE6.3, Table 2), FS Driverless Specification 2026 (DS 1). Each value and source is in `config/tracks.yaml`; our own choices are marked | `fsai/autonomy/track.py` |
| LiDAR | Ray-sampled spinning LiDAR; cones are cone shapes of the official sizes on flat ground; range noise and occlusion | Sensor layout and noise are fsai-open estimates (`config/autonomy.yaml`, `sensors:`) | `fsai/autonomy/sensors.py` |
| Camera | Cone detections with range and bearing noise, colour mistakes, misses, phantom detections and a range limit | fsai-open estimates | `fsai/autonomy/sensors.py` |
| Trackside clutter (`clutter_level: 1`) | Barrier runs, standing people, crouching photographers, grass patches, heat-shimmer returns, and some clutter seen as cones by the camera | fsai-open estimates, chosen to be harder than a tidy track | `fsai/autonomy/sensors.py` (`make_clutter`) |
| Vehicle | Kinematic and linear-tyre models, tyre grip with seeded grip patches, steering compliance, steering rate limit, brake pressure build-up, actuator dead times | ADS-DV parameters in `config/ads_dv.yaml`, each with a source or marked as an estimate | `fsai/vcu/emulator.py` |
| Referee | Watches the true car, not the stack: cones hit, time off track, lap times, stop distance, FS-AI 2026 D9.1.7 penalties and official times | FS-AI 2026 Rules D4 to D9, IN2 | `fsai/autonomy/sim.py` |

**Known gaps (what the real world adds).** Flat ground only; no rain, dust clouds or sun glare; the camera model makes detections directly rather than from images; tyre temperature is not modelled as a changing state (grip patches and a lower grip stand in for it). See `docs/ASSUMPTIONS.md` A13 to A17 and the Day 1 list in `docs/TEAM_SETUP_GUIDE.md` section 6.

No personal data. No human participants. The "people" in the clutter model are cylinders.
