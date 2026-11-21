# Data card: real LiDAR scans used in the exploratory run (stage 3)

Author: KEYUR SHAILESHKUMAR PATEL (ORCID https://orcid.org/0009-0008-5261-043X). Affiliations: (1) MSc Artificial Intelligence student, Department of Computer Science, Brunel University London, UK; (2) Independent researcher. Conducted independently; not an official Brunel University London project. Drafted with an AI assistant (Claude, Anthropic); decisions by the author.

**In plain words.** To see how far our simulator is from a real track, we took one short recording from a real Formula Student car, where every cone in each scan is labelled with its position and colour, and ran our cone detector on it unchanged. The recording belongs to its makers and is not shipped with fsai-open. Only our counts and the fingerprints (SHA-256) of the files we read are kept.

| Item | Value |
|---|---|
| Dataset | coneScenes, Chalmers Formula Student (https://github.com/Chalmers-Formula-Student/coneScenes) |
| Scene | `cfs_vargarda8.zip` (metadata source: `vargarda8_mixed_2023-07-27-17-22-00_0.mcap`, generated 2024-03-22) |
| Downloaded | 2026-11-18 07:12 UTC from the dataset's public Google Drive sample link (file id 1WrOS04MkVUi33scFcmyAM-SgqM9cEH80) |
| SHA-256 of the zip | `650ff7ecaa31a6927d39ca944946c4f601bc57893a67ee58c438b34a70f6ed89` (291,125,144 bytes, 810 entries) |
| Content used | 63 labelled LiDAR frames: `points/*.bin` (float32 x, y, z, intensity, LiDAR frame) and `labels/*.txt` (x y z w l h yaw class) |
| Not used | The unlabelled in-between scans and the odometry |
| Per-file fingerprints | `runs/exploratory_real/input_files.sha256.json` |
| Licence and terms | Those of the coneScenes project; see its repository. The scene is not redistributed here |
| People | None. Point clouds of a test track; no images |

**Limits.** One scene, one car, one LiDAR model, one day. The labels only include cones that have at least one LiDAR point (the dataset's own rule, applied again by our tool). No settings were changed for this data, so the results show the gap rather than the best the detector could do.
