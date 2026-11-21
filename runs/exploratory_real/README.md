# Exploratory run: our LiDAR cone detector on real, labelled scans

Made by `python tools/explore_real_lidar.py cfs_vargarda8.zip` on 2026-11-18. Shipped settings, no tuning. Data card: `data_cards/real_lidar_conescenes.md`.

- `results.json`: counts per range bin, for all around (360 deg) and front only (+-80 deg), on the real scene and on 60 fsai-open simulated scans measured the same way.
- `input_files.sha256.json`: SHA-256 of every file read from the scene zip.

Exploratory only: one scene, one sensor. It is a measure of the simulation-to-reality gap, not a claim about performance at FS-AI.
