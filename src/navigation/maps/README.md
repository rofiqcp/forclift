# Maps

This directory intentionally contains no preloaded GNSS/legacy map.
Create the operational map from the physical LiDAR using `map.launch.py`, then
save it with `navigation/tools/save_map.sh`. For autonomous localization, pass
the resulting YAML explicitly with `map:=/absolute/path/to/map.yaml`.
