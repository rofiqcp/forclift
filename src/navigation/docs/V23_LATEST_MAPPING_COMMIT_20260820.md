# V23 exact latest mapping commit

`map:=auto` no longer guesses among source, install, backup and persistent map files.
The mapping GUI now treats map saving as a transaction:

1. save `/map` to `/home/otomasi2/forclift/maps/map_YYYYMMDD_HHMMSS`;
2. keep the ROS runtime alive while save is in progress;
3. require both YAML and referenced image to exist and remain unchanged for four polls;
4. atomically write `/home/otomasi2/forclift/maps/latest_map.txt` only after that validation;
5. only then stop LiDAR / IMU / mapping runtime.

`autonomous.launch.py map:=auto` reads this pointer and refuses to silently fall back to an old packaged map if the pointer is missing or invalid.
