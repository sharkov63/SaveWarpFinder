# SaveWarpFinder

A Python script for finding warp sequences in Half-Life.

For Half-Life 2 Old Engine configurations, check `hl2oe` branch.

### How to use
1. Dump all your potential starting positions for warps (*waypoints*) into `waypoint.txt` file. Waypoints shall be described in the following format: `<map> | <x> <y> <z>`. For making waypoints, use `swarp_wp_` commands from SWT:
- `swarp_wp_add` creates a new waypoint in the player's current position on current map and adds it to the list.
- `swarp_wp_dump` dumps all waypoints in the list into console in the above format.
- `swarp_wp_clear` clears the list.
2. In `builder.py` change `DESTINATION` to the wanted final position written in this format: `<map> at: <x> x, <y> y, <z> z`, which can be just copied from the `status` command.
3. Change `WAYPOINT_DISTANCE` to the maximum allowed distance between the actual and the wanted destination point.
4. Change `WARP_START_MAP` to the first map of the game.
5. Specify which triggers cannot be activated by the player in `BANNED_TRANSITIONS` list. Each trigger here is a list that looks like this: `[<map_from>, <landmark>, <map_to>]`.
6. Specify which maps allow the player to freely move out of bounds in `VOID_MAPS` list. In original Half-Life, there are 5 such maps:
*c1a4d*,
*c1a4e*,
*c1a4g*,
*c1a4j*,
*c2a4*.
7. Finally, save and launch `builder.py`. As a new warp chain is found, it will be dumped into `stdout`.
