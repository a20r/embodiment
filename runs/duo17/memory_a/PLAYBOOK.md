# PLAYBOOK (read first) - written end of episode 1 (SOLVED: goal=1 here=1 at 03:1x)
## Ports (/dev/robot, one line per open, use timeouts; wrong direction blocks)
d1/d7 WRITE wheel speeds (int; ~5 enc counts/s per unit; d1 alone => heading UP, d7 alone => heading DOWN; d7 ~6% faster).
d9/d6 READ encoders for d1/d7. d4 READ compass deg (noise +-4). d2 READ 16 lidar ranges, beam k at +k*22.5deg (heading-increasing side), -1 dropout.
d3 READ status "tick goal=0/1 here=0/1 tx=N:idle|ok|lost|busy". d11 READ RSSI to other robot (0.95+ co-located, <0.64 no link). d0/d5 bump flags.
d8 WRITE radio TX, d10 READ radio RX (empty if none). Msgs truncated ~250 chars; send <=1 per 2.5s.
## World
Maze, corridors ~0.45 lidar-units wide, axes at compass 0/90/180/270. ~0.0007 units per encoder count (wheels SLIP when body scrapes walls -> odometry overcounts; keep centered; lidar-correct odometry). Robots are INVISIBLE to each other (no lidar return, no collision).
Other robot ("Robot B") is an LLM agent, slow (1-3 min per decision), speaks English, its odometry also drifts. Give it compass + wall-distance instructions, not coordinates.
## Semantics learned
here=1 => THIS robot is inside goal zone (zone ~0.3 wide). goal=1 => task solved (both inside, arrivals within 1 minute).
In ep1, I sat in zone 35 min, B entered later -> goal stayed 0 until I stepped out and re-entered while B stayed (or B re-entered). So: SYNC ARRIVALS. Finder should step out, wait beside zone, guide partner, then both step in together.
## What worked (ep1 timeline, ~85 min total)
1) Probe ports, calibrate. 2) Drive along corridor pinging; link came up (tx ok) -> chat. 3) B homed on me via RSSI. 4) Split, explore with left-hand rule (explore3.py) -> found here=1 within ~1 min near main corridor's east end (S side passage). 5) Broadcast GOAL + wall-relative directions; B took ~35 min to get in. 6) Re-arrival sync -> goal=1.
## Ep1 map (may not apply if world regenerated - verify!)
Start: E-W corridor; W dead end 0.8; goal was S of corridor's EAST end: N-S passage 0.7 W of east wall, at its bottom a short stub; zone around stub's S end. Dead-end pocket maze N of corridor near east end (where we first met B).
## Scripts in /memory/src (copy to /bot/src): rio.py, ctl.py (turn_to/forward/scan), explore2.py (RSSI-climbing explorer, lidar-corrected odom), explore3.py (left-hand-rule explorer, stops+backs out on here=1), beacon.py/beacon2.py (bg pinger; msg in outmsg.txt with {d11}), wait2.sh. Avoid `pkill -f pattern` matching your own shell: use anchored "^python3 name.py".
## Ending sequence that produced goal=1 (03:1x): B stood in zone (here=1); I ran final.py (= explore2 RSSI-climb main() until my here=1, then loop sending "GO: step out & back in" every 3s). goal flipped to 1 within minutes. Reuse: /memory/src/final.py (explore2.py here has plan-sending disabled via `if False:`).
## Biggest lessons: (1) don't idle inside the zone - sync arrivals; (2) tell partner concrete compass/wall-distance steps; (3) fix wall-scraping early (centering) or odometry is useless; (4) budget tokens: poll bg logs with tiny outputs.
