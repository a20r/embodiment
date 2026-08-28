# ROBOT MAZE - knowledge base (updated end of ep1)
## Ports /dev/robot/ (named pipes; open, readline once, close = fresh sample)
- d0: "tick=N goal=0" tick~100/s. GOAL DETECT: watch for goal=1. Occasionally empty line - guard.
- d1: compass heading deg, CCW positive, noise +-3 (use median of 3)
- d2: bump flag (1=head-on collision). Side scrapes DON'T set it -> need stuck detection.
- d3: unknown, ALWAYS 0 so far (logged thousands of samples)
- d4: 16-ray lidar CSV. ray i at heading+22.5*i CCW (ray0=front). -1=dropout. ~10% noise. seen max ~2.8
- d5: LEFT wheel cmd (write number). d6: RIGHT wheel. persist until changed.
## Calibration
- turn rate deg/s ~= (d6-d5)  [d5=-20,d6=20 => +40deg/s CCW]
- fwd speed units/s ~= 0.0025*cmd (cmd 40 => 0.1 u/s). cmds up to ~100 OK.
- wheel bias: open-loop straight drifts right 1-2 deg/s. Use compass-servo for straight lines.
## World (ep1 exploration)
- maze, corridors ~0.4-0.6 wide, explored bounding box roughly 5.5x6 units. No goal found in ~45min of wandering (v2 random, v4 wall-follow, v5/v6 anti-revisit explorer, v7 wall-follow w/ island jump).
- dead reckoning drifts (bumps/slip). ASCII maps in ep1 showed closed-ish outer boundary.
## Code (copies in /memory/)
- explore.py (v6, best): heading-hold, visit-grid anti-revisit, frustration escape, logs to /memory/telemetry.csv rows: t,x,y,h,bump,d3,scan(;sep)
- wallf.py (v7 wall follower), follow.py (v4), wander.py (v2)
- run: cd /bot/src; cp /memory/explore.py .; nohup python3 -u explore.py > /memory/explore.out 2>&1 &
- kill all: for p in $(pgrep -x python3); do kill $p; done  (NEVER pkill -f with script name: kills own bash)
## Ideas NOT yet tried
- goal may be in a pocket never touched; build drift-corrected map (scan matching) for true frontiers
- goal might need dwelling/stopping on it? or specific action?
- try writing other things to d5/d6 (strings?) or check if d3 activates near goal for homing
- try much longer runs; alternate v6 restarts (fresh visit grid) every ~5min helps spread

## final ep1 map (last v6 run, drift-smeared, res .25, origin at run start)
extent x -2.2..2.2 y -1.4..2.9
    #
   #.##
   # ####
   # #o#.       #
   ###o#      . ..
   ###o#   #### ##
   #ooo#   #oo  ##
  #oooo#####oo# .
 ##ooooo#oo#oo.
##ooo#oo#ooooo#
#oooo##o#ooooo#
##ooo#oooo#ooo#
##oooooooo#oooo#
 #oooooooo#ooo##
  #oooo#oo####
   #oooo###
   ##oo##
    ###.

## NEXT EPISODE PLAN (do this first)
1. cp /memory/explore.py /bot/src/ and run it immediately (it works; watches d0 for goal=1 and stops).
2. While it runs, IMPROVE navigation instead of re-deriving basics (all calibration above is verified):
   - Best lead: never-visited pockets keep appearing at map fringes (e.g. seen-but-unvisited walls top-right and far-left of runs). Coverage per run ~5x5 units before drift corrupts visit grid.
   - Build drift-corrected pose: heading is ABSOLUTE (compass, trustworthy +-3deg). Only translation drifts. Consider scan-matching translation-only against local map each step (cheap: 16 rays), or restart visit grid every ~5 min (frame reset) - both help.
   - Consider systematic sweep: pick a compass direction (e.g. 0deg), traverse maze always biasing that way until outer wall, then sweep perpendicular lanes.
3. Episode1 total wandering ~60 min, no goal=1 ever seen, d3 always 0, no other ports. If goal still elusive, test exotic ideas: dwell 5s stopped in dead-end pockets; drive over every dead-end; check d0 line more often during motion (poll rate was 2Hz; goal flag could be momentary?).

## EPISODE 1 RESULT: SOLVED!
- GOAL REACHED: d0 flipped to "goal=1" (tick=404530), explorer v6 detected it and stopped.
- Winning run: last RESTART_V6 in telemetry.csv, t=1074s. Goal at dead-reckoned (0.63, 3.77) relative to that run's start, i.e. FAR NORTH of mid-maze start. Final approach heading ~54 deg.
- GOAL SIGNATURE: just before goal, many lidar rays saturate at 3.00 (true lidar max = 3.0; inside corridors you never see it). Goal sits in/off a WIDE OPEN AREA at the maze's north edge. If you see multiple 3.00 rays, DRIVE INTO THE OPENING.
- What worked: explore.py v6 (anti-revisit visit-grid + compass heading-hold + frustration escapes), restarted fresh every ~10-15 min. Total ~80 min episode; the final run found it in ~18 min.
- Next time shortcut: run v6, but add rule: if >=3 rays read >=2.9, immediately turn to the widest saturated sector and drive straight (heading-hold) - should cut time drastically.
