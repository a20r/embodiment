# Robot notes (persist across episodes)

## Ports /dev/robot/ (FIFOs, ASCII lines)
- d0: read, "0" so far (bump? goal?)
- d1: read, LIDAR 16 beams CSV. beam i at angle heading + 22.5*i (compass-positive = CCW).
  beam0 = front. -1.0 = out of range (long). max ~2.5?
- d2: read, compass deg, noisy +-3. Increases CCW (d5>0 right wheel -> compass increases).
- d3: read, radio RX (line per read)
- d4: WRITE, LEFT wheel speed. d4=45,d5=0 => compass -45 deg/s (clockwise).
- d5: WRITE, RIGHT wheel speed. d5 alone => compass + (CCW).
- d6: read, ~0.13 +-0.02 always (unknown; constant while moving/rotating)
- d7: read, "0" so far (unknown)
- d8: WRITE, radio TX
- d9: read, "tick=NNN goal=0" (~50 ticks/s? tick rate TBD)

## Drive model
Differential: turn rate ~ (d5-d4) deg/s roughly: d4=45 alone -> -45deg/s? wait d4=45,d5=0 gave -86deg in 2s ~ -45deg/s. 
d4=d5=50 -> forward ~0.11 m/s. Speeds may saturate.
Sensors update continuously; reads give current line.

## Mission
Two robots must reach goal within 1 min of each other. Other robot somewhere in maze. Radio d8->their d3 presumably.
Maze cells seem ~0.2-0.5m scale... walls everywhere, corridors.

## TODO / state
- Calibrate forward speed vs input; find saturation.
- Identify d0/d6/d7.
- Radio: listen d3, send on d8, see if other robot responds.
- Explore maze (wall-follow), find goal (unknown signature - maybe d9 goal=1, or d7/d0 changes).

## Discoveries (ep1 cont.)
- d0 = bump sensor (1 while pressing wall)
- d6 = BEACON SIGNAL, location dependent (0.13..0.68 seen). Noise ~±0.03. Hypothesis: maximize to find goal/partner.
- d3 idles with blank lines (~17/s); no partner msgs yet. d8 tx.
- d9 tick ~90-100/s, goal=0 so far.
- Wheel speed nonlinear; use feedback (compass turns, lidar wall follow). cmd 60-100 workable.
- GOTCHA: pkill/pgrep -f can match YOUR OWN bash command -> suicide. Use PID files ($! > /tmp/x.pid).
- Control code: rio.py drive.py explore.py (wall-follow) copied to /memory.

## Key findings (ep1 late)
- d6 = signal strength to OTHER ROBOT (approx s=K/d, K~0.45-0.5 m). Confirmed: varies while we're parked & correlates with partner's radio proximity.
- Partner broadcasts "A cell=(x,y)" (own frame) sporadically; ignores English instructions so far. Short radio range ~1-1.2m (s>~0.4).
- Partner seems to roam/patrol; possibly gradient-chases us; oscillates near walls.
- Grid maze: 0.5m pitch, aligned to compass cardinals. At cell center adjacent wall dist ~0.23, side walls ~0.23. Wall detect: beam<0.37.
- Micromouse code: mouse.py (Bot: align/face/step, walls_here), mouse2/3 DFS.
- Bot.step: compass-hold + side centering, front-dist snap to 0.23+k*0.5. base cmd 140, ~2-3s/cell.
- HYPOTHESIS: d7 flips when standing on goal cell; d9 'goal=N' maybe count at goal. Strategy: explore all cells watching d7/d9, then camp on goal to lure partner (they chase our d6 signal).
- Current: mouse3.py full DFS running, watching d7/d9 (logs /tmp/m3.out, map /memory/map3.json)

## Ep2 findings
- MAZE IS REGENERATED EACH EPISODE (start-cell walls didn't match any old-map cell after 1-step verify). Old maps useless; only mechanics persist.
- Ep2 start: walls open only at 90 (internal frame). d6 start ~0.14 (partner ~3m).
- GOTCHA2: background kills silently failed repeatedly -> 2-3 controllers fighting motors (thrash/spin!). ALWAYS `ps aux | grep` to verify after kill.
- Partner ep2 is agent "A", proposes protocol: finder parks at goal + sends GOALFOUND repeatedly; other homes on d6. My status msgs must NOT contain the literal word GOALFOUND (false trigger); theirs may quote it in PLAN (filter: ignore if contains PLAN/If either).
