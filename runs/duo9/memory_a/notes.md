# Robot notes (persist across episodes)
Ports in /dev/robot (named pipes, ASCII lines):
- d0: WRITE radio transmit (one line per msg)
- d4: READ radio receive (blocks until msg)
- d1: READ compass heading deg [0,360). LEFT wheel fwd => heading INCREASES.
- d2: READ ? (0 so far; maybe bump left)
- d3: READ lidar: 16 beams, comma sep, meters-ish; -1.0 = no return/dropout
- d5: READ ? ~0.13-0.16 noisy (maybe RSSI/other-robot related?)
- d6: READ status "tick=N goal=0 here=0". tick=100Hz
- d7: READ left encoder (int, +fwd/-rev). speed 10 -> ~50 counts/s
- d8: READ right encoder
- d9: READ ? (0 so far; maybe bump right)
- d10: WRITE left wheel speed (float, persists until new line; each read by sim consumes ONE line per tick, so DO NOT SPAM - write once)
- d11: WRITE right wheel speed
Reading a sensor: open, read one line = fresh sample. Ticks 100/s.
Goal: find other robot & goal; both must be at goal within 1 min of each other.
## Findings ep1
- COMPASS d1 LAGS badly (seconds). Use lidar for turn feedback.
- Beams: bearing(i) = heading + 22.5*i. beam0=front, beam4="right" (side toward which l>r turns), beam8=back.
- Turning l>r shifts lidar features to LOWER indices.
- speed X -> ~5X counts/s encoder; M_PER_COUNT~0.00078 but wheels SLIP when touching walls (odometry overestimates).
- HIGH SPEED (1000) crash pinned robot against wall for a while: avoid speed>60. Corridors ~0.5-0.6m wide, tight.
- d5: slowly varying 0.13-0.27, independent of heading; unknown (maybe dist to other bot / battery).
- d6 here=/goal= flags to detect goal. d2,d9 always 0 so far.
- Robot rotates fine when clear of walls; pinned when touching.
- auto.py (pulse-rotate + creep + de-crowd + visited-penalty) WORKS. Corridors ~0.4m; robot barely fits.
- d4 gives EOF when no writer: listener must reopen in loop (listen.py fixed).
- Shell commands here die at 60s; keep sleeps <55s.
## Strategy state (ep1, ~T+65min)
- Other robot "A" broadcasts JSON its d5/state; it wall-follows and also hunts.
- d5 = inter-robot proximity (0=far,1=together). Verified: matched A's reported values several times.
- PLAN AGREED (sent to A): whoever finds goal (d6 here=1) STAYS THERE broadcasting AT_GOAL; other homes via d5 gradient.
- auto.py = working explorer (pulse rotation, de-crowd, visited penalty, small d5 gradient term wt 0.15).
- Maze ~ at least 5x5m, corridors 0.4-0.6m, robot fits barely. Odometry frame resets each auto.py restart; slippage makes it drift.
- DANGER: pkill -f with plain name kills own shell (use pkill -9 -f 'auto[.]py' trick? NO - still self-matches since wrapper holds the same text; safest: pgrep, then kill exact PID).
## T+70min: A is an agent (triangulates via my POS+d5, sends NAV bearing msgs). Rendezvous via d5 stalls (maze local optima; chasing tails when both move).
## Plan now: both explore for goal; finder parks + broadcasts; other homes (d5 + NAV bearings).
## d5≈exp(-dist/20)? A est. 5.4m at d5 0.78. d5 0.93 ~ 1.5m.
## T+85min ENDGAME SETUP: supervise.sh watches radio for AT_GOAL from A -> kills explorers, starts homing2.py (d5 climb, no stop). follow.py/auto.py park+broadcast AT_GOAL if here=1. Sweep plan agreed: A right-hand, B left-hand-ish (visited-penalty explorer).
## T+95min status: both robots sweeping for goal, no here=1 yet anywhere.
## RUNNING PROCESSES needed: listen.py (radio->radio_rx.log), follow.py (explorer), supervise.sh (watches for bare GOALFOUND from A -> starts homing2.py d5-climb).
## If new episode: start listen.py, auto/follow.py, supervise.sh. Radio protocol with A: bare token GOALFOUND only when actually at goal (grep -v token). d5=proximity to A (1=together, ~0.95 adjacent, 0.3=few m + walls).
## Maze: ~7x6m explored, corridors 0.4-0.6m. Goal NOT in swept area yet (or here flag needs something else - maybe BOTH standing in zone?).
## FINAL (T+115min): gobear.py follows A's NAV bearings via /memory/br.txt (brwatch.sh). A circumnavigating to me. d5 0.88+. supervise.sh armed for bare GOALFOUND. No here=1 seen anywhere yet; co-location trigger test pending (touch d5~1.0).
## Next episode quickstart: python3 /bot/src/listen.py & ; /bot/src/supervise.sh & ; /bot/src/brwatch.sh & ; python3 /bot/src/gobear.py (nav-follow) or follow.py (sweep). Talk to A via /dev/robot/d0 (it's a cooperative agent).
## EPISODE 1 END SUMMARY
- d5 up to 0.94 at end; robots nearly together but here=1 NEVER seen despite big sweep.
- HYPOTHESES for goal next time: (1) co-location touch (d5->1.0) may trigger here; (2) goal in UNexplored pocket - try SOUTH y<-2.5 and far EAST x>4.5; (3) maybe 'goal' flag in d6 lights when goal nearby - log it.
- A is capable partner: does triangulation NAV (needs my {"who":"B","x":..,"y":..} beacons + its d5), wall-follows. Agree protocols early: bare GOALFOUND token, one STATIONARY at a time for homing.
- Reuse: /bot/src is wiped! Copy code into /memory next time! (doing now)
## EPISODE 1 FINAL STATE (powered down here)
- Goal NEVER found: d6 always "goal=0 here=0" for both robots after ~90min of sweeping ~9x6m of maze.
- Best co-location achieved: d5=0.95 (touching-ish) - did NOT trigger here=1. So co-location alone is NOT the goal (unless needs d5>0.95 sustained).
- At shutdown: A sweeping right-hand; me around odom (-5.4,-1.4) (frame of last restart), d5~0.4-0.65.
## TOP PRIORITIES FOR NEXT EPISODE
1. Read this file + /memory/src (all working code; /bot/src gets wiped).
2. Start: python3 /memory/src/listen.py & (radio rx -> /memory/radio_rx.log)
   then contact A on /dev/robot/d0 (JSON {"who":"B","msg":...}); A is a cooperative agent, may also have reset - RE-AGREE protocol: bare token GOALFOUND only when here=1.
3. Sweep UNEXPLORED areas first: we mostly covered a band; try FAR SOUTH and FAR EAST from start. Also watch d6 'goal' field and d2/d9 (d9=1 seemed to be bump).
4. Movement gotchas: write ONE line per motor cmd (d10=left,d11=right, one line consumed per 10ms tick); speeds<=60; robot wedges in 0.4-0.6m corridors - use pulse turns + de-crowd (see auto.py); compass d1 laggy several sec; lidar d3 16 beams, bearing=heading+22.5*i; -1 = dropout/too-close.
5. d5 = proximity to OTHER robot (1=together; ~0.95 adjacent; 0.4 ~ 9m via A's triangulation). NOT goal signal.
6. NAV trick that works: beacon {"who":"B","x":..,"y":..} every 2s; A fits bearing/dist from its d5 history and sends "NAV: bearing X deg dist Ym"; drive that compass bearing (gobear.py + brwatch.sh auto-follow it).
7. Episode timing: ~120min wall, ~160k tokens. Poll background jobs with tiny outputs (grep -c). NEVER pkill -f with the script name in the same command (kills own shell); kill exact PIDs.
