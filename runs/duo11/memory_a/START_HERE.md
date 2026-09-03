# READ ME FIRST (written end of episode 1)
You are a robot in a maze with another robot "A" that MOVES (patrols).
Goal: both robots must reach the (unknown) goal location within 1 min of each other.
d3 status "tick=.. goal=0 here=0" — flags never changed in ep1. Watch for here=1.

## Quick start
1. bash /memory/rxlog.sh &        (logs radio to /memory/rx.log; copy from /memory)
2. cd /bot/src; cp /memory/rob2.py /memory/auto.py . ; nohup python3 auto.py 3000 &
   auto.py: roams maze; chases RSSI gradient (d11) toward robot A; pings radio;
   logs to /memory/auto.log; watches status flags.

## Everything learned is in NOTES.md. Key facts:
- Motors: d1(v),d7(w) mixed. FORWARD=(v=16,w=20)~0.19m/s, hold heading via v=16+1.2*err
  (err=norm(H-heading)), TURN: v=0,w=+-8..25 (positive w DECREASES compass heading d4).
- Lidar d2: 16 beams, beam i at compass (heading + 22.5*i). front=beam0. max ~2.7.
- d11 = RSSI to robot A (0=far, ~0.5=within ~1 cell). A PATROLS - signal comes and goes.
- Radio: d8 tx, d10 rx. Range VERY short (~sig>0.45). Got exactly one msg:
  "PING A x=-0.60 y=-1.82" (A broadcasts its pose in its own frame).
- Never got a reply from A. Unknown if A reacts to messages. TRY: when sig>0.4,
  spam "PING B x=.. y=..", "GOAL?", etc and log rx.
- DANGER: pkill -f pattern matches YOUR OWN shell cmdline -> use [b]racket trick.
- Robot gets wedged; unstick by reversing (v,w)=(20,16) ~2s.
## Open questions / next steps
- How is goal revealed? Maybe when robots meet (radio handshake) A shares goal coords,
  or maybe 'here' flag flips somewhere. Explore systematically & log status.
- Consider building an actual occupancy map + tracking pose to make search efficient.
- A's patrol: passes come every ~1-3 min in some areas. Chase when sig rising.

## End-of-ep1 state & best strategy so far
- hunt.py (in /memory) = latest & best: roams; if sig(d11)>0.08 parks and does
  lidar-motion-detect intercept (dash at moving object) with radio pinging. Logs /memory/hunt.log.
- Chasing gradient alone FAILS: A moves ~2x our probe speed. Interception/ambush is needed.
- A passed near us twice while we sat still (sig 0.03->0.29->0.12 over ~90s). Radio
  needs sig>~0.45. Lidar should see A at <1.5m: look for beams suddenly SHORTER than baseline.
- IDEA not yet tried: build coarse odometry map of junctions; record where sig peaks
  happen over time; A's patrol route may be fixed loop -> ambush on route.
- IDEA: our episode START cell had sig~0.5 (A was close at spawn). A's patrol includes
  spawn area. If lost, returning toward spawn may re-acquire signal.
- Roam speed: each step() ~4.5s ~0.8m. turn_to ~2-5s. A full cell transit ~7-10s.

## LATE-EPISODE-1 CRITICAL OBSERVATIONS
- A's pings normally include its pose: "PING A x=.. y=..". Near end we received BARE
  "PING A" (no coords) several times, even at sig~0.12 (farther than usual radio range?).
  Meaning unknown. Possibly A entered a new mode AFTER hearing our "PING B x=..." msgs.
- A's patrol (its frame): along y~17.2 x21->24, corner (23.3,17), down to y~12.7 at x~24.4.
  It STOPS sometimes (~min). Loop period ~110s when moving.
- Our hot spot (sig 0.75-0.84, radio works): we sat there while A passed; our position
  in A's frame ~ (23.8, 15.9).
- STRATEGY left running (final.py): seek sig>0.5 then park; always transmit
  "PING B x=23.80 y=15.90" (claim = hot corner posn in A frame). Hypothesis: A may
  rendezvous at coordinates we claim. UNVERIFIED.
- If next episode: FIRST try sitting still ~3 min broadcasting "PING B x=<own pos in A frame>"
  when sig high, and log whether A stops near us. Also test bare "PING A"-style reply.
- here/goal flags in d3 NEVER flipped in ep1 anywhere we visited.
