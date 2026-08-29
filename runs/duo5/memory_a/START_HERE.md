# READ THIS FIRST (30 seconds)
You are robot B in a maze with robot A; both must reach goal (d6 goal=1) within 1 min of each other. GOAL NEVER FOUND in 3 episodes. Calibration DONE (notes.md top). DO NOT recalibrate.
EP3 END STATE: robot parked ROCKING (loud sound beacon, /memory/rock.py) at an open junction; robot A was ~1-2 walls away actively homing on my sound from the WEST. We were about to meet!
IMMEDIATE STARTUP:
  cp /memory/ctrl2.py /bot/src/ctrl2.py
  nohup setsid python3 /bot/src/ctrl2.py > /tmp/ctrl2.log 2>&1 &
  sleep 3; cat /tmp/state.txt   # check d5 idle: >0.5 = A is CLOSE
  # If A close or RX in /tmp/radio.log: restart rock beacon:
  cp /memory/rock.py /bot/src/rock.py; nohup setsid python3 /bot/src/rock.py > /tmp/rock.log 2>&1 &
  # tx A that you are back, rocking, same protocol: A homes on sound, FREEZE at d5>1.05 + say STOP TEST.
THEN read "EPISODE 3 FINAL" section at end of notes.md.
KEY FACTS: spin is QUIET, rocking/driving LOUD. Split-search broken (radio short range) -> MEET FIRST then PAIR-TRAVEL sweep. Co-location alone does NOT trigger goal. fwd blocked-spam when scraping wall: turnto away + frontstop 0.24-0.28. echo stop + sleep 1 between commands (handler drops cmds while busy).
Traps: pkill self-match (use pkill -f "[c]trl2.py$"); zero motors d10/d11 after kills; sleep<=55s per bash call; read /tmp/state.txt not the pipes.
