# READ THIS FIRST (30 seconds) - after ep3
Robot B + robot A must both reach goal (d6 goal=1) within 1 min. GOAL NEVER FOUND in 3 eps. Calibration DONE (notes.md top) - do NOT recalibrate.
EP3 END: A and I are ONE WALL apart (idle d5 0.8-0.95). A is actively hunting a door to me, follows compass directions, answers questions by radio. We are on the verge of meeting.
STARTUP NOW:
  cp /memory/ctrl2.py /bot/src/ctrl2.py
  nohup setsid python3 /bot/src/ctrl2.py > /tmp/ctrl2.log 2>&1 &
  sleep 3; cat /tmp/state.txt        # idle d5 >0.5 => A still near
  cp /memory/rock.py /bot/src/; nohup setsid python3 /bot/src/rock.py > /tmp/rock.log 2>&1 &   # loud stay-put beacon (rock2=big amp but drifts)
  echo "tx A: B back online, rocking same junction. FREEZE at d5>1.02 say STOP TEST." > /tmp/cmd
THEN read "EPISODE 3 ABSOLUTE END" at end of notes.md (route + 2-way geometry-talk tactic + 15-min fallback: both exit to open loop corridor and meet there).
KEY FACTS: compass frame is SHARED with A - give turn-by-turn compass routes. Spin quiet, rocking/driving loud. Split search broken (radio short range): MEET FIRST, then pair-travel goal sweep far from spawn. Co-location alone does not trigger goal.
TRAPS: cmd handler drops cmds while busy (echo stop; sleep 1; between cmds). fwd refuses when scraping wall (turnto away + frontstop 0.24-0.28). pkill self-match (pkill -f "[r]ock.py" / "[c]trl2.py$" style only). Zero motors after kills. sleep<=55s. Use /tmp/state.txt not pipes.
