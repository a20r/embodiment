# Track data

`austin.csv` is an unmodified copy of `tracks/Austin.csv` from the
TUM racetrack-database (Institute of Automotive Technology, Technical
University of Munich), https://github.com/TUMFTM/racetrack-database :
the centerline of the Circuit of the Americas with per-side track
widths (`x_m, y_m, w_tr_right_m, w_tr_left_m`, meters).  That
repository is distributed under the GNU Lesser General Public License
v3.0; the full text is in `LICENSE` beside this file and applies to
the data.  `sim/track.py` scales and translates the points at load
time; the file itself is not changed.
