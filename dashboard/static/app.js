/* Mazebot dashboard — vanilla JS, no build step. */
"use strict";

const $ = (s) => document.querySelector(s);
const api = async (path) => {
  const r = await fetch(path);
  if (!r.ok) throw new Error(`${path}: ${r.status}`);
  return r.json();
};
const css = (name) =>
  getComputedStyle(document.documentElement).getPropertyValue(name).trim();

const S = {
  series: null, ep: null, running: false,
  maze: null, timers: [], transcriptCursor: 0,
  liveTrail: [], lastState: null,
  skids: [],   // live car: [x, y, theta] at every polled sliding pose
  replay: { poses: [], events: [], idx: 0, playing: false },
  memorySel: null,
};

const fmtLap = (s) => s == null ? "—"
  : `${Math.floor(s / 60)}:${(s % 60).toFixed(1).padStart(4, "0")}`;

function clearTimers() {
  S.timers.forEach(clearInterval);
  S.timers = [];
}

/* ---------------- series / episode selection ---------------- */

async function loadSeries() {
  const list = await api("/api/series");
  const sel = $("#series-sel");
  sel.innerHTML = "";
  for (const s of list) {
    const o = document.createElement("option");
    o.value = s.name;
    o.textContent = `${s.name} (${s.episodes} ep, arm ${s.arm ?? "?"})`;
    sel.appendChild(o);
  }
  if (list.length) {
    sel.value = list[list.length - 1].name;
    await selectSeries(sel.value);
  } else {
    $("#tab-transcript").innerHTML =
      '<div class="empty">no runs yet — start one with ./botctl run</div>';
  }
}

async function selectSeries(name) {
  S.series = name;
  const eps = await api(`/api/episodes?series=${encodeURIComponent(name)}`);
  const sel = $("#episode-sel");
  sel.innerHTML = "";
  for (const e of eps) {
    const o = document.createElement("option");
    o.value = e.episode;
    o.dataset.running = e.running ? "1" : "";
    const tag = e.running ? "running"
      : e.solved ? "solved" : (e.end_reason || "DNF");
    o.textContent = `ep ${e.episode} — ${tag}`;
    sel.appendChild(o);
  }
  if (eps.length) {
    const last = eps[eps.length - 1];
    sel.value = last.episode;
    await selectEpisode(last.episode, last.running);
  }
}

async function selectEpisode(ep, running) {
  clearTimers();
  S.ep = ep; S.running = !!running;
  S.liveTrail = []; S.skids = []; S.transcriptCursor = 0; S.lastState = null;
  S.replay = { poses: [], events: [], idx: 0, playing: false };
  if (window.view3dReset) window.view3dReset();
  $("#tab-transcript").innerHTML = "";
  $("#memory-view").innerHTML = '<div class="empty">select a file</div>';
  const q = `series=${encodeURIComponent(S.series)}&ep=${ep}`;
  try {
    S.maze = await api(`/api/maze?${q}`);
  } catch {
    try { S.maze = await api("/api/live/maze"); } catch { S.maze = null; }
  }
  const eps = await api(`/api/episodes?series=${encodeURIComponent(S.series)}`);
  const mine = eps.find((e) => e.episode === Number(ep)) || {};
  $("#ep-summary").textContent = mine.running ? "in progress"
    : `${mine.solved ? "solved" : "unsolved"} · ${mine.end_reason ?? ""}`
      + (mine.goal_tick ? ` · tick ${mine.goal_tick}` : "")
      + (mine.best_lap_s != null
          ? ` · ${(mine.laps || []).length} timed laps · best ` +
            fmtLap(mine.best_lap_s) : "");

  if (S.running) {
    $("#live-badge").textContent = "live";
    $("#live-badge").classList.add("live");
    $("#sim-controls").style.display = "";
    $("#replay-controls").style.display = "none";
    S.timers.push(setInterval(pollLive, 250));
    S.timers.push(setInterval(pollTranscript, 1200));
    pollLive(); pollTranscript();
  } else {
    $("#live-badge").textContent = "replay";
    $("#live-badge").classList.remove("live");
    $("#sim-controls").style.display = "none";
    $("#replay-controls").style.display = "flex";
    await loadReplay();
    await pollTranscript(true);
  }
  refreshActiveTab();
}

/* ---------------- live polling ---------------- */

async function pollLive() {
  let st;
  try {
    const since = S.liveTrail.length
      ? S.liveTrail[S.liveTrail.length - 1][0] : 0;
    // The point cloud is only cast when the 3D view is showing.
    const cloud = window.view3dActive && window.view3dActive() ? 1 : 0;
    st = await api(`/api/live/state?since=${since}&cloud=${cloud}`);
  } catch {
    $("#live-badge").textContent = "live (daemon unreachable)";
    return;
  }
  $("#live-badge").textContent = st.paused ? "live · paused" : "live";
  for (const p of st.trail || []) S.liveTrail.push(p);
  if (S.liveTrail.length > 30000) S.liveTrail.splice(0, 10000);
  S.lastState = st;
  if (!$("#rtf-input").value) $("#rtf-input").value = st.realtime_factor;
  // Only the car model reports `slipping`; phi_deg is always present.
  const car = st.slipping !== undefined;
  if (car && st.slipping && st.pose) {
    S.skids.push([st.pose[0], st.pose[1], st.pose[2]]);
    if (S.skids.length > 8000) S.skids.splice(0, 2000);
  }
  drawState({
    pose: st.pose, trail: S.liveTrail, colliding: st.colliding,
    bump: st.bump, rays: st.lidar_true, rayAngles: st.ray_angles,
    keyTaken: st.key_carried, doorOpen: st.door_open,
    car, phi: st.phi_deg, slipping: st.slipping, skids: S.skids,
  });
  if (window.view3dUpdate) window.view3dUpdate(st);
  const lapTxt = st.lap === undefined ? "" :
    ` · lap ${st.lap} · last ${fmtLap(st.last_lap_s)}` +
    ` · best ${fmtLap(st.best_lap_s)}` +
    (st.slipping ? " · SLIDING" : "");
  $("#statusline").textContent =
    `tick ${st.tick} · sim ${st.sim_time_s}s · rtf ${st.realtime_factor}` +
    ` · cmd [${st.cmd}] · enc [${st.enc}] · collisions ${st.collision_count}` +
    lapTxt + ` · goal ${st.goal_reached ? "REACHED" : "—"}`;
}

/* ---------------- replay ---------------- */

async function loadReplay() {
  const q = `series=${encodeURIComponent(S.series)}&ep=${S.ep}`;
  try {
    const gt = await api(`/api/gt_trail?${q}`);
    S.replay.poses = gt.poses; S.replay.events = gt.events;
  } catch { S.replay.poses = []; }
  const n = S.replay.poses.length;
  $("#scrub").max = Math.max(0, n - 1);
  $("#scrub").value = Math.max(0, n - 1);
  S.replay.idx = Math.max(0, n - 1);
  drawReplay();
}

function drawReplay() {
  const R = S.replay;
  if (!R.poses.length) { drawState({}); return; }
  const i = Math.min(R.idx, R.poses.length - 1);
  const p = R.poses[i];
  const [t, x, y, th, col] = p;
  const evTick = (kind) => {
    const e = R.events.find((ev) => ev.event === kind);
    return e ? e.t : null;
  };
  const pickupT = evTick("key_pickup"), unlockT = evTick("door_unlocked");
  // gt_trail poses of a car carry [phi_deg, slip] at indices 5, 6.
  const car = p.length > 5;
  const upto = R.poses.slice(0, i + 1);
  drawState({
    pose: [x, y, th], colliding: !!col,
    trail: upto.map((q) => [q[0], q[1], q[2]]),
    keyTaken: pickupT !== null && t >= pickupT,
    doorOpen: unlockT !== null && t >= unlockT,
    car, phi: p[5], slipping: !!p[6],
    skids: car ? upto.filter((q) => q[6]).map((q) => [q[1], q[2], q[3]])
               : null,
  });
  if (window.view3dUpdate) window.view3dUpdate({ pose: [x, y, th] });
  const dt = t / 50.0;
  const laps = R.events.filter((e) => e.event === "lap" && e.t <= t);
  $("#replay-time").textContent = `tick ${t} · ${dt.toFixed(1)}s` +
    (laps.length ? ` · lap ${laps.length}` : "");
  $("#statusline").textContent =
    `replay · ${R.poses.length} samples · events: ` +
    R.events.map((e) => e.event === "lap"
      ? `lap${e.lap}${e.timed ? "" : "(warm)"}=${fmtLap(e.time_s)}@${e.t}`
      : `${e.event}@${e.t}`).slice(0, 12).join(", ");
}

$("#scrub").addEventListener("input", (e) => {
  S.replay.idx = Number(e.target.value);
  S.replay.playing = false; $("#btn-play").textContent = "play";
  drawReplay();
});
$("#btn-play").addEventListener("click", () => {
  const R = S.replay;
  R.playing = !R.playing;
  $("#btn-play").textContent = R.playing ? "pause" : "play";
  if (R.playing) {
    if (R.idx >= R.poses.length - 1) R.idx = 0;
    const step = Math.max(1, Math.round(R.poses.length / 600));
    const tick = () => {
      if (!R.playing) return;
      R.idx = Math.min(R.idx + step, R.poses.length - 1);
      $("#scrub").value = R.idx;
      drawReplay();
      if (R.idx >= R.poses.length - 1) {
        R.playing = false; $("#btn-play").textContent = "play"; return;
      }
      requestAnimationFrame(tick);
    };
    requestAnimationFrame(tick);
  }
});

/* ---------------- canvas ---------------- */

// Asphalt has no token in style.css; pick a grey that reads against
// either surface.
const asphalt = () =>
  matchMedia("(prefers-color-scheme: dark)").matches ? "#3a3a37" : "#bdbbb3";

function loopPath(ctx, X, Y, pts) {
  ctx.moveTo(X(pts[0][0]), Y(pts[0][1]));
  for (let i = 1; i < pts.length; i++) ctx.lineTo(X(pts[i][0]), Y(pts[i][1]));
  ctx.closePath();
}

// Circuit: asphalt between the edge loops, both edges (the barriers),
// a checkered start/finish bar between the two edge points.
function drawTrackScene(ctx, m, X, Y, scale) {
  ctx.beginPath();
  loopPath(ctx, X, Y, m.left); loopPath(ctx, X, Y, m.right);
  ctx.fillStyle = asphalt();
  ctx.fill("evenodd");
  ctx.strokeStyle = css("--text-primary");
  ctx.lineWidth = Math.max(1.2, 0.03 * scale); ctx.lineJoin = "round";
  ctx.beginPath(); loopPath(ctx, X, Y, m.left); ctx.stroke();
  ctx.beginPath(); loopPath(ctx, X, Y, m.right); ctx.stroke();
  const [[lx, ly], [rx, ry]] = m.start_line;
  const len = Math.hypot(rx - lx, ry - ly);
  const n = Math.max(2, Math.min(8, Math.floor(len * scale / 3)));
  const cw = len * scale / n, rh = Math.max(3, cw);
  ctx.save();
  ctx.translate(X((lx + rx) / 2), Y((ly + ry) / 2));
  ctx.rotate(-Math.atan2(ry - ly, rx - lx));
  for (let r = 0; r < 2; r++) for (let i = 0; i < n; i++) {
    ctx.fillStyle = (i + r) % 2 ? css("--text-primary") : css("--surface-1");
    ctx.fillRect(-len * scale / 2 + i * cw, -rh + r * rh, cw + 0.5, rh);
  }
  ctx.restore();
}

// Race car: 0.40 x 0.18 m body with a pointed nose, four wheels (front
// pair steered by phi), cockpit dot.  The sprite scale floors at
// 40 px/m so the car stays a legible marker on a 130 m circuit.
function drawCarSprite(ctx, X, Y, scale, px, py, th, phiDeg, slip, col) {
  const s = Math.max(scale, 40), phi = (phiDeg || 0) * Math.PI / 180;
  ctx.save();
  ctx.translate(X(px), Y(py));
  ctx.rotate(-th);
  const ax = 0.12 * s, ay = 0.10 * s, wl = 0.085 * s, ww = 0.05 * s;
  ctx.fillStyle = css("--text-primary");
  for (const [x, y, rot] of [[-ax, -ay, 0], [-ax, ay, 0],
                             [ax, -ay, -phi], [ax, ay, -phi]]) {
    ctx.save(); ctx.translate(x, y); ctx.rotate(rot);
    ctx.fillRect(-wl / 2, -ww / 2, wl, ww);
    ctx.restore();
  }
  const body = [[-0.20, -0.09], [0.06, -0.09], [0.17, -0.035], [0.22, 0],
                [0.17, 0.035], [0.06, 0.09], [-0.20, 0.09]];
  ctx.beginPath();
  body.forEach(([x, y], i) =>
    i ? ctx.lineTo(x * s, y * s) : ctx.moveTo(x * s, y * s));
  ctx.closePath();
  ctx.fillStyle = css("--series-1"); ctx.fill();
  ctx.lineWidth = Math.max(1, 0.02 * s);
  ctx.strokeStyle = slip ? css("--series-2") : css("--surface-1");
  ctx.stroke();
  ctx.fillStyle = css("--surface-1");
  ctx.beginPath(); ctx.arc(0, 0, 0.032 * s, 0, 7); ctx.fill();
  ctx.restore();
  if (col) {
    ctx.strokeStyle = css("--serious"); ctx.lineWidth = 4;
    ctx.globalAlpha = 0.9;
    ctx.beginPath(); ctx.arc(X(px), Y(py), 0.24 * s + 4, 0, 7); ctx.stroke();
    ctx.globalAlpha = 1;
  }
}

// Skid marks: a short dash behind each rear wheel per sliding pose.
function drawSkidMarks(ctx, X, Y, scale, skids) {
  ctx.strokeStyle = css("--text-secondary"); ctx.globalAlpha = 0.55;
  ctx.lineWidth = Math.max(1, 0.045 * scale); ctx.lineCap = "butt";
  ctx.beginPath();
  for (const [x, y, th] of skids) {
    const ct = Math.cos(th), st = Math.sin(th);
    for (const side of [-0.10, 0.10]) {
      const wx = x - 0.12 * ct - side * st, wy = y - 0.12 * st + side * ct;
      ctx.moveTo(X(wx - 0.05 * ct), Y(wy - 0.05 * st));
      ctx.lineTo(X(wx + 0.05 * ct), Y(wy + 0.05 * st));
    }
  }
  ctx.stroke();
  ctx.globalAlpha = 1;
}

function drawState({ pose, trail, colliding, bump, rays, rayAngles,
  keyTaken, doorOpen, car, phi, slipping, skids }) {
  const cv = $("#maze"), ctx = cv.getContext("2d");
  const m = S.maze, isTrack = !!m && m.kind === "track";
  // A circuit's bbox is wide; a maze keeps the square canvas.
  const wantH = isTrack ? Math.round(cv.width * Math.min(1.2, Math.max(
    0.45, m.height / m.width))) : 900;
  if (cv.height !== wantH) cv.height = wantH;
  ctx.fillStyle = css("--surface-1");
  ctx.fillRect(0, 0, cv.width, cv.height);
  if (!m) return;
  const worldW = isTrack ? m.width : m.width * m.cell_size;
  const worldH = isTrack ? m.height : m.height * m.cell_size;
  const pad = 24;
  const scale = Math.min((cv.width - 2 * pad) / worldW,
                         (cv.height - 2 * pad) / worldH);
  const ox = (cv.width - worldW * scale) / 2;
  const oy = (cv.height - worldH * scale) / 2;
  const X = (x) => ox + x * scale;
  const Y = (y) => cv.height - oy - y * scale;

  if (isTrack) {
    drawTrackScene(ctx, m, X, Y, scale);
  } else {
  // goal + start cells
  const cs = m.cell_size;
  const cell = (c, fill, label) => {
    ctx.fillStyle = fill;
    ctx.fillRect(X(c[0] * cs), Y((c[1] + 1) * cs), cs * scale, cs * scale);
    ctx.fillStyle = css("--text-secondary");
    ctx.font = `${Math.round(0.4 * cs * scale)}px system-ui`;
    ctx.textAlign = "center"; ctx.textBaseline = "middle";
    ctx.fillText(label, X((c[0] + 0.5) * cs), Y((c[1] + 0.5) * cs));
  };
  ctx.globalAlpha = 0.18;
  cell(m.goal_cell, css("--good"), "");
  cell(m.start_cell, css("--text-muted"), "");
  ctx.globalAlpha = 1;
  cell(m.goal_cell, "transparent", "G");
  cell(m.start_cell, "transparent", "S");

  // walls
  ctx.strokeStyle = css("--text-primary");
  ctx.lineWidth = 3; ctx.lineCap = "round";
  ctx.beginPath();
  for (const [x1, y1, x2, y2] of m.segments) {
    ctx.moveTo(X(x1), Y(y1)); ctx.lineTo(X(x2), Y(y2));
  }
  ctx.stroke();
  }

  // locked-exit scenario: door (until opened) and key (until taken)
  if (m.locked && m.door_segments && !doorOpen) {
    ctx.strokeStyle = css("--serious");
    ctx.lineWidth = 5;
    ctx.beginPath();
    for (const [x1, y1, x2, y2] of m.door_segments) {
      ctx.moveTo(X(x1), Y(y1)); ctx.lineTo(X(x2), Y(y2));
    }
    ctx.stroke();
  }
  if (m.locked && m.key_pos && !keyTaken) {
    const [kx, ky] = m.key_pos;
    ctx.fillStyle = css("--series-1");
    ctx.save();
    ctx.translate(X(kx), Y(ky));
    ctx.rotate(Math.PI / 4);
    ctx.fillRect(-5, -5, 10, 10);
    ctx.restore();
  }

  // trail (a car's trail is its racing line: thin, in the robot color)
  if (car && skids && skids.length && $("#chk-trail").checked) {
    drawSkidMarks(ctx, X, Y, scale, skids);
  }
  if (trail && trail.length > 1 && $("#chk-trail").checked) {
    ctx.strokeStyle = css(car ? "--series-1" : "--series-2");
    ctx.lineWidth = car ? 1.3 : 1.6; ctx.globalAlpha = car ? 0.7 : 0.8;
    ctx.lineJoin = "round";
    ctx.beginPath();
    ctx.moveTo(X(trail[0][1]), Y(trail[0][2]));
    for (const p of trail) ctx.lineTo(X(p[1]), Y(p[2]));
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  if (!pose) return;
  const [px, py, th] = pose;

  // lidar rays (live only — ground-truth cast)
  if (rays && rayAngles && $("#chk-rays").checked) {
    ctx.strokeStyle = css("--series-3");
    ctx.lineWidth = 1; ctx.globalAlpha = 0.55;
    ctx.beginPath();
    rays.forEach((r, i) => {
      const a = th + rayAngles[i];
      ctx.moveTo(X(px), Y(py));
      ctx.lineTo(X(px + r * Math.cos(a)), Y(py + r * Math.sin(a)));
    });
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  // robot
  if (car) {
    drawCarSprite(ctx, X, Y, scale, px, py, th, phi, slipping, colliding);
    return;
  }
  const rr = 0.09 * scale;
  if (colliding) {
    ctx.strokeStyle = css("--serious");
    ctx.lineWidth = 4; ctx.globalAlpha = 0.9;
    ctx.beginPath(); ctx.arc(X(px), Y(py), rr + 5, 0, 7); ctx.stroke();
    ctx.globalAlpha = 1;
  }
  ctx.fillStyle = css("--series-1");
  ctx.beginPath(); ctx.arc(X(px), Y(py), rr, 0, 7); ctx.fill();
  ctx.strokeStyle = css("--surface-1");
  ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(X(px), Y(py));
  ctx.lineTo(X(px + 0.09 * Math.cos(th)), Y(py + 0.09 * Math.sin(th)));
  ctx.stroke();
  if (bump && (bump[0] || bump[1])) {
    ctx.strokeStyle = css("--serious"); ctx.lineWidth = 3;
    const dir = bump[0] ? th : th + Math.PI;
    ctx.beginPath();
    ctx.arc(X(px), Y(py), rr + 2, -dir - 0.6, -dir + 0.6);
    ctx.stroke();
  }
}

/* ---------------- transcript ---------------- */

function renderBlockText(txt) {
  const d = document.createElement("div");
  d.textContent = txt;
  return d.innerHTML;
}

function renderEvent(ev) {
  const root = document.createElement("div");
  root.className = `ev ${ev.type}`;
  if (ev.type === "meta") {
    root.className = "ev note";
    root.innerHTML = `<div class="body">episode ${ev.episode} · arm ` +
      `${ev.arm} · labels ${ev.labels} · ${ev.noise_profile} · maze ` +
      `${ev.maze_hash} · ${ev.model}</div>`;
  } else if (ev.type === "system_prompt") {
    root.innerHTML = `<details><summary class="who">system prompt` +
      `</summary><div class="out">${renderBlockText(ev.content)}` +
      `</div></details>`;
  } else if (ev.type === "assistant") {
    let html = `<div class="who">agent</div>`;
    for (const b of ev.content || []) {
      if (b.type === "text" && b.text) {
        html += `<div class="body">${renderBlockText(b.text)}</div>`;
      } else if (b.type === "thinking" && b.thinking) {
        html += `<div class="think">${renderBlockText(b.thinking)}</div>`;
      } else if (b.type === "tool_use") {
        html += `<div class="cmd">$ ${renderBlockText(
          (b.input || {}).command || "")}</div>`;
      }
    }
    root.innerHTML = html;
  } else if (ev.type === "exec_result") {
    root.innerHTML = `<div class="out">${renderBlockText(ev.output)}</div>`;
  } else if (ev.type === "note") {
    root.className = "ev note";
    root.innerHTML = `<div class="body">— ${ev.kind}` +
      `${ev.reason ? ": " + ev.reason : ""} —</div>`;
  } else {
    return null;
  }
  return root;
}

async function pollTranscript(all = false) {
  const q = `series=${encodeURIComponent(S.series)}&ep=${S.ep}` +
    `&after=${S.transcriptCursor}`;
  let t;
  try { t = await api(`/api/transcript?${q}`); } catch { return; }
  if (!t.events.length) return;
  const pane = $("#tab-transcript");
  for (const ev of t.events) {
    const node = renderEvent(ev);
    if (node) pane.appendChild(node);
  }
  S.transcriptCursor = t.next;
  if ($("#chk-autoscroll").checked) pane.scrollTop = pane.scrollHeight;
  if (all && t.events.length >= 500) pollTranscript(true);
}

/* ---------------- memory ---------------- */

async function loadMemory() {
  const snap = S.running ? "" : `&snapshot=${S.ep}`;
  const q = `series=${encodeURIComponent(S.series)}${snap}`;
  let tree = [];
  try { tree = await api(`/api/memory/tree?${q}`); } catch { }
  const pane = $("#memory-tree");
  pane.innerHTML = "";
  const diffBtn = document.createElement("div");
  diffBtn.className = "file";
  diffBtn.textContent = "± changes this episode";
  diffBtn.onclick = () => showMemoryDiff();
  pane.appendChild(diffBtn);
  if (!tree.length) {
    const d = document.createElement("div");
    d.className = "empty"; d.textContent = "(memory is empty)";
    pane.appendChild(d);
  }
  for (const f of tree) {
    const d = document.createElement("div");
    d.className = "file";
    d.textContent = `${f.path} (${f.size}b)`;
    d.onclick = () => showMemoryFile(f.path, d);
    pane.appendChild(d);
  }
}

async function showMemoryFile(path, node) {
  document.querySelectorAll("#memory-tree .file.sel")
    .forEach((n) => n.classList.remove("sel"));
  if (node) node.classList.add("sel");
  const snap = S.running ? "" : `&snapshot=${S.ep}`;
  const q = `series=${encodeURIComponent(S.series)}${snap}` +
    `&path=${encodeURIComponent(path)}`;
  const f = await api(`/api/memory/file?${q}`);
  $("#memory-view").innerHTML =
    `<pre>${renderBlockText(f.content)}</pre>` +
    (f.truncated ? '<div class="empty">(truncated)</div>' : "");
}

async function showMemoryDiff() {
  const q = `series=${encodeURIComponent(S.series)}&ep=${S.ep}`;
  let d;
  try { d = await api(`/api/memory/diff?${q}`); } catch {
    $("#memory-view").innerHTML =
      '<div class="empty">no snapshot yet (episode still running?)</div>';
    return;
  }
  if (!d.diffs.length) {
    $("#memory-view").innerHTML =
      '<div class="empty">no memory changes this episode</div>';
    return;
  }
  let html = "";
  for (const f of d.diffs) {
    html += `<div class="diff-file">${renderBlockText(f.path)}</div><pre>`;
    for (const line of f.diff.split("\n")) {
      const cls = line.startsWith("+") ? "diff-add"
        : line.startsWith("-") ? "diff-del"
          : line.startsWith("@@") ? "diff-hunk" : "";
      html += `<span class="${cls}">${renderBlockText(line)}</span>\n`;
    }
    html += "</pre>";
  }
  $("#memory-view").innerHTML = html;
}

/* ---------------- metrics ---------------- */

function svgLineChart(points, { w = 620, h = 200, yFmt = (v) => v,
  color, dnf = [] }) {
  const pad = { l: 46, r: 12, t: 12, b: 26 };
  const xs = points.map((p) => p.x);
  const ys = points.filter((p) => p.y != null).map((p) => p.y);
  const xmin = Math.min(...xs), xmax = Math.max(...xs);
  const ymax = ys.length ? Math.max(...ys) * 1.15 : 1;
  const SX = (x) => pad.l + (xmax === xmin ? 0.5 : (x - xmin) / (xmax - xmin))
    * (w - pad.l - pad.r);
  const SY = (y) => h - pad.b - (y / ymax) * (h - pad.t - pad.b);
  const grid = css("--border"), ink = css("--text-secondary");
  let s = `<svg viewBox="0 0 ${w} ${h}" role="img">`;
  for (let i = 0; i <= 3; i++) {
    const yv = (ymax / 3) * i, y = SY(yv);
    s += `<line x1="${pad.l}" x2="${w - pad.r}" y1="${y}" y2="${y}"
      stroke="${grid}" stroke-width="1"/>`;
    s += `<text x="${pad.l - 6}" y="${y + 4}" text-anchor="end"
      font-size="10" fill="${ink}">${yFmt(yv)}</text>`;
  }
  const solid = points.filter((p) => p.y != null);
  if (solid.length > 1) {
    s += `<path fill="none" stroke="${color}" stroke-width="2" d="M` +
      solid.map((p) => `${SX(p.x)},${SY(p.y)}`).join(" L") + `"/>`;
  }
  for (const p of points) {
    if (p.y != null) {
      s += `<circle cx="${SX(p.x)}" cy="${SY(p.y)}" r="4.5"
        fill="${color}" stroke="${css("--surface-1")}" stroke-width="2">
        <title>ep ${p.x}: ${yFmt(p.y)}${p.note ? " · " + p.note : ""}</title>
        </circle>`;
    } else {
      s += `<g><circle cx="${SX(p.x)}" cy="${SY(ymax * 0.92)}" r="4.5"
        fill="none" stroke="${css("--serious")}" stroke-width="2">
        <title>ep ${p.x}: did not solve (${p.note || "DNF"})</title>
        </circle>
        <text x="${SX(p.x)}" y="${SY(ymax * 0.92) - 8}" text-anchor="middle"
        font-size="9" fill="${css("--serious")}">DNF</text></g>`;
    }
    s += `<text x="${SX(p.x)}" y="${h - 8}" text-anchor="middle"
      font-size="10" fill="${ink}">${p.x}</text>`;
  }
  return s + "</svg>";
}

function svgBarChart(points, { w = 620, h = 160, yFmt = (v) => v, color }) {
  const pad = { l: 52, r: 12, t: 10, b: 26 };
  const ymax = Math.max(...points.map((p) => p.y || 0), 1) * 1.1;
  const n = points.length;
  const bw = Math.min(34, (w - pad.l - pad.r) / Math.max(n, 1) - 4);
  const grid = css("--border"), ink = css("--text-secondary");
  let s = `<svg viewBox="0 0 ${w} ${h}" role="img">`;
  for (let i = 0; i <= 2; i++) {
    const yv = (ymax / 2) * i;
    const y = h - pad.b - (yv / ymax) * (h - pad.t - pad.b);
    s += `<line x1="${pad.l}" x2="${w - pad.r}" y1="${y}" y2="${y}"
      stroke="${grid}"/>`;
    s += `<text x="${pad.l - 6}" y="${y + 4}" text-anchor="end"
      font-size="10" fill="${ink}">${yFmt(yv)}</text>`;
  }
  points.forEach((p, i) => {
    const x = pad.l + (i + 0.5) * ((w - pad.l - pad.r) / n) - bw / 2;
    const bh = ((p.y || 0) / ymax) * (h - pad.t - pad.b);
    const y = h - pad.b - bh;
    s += `<rect x="${x}" y="${y}" width="${bw}" height="${bh}"
      rx="4" fill="${color}">
      <title>ep ${p.x}: ${yFmt(p.y || 0)}</title></rect>`;
    s += `<text x="${x + bw / 2}" y="${h - 8}" text-anchor="middle"
      font-size="10" fill="${ink}">${p.x}</text>`;
  });
  return s + "</svg>";
}

function metricsTable(rows, cols) {
  // Escape everything: quiz/provenance rows carry agent-influenced text.
  let h = '<table class="metrics"><tr>' +
    cols.map((c) => `<th>${renderBlockText(c)}</th>`).join("") + "</tr>";
  for (const r of rows) {
    h += "<tr>" + cols.map((c) =>
      `<td>${r[c] == null ? "—" : renderBlockText(String(r[c]))}</td>`)
      .join("") + "</tr>";
  }
  return h + "</table>";
}

async function loadMetrics() {
  const pane = $("#tab-metrics");
  let m;
  try {
    m = await api(`/api/metrics?series=${encodeURIComponent(S.series)}`);
  } catch { pane.innerHTML = '<div class="empty">no metrics</div>'; return; }
  const c = m.learning_curve;
  if (!c.length) {
    pane.innerHTML = '<div class="empty">no finished episodes yet</div>';
    return;
  }
  let html = '<div class="chart-block"><h3>Time to solve (sim seconds)' +
    "</h3>" + svgLineChart(
      c.map((e) => ({
        x: e.episode, y: e.sim_time_to_solve_s,
        note: e.end_reason,
      })),
      { color: css("--series-1"), yFmt: (v) => `${Math.round(v)}s` }) +
    "</div>";
  html += '<div class="chart-block"><h3>Model output tokens</h3>' +
    svgBarChart(c.map((e) => ({ x: e.episode, y: e.tokens_out })),
      { color: css("--series-2"), yFmt: (v) => `${Math.round(v / 1000)}k` })
    + "</div>";
  html += "<h3>Episodes</h3>" + metricsTable(c,
    ["episode", "solved", "end_reason", "sim_time_to_solve_s", "wall_s",
     "turns", "restarts", "collisions", "tokens_out"]);
  for (const [name, ev] of Object.entries(m.evals || {})) {
    html += `<div class="chart-block"><h3>eval: ` +
      `${renderBlockText(name)}</h3>`;
    if (ev.rows && ev.rows.length) {
      html += metricsTable(ev.rows, Object.keys(ev.rows[0]));
    }
    if (ev.summary) {
      html += `<pre style="font-size:12px">${renderBlockText(
        JSON.stringify(ev.summary, null, 2))}</pre>`;
    }
    html += "</div>";
  }
  pane.innerHTML = html;
}

/* ---------------- tabs & controls ---------------- */

function refreshActiveTab() {
  const active =
    document.querySelector(".tabs button[data-tab].active").dataset.tab;
  if (active === "memory") loadMemory();
  else if (active === "metrics") loadMetrics();
}

// Right-panel tabs only; the left panel's view tabs (data-view) are
// wired in view3d.js.
document.querySelectorAll(".tabs button[data-tab]").forEach((b) => {
  b.addEventListener("click", () => {
    document.querySelectorAll(".tabs button[data-tab]")
      .forEach((x) => x.classList.remove("active"));
    document.querySelectorAll(".tabview")
      .forEach((x) => x.classList.remove("active"));
    b.classList.add("active");
    $(`#tab-${b.dataset.tab}`).classList.add("active");
    $("#autoscroll-wrap").style.display =
      b.dataset.tab === "transcript" ? "" : "none";
    refreshActiveTab();
  });
});

$("#series-sel").addEventListener("change",
  (e) => selectSeries(e.target.value));
$("#episode-sel").addEventListener("change", (e) => {
  const opt = e.target.selectedOptions[0];
  selectEpisode(Number(opt.value), !!opt.dataset.running);
});
$("#btn-pause").addEventListener("click",
  () => fetch("/api/live/pause", { method: "POST" }));
$("#btn-resume").addEventListener("click",
  () => fetch("/api/live/resume", { method: "POST" }));
$("#btn-rtf").addEventListener("click", () =>
  fetch("/api/live/rtf", {
    method: "POST",
    body: JSON.stringify({ factor: Number($("#rtf-input").value) }),
  }));

/* periodically refresh episode list while a run is live */
setInterval(async () => {
  if (!S.series || !S.running) return;
  const eps = await api(
    `/api/episodes?series=${encodeURIComponent(S.series)}`);
  const mine = eps.find((e) => e.episode === Number(S.ep));
  if (mine && !mine.running) selectEpisode(S.ep, false);
}, 5000);

loadSeries();
