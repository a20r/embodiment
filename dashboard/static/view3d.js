/* Mazebot dashboard — 3D view.  three.js r128 (UMD) + OrbitControls,
   vendored.  Renders the maze walls extruded to their configured
   height, the floor, the goal cell, the locked-maze door and key, each
   robot as a cylinder with a heading mark, and the live ground-truth
   point cloud when the run has lidar3d.  Everything here is
   experimenter-side; nothing reaches the bot. */
"use strict";

(() => {
  const V = {
    active: false, ready: false, failed: false, raf: 0,
    scene: null, camera: null, renderer: null, controls: null,
    mazeHash: null, mazeGroup: null, door: null, key: null, cfg: null,
    robots: new Map(), clouds: new Map(),
  };
  const DEF = { wall_height: 0.4, robot_height: 0.15, sensor_height: 0.15,
                post_height: 0.25 };
  const host = () => document.querySelector("#view3d");
  const note = (msg) => {
    const n = document.querySelector("#view3d-note");
    if (n) { n.textContent = msg; n.style.display = msg ? "" : "none"; }
  };
  const color = (v) => new THREE.Color(css(v) || "#888");

  function init() {
    if (V.ready || V.failed) return V.ready;
    if (typeof THREE === "undefined" || !THREE.OrbitControls) {
      V.failed = true;
      note("three.js vendor files did not load — 3D view unavailable");
      return false;
    }
    let renderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true });
    } catch (e) {
      V.failed = true;
      note("WebGL unavailable — 3D view disabled");
      return false;
    }
    const h = host();
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    h.appendChild(renderer.domElement);
    const scene = new THREE.Scene();
    scene.background = color("--surface-1");
    const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
    camera.up.set(0, 0, 1);                 // z is up, like the sim
    camera.position.set(1.5, -2.2, 1.8);
    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true; controls.dampingFactor = 0.12;
    controls.target.set(1.5, 1.5, 0);
    scene.add(new THREE.AmbientLight(0xffffff, 0.75));
    const sun = new THREE.DirectionalLight(0xffffff, 0.55);
    sun.position.set(2, -3, 5);
    scene.add(sun);
    Object.assign(V, { scene, camera, renderer, controls, ready: true });
    new ResizeObserver(resize).observe(h);
    // Theme flips re-sample the CSS palette on the next update.
    const mq = window.matchMedia && window.matchMedia(
      "(prefers-color-scheme: dark)");
    if (mq && mq.addEventListener) mq.addEventListener("change", () => {
      scene.background = color("--surface-1");
      reset();
      if (V.active && S.lastState) update(S.lastState);
    });
    resize();
    return true;
  }

  function resize() {
    if (!V.ready) return;
    const h = host();
    const w = h.clientWidth || 600, ht = h.clientHeight || 420;
    V.renderer.setSize(w, ht, false);
    V.camera.aspect = w / ht;
    V.camera.updateProjectionMatrix();
  }

  // One render loop, started on demand and self-terminating when the
  // view is hidden.
  function animate() {
    if (!V.active || !V.ready) { V.raf = 0; return; }
    V.controls.update();
    V.renderer.render(V.scene, V.camera);
    V.raf = requestAnimationFrame(animate);
  }
  function startLoop() {
    if (!V.raf) V.raf = requestAnimationFrame(animate);
  }

  function dispose(obj) {
    if (!obj) return;
    obj.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) {
        (Array.isArray(o.material) ? o.material : [o.material])
          .forEach((m) => m.dispose());
      }
    });
    V.scene.remove(obj);
  }

  /* Drop everything episode-specific (called on episode change). */
  function reset() {
    if (!V.ready) return;
    dispose(V.mazeGroup); dispose(V.door); dispose(V.key);
    V.mazeGroup = V.door = V.key = null;
    for (const r of V.robots.values()) dispose(r);
    for (const c of V.clouds.values()) dispose(c);
    V.robots.clear(); V.clouds.clear();
    V.mazeHash = null; V.cfg = null;
  }

  /* ---- static world: walls, floor, goal, door, key ---- */

  function buildMaze(m, cfg) {
    dispose(V.mazeGroup); dispose(V.door); dispose(V.key);
    V.door = V.key = null;
    const g = new THREE.Group();
    const W = m.width * m.cell_size, H = m.height * m.cell_size;
    const wallH = cfg.wall_height;
    const floor = new THREE.Mesh(
      new THREE.PlaneGeometry(W + 1.2, H + 1.2),
      new THREE.MeshLambertMaterial({ color: color("--surface-0") }));
    floor.position.set(W / 2, H / 2, -0.001);
    g.add(floor);
    // Translucent walls: the interesting geometry (clouds, robots) is
    // inside the corridors, seen from above.  One shared unit box,
    // scaled per segment.
    const wallMat = new THREE.MeshLambertMaterial({
      color: color("--text-muted"), transparent: true, opacity: 0.45,
      depthWrite: false });
    const unit = new THREE.BoxGeometry(1, 1, 1);
    for (const [x1, y1, x2, y2] of m.segments) {
      const len = Math.hypot(x2 - x1, y2 - y1);
      if (len < 1e-6) continue;
      const box = new THREE.Mesh(unit, wallMat);
      box.scale.set(len + 0.02, 0.02, wallH);
      box.position.set((x1 + x2) / 2, (y1 + y2) / 2, wallH / 2);
      box.rotation.z = Math.atan2(y2 - y1, x2 - x1);
      g.add(box);
    }
    const cs = m.cell_size, gc = m.goal_cell;
    const goal = new THREE.Mesh(
      new THREE.PlaneGeometry(cs, cs),
      new THREE.MeshBasicMaterial({ color: color("--good"),
        transparent: true, opacity: 0.35 }));
    goal.position.set((gc[0] + 0.5) * cs, (gc[1] + 0.5) * cs, 0.002);
    g.add(goal);
    V.scene.add(g);
    V.mazeGroup = g;
    // Locked-exit scenario: the door is a wall until unlocked, the key
    // a post until carried - both are solids in the 3D cast.
    if (m.locked && m.door_segments) {
      const d = new THREE.Group();
      const mat = new THREE.MeshLambertMaterial({ color: color("--serious") });
      for (const [x1, y1, x2, y2] of m.door_segments) {
        const box = new THREE.Mesh(unit, mat);
        box.scale.set(Math.hypot(x2 - x1, y2 - y1) + 0.02, 0.03, wallH);
        box.position.set((x1 + x2) / 2, (y1 + y2) / 2, wallH / 2);
        box.rotation.z = Math.atan2(y2 - y1, x2 - x1);
        d.add(box);
      }
      V.scene.add(d); V.door = d;
    }
    if (m.locked && m.key_pos) {
      const post = new THREE.Mesh(
        new THREE.CylinderGeometry(0.035, 0.035, cfg.post_height, 12),
        new THREE.MeshLambertMaterial({ color: color("--series-1") }));
      post.rotation.x = Math.PI / 2;
      post.position.set(m.key_pos[0], m.key_pos[1], cfg.post_height / 2);
      V.scene.add(post); V.key = post;
    }
    V.mazeHash = m.hash;
    // Frame the whole arena from above and slightly south (~55 deg).
    const span = Math.max(W, H);
    V.controls.target.set(W / 2, H / 2, 0);
    V.camera.position.set(W / 2, H / 2 - 0.9 * span, 1.25 * span);
    V.controls.update();
  }

  /* ---- robots ---- */

  function robotMesh(id, cfg) {
    let r = V.robots.get(id);
    if (r) return r;
    const grp = new THREE.Group();
    const body = new THREE.Mesh(
      new THREE.CylinderGeometry(0.09, 0.09, cfg.robot_height, 24),
      new THREE.MeshLambertMaterial({
        color: color(id === "b" ? "--series-2" : "--series-1") }));
    body.rotation.x = Math.PI / 2;
    body.position.z = cfg.robot_height / 2;
    grp.add(body);
    const nose = new THREE.Mesh(
      new THREE.BoxGeometry(0.08, 0.02, 0.02),
      new THREE.MeshBasicMaterial({ color: color("--text-primary") }));
    nose.position.set(0.07, 0, cfg.robot_height + 0.01);
    grp.add(nose);
    const sensor = new THREE.Mesh(
      new THREE.CylinderGeometry(0.03, 0.03, 0.03, 16),
      new THREE.MeshLambertMaterial({ color: 0x222222 }));
    sensor.rotation.x = Math.PI / 2;
    sensor.position.z = cfg.sensor_height + 0.015;
    grp.add(sensor);
    V.scene.add(grp);
    V.robots.set(id, grp);
    return grp;
  }

  /* ---- point clouds: preallocated buffers, written in place ---- */

  function cloudFor(id, capacity) {
    let c = V.clouds.get(id);
    if (c && c.userData.capacity >= capacity) return c;
    if (c) dispose(c);
    const geom = new THREE.BufferGeometry();
    const pos = new THREE.BufferAttribute(new Float32Array(capacity * 3), 3);
    const col = new THREE.BufferAttribute(new Float32Array(capacity * 3), 3);
    pos.setUsage(THREE.DynamicDrawUsage); col.setUsage(THREE.DynamicDrawUsage);
    geom.setAttribute("position", pos);
    geom.setAttribute("color", col);
    const mat = new THREE.PointsMaterial({
      size: 0.022, vertexColors: true, sizeAttenuation: true });
    c = new THREE.Points(geom, mat);
    c.frustumCulled = false;
    c.userData.capacity = capacity;
    V.scene.add(c);
    V.clouds.set(id, c);
    return c;
  }

  function setCloud(id, pts, cfg) {
    const n = pts ? pts.length : 0;
    const c = cloudFor(id, Math.max(n, 1024));
    const pos = c.geometry.getAttribute("position");
    const col = c.geometry.getAttribute("color");
    const tmp = new THREE.Color();
    const top = Math.max(cfg.wall_height, 0.01);
    for (let i = 0; i < n; i++) {
      const [x, y, z] = pts[i];
      pos.setXYZ(i, x, y, z);
      // floor returns cool, wall faces warm: hue by height.
      tmp.setHSL(0.62 - 0.5 * Math.min(1, z / top), 0.85, 0.55);
      col.setXYZ(i, tmp.r, tmp.g, tmp.b);
    }
    pos.needsUpdate = true; col.needsUpdate = true;
    c.geometry.setDrawRange(0, n);
  }

  /* ---- per-state update (called from app.js) ---- */

  function update(st) {
    if (!V.active || !init()) return;
    const cfg = Object.assign({}, DEF, st.lidar3d_cfg || V.cfg || {});
    if (st.lidar3d_cfg) V.cfg = st.lidar3d_cfg;
    if (S.maze && S.maze.hash !== V.mazeHash) buildMaze(S.maze, cfg);
    const bots = st.bots
      ? st.bots.map((b) => ({ id: b.bot_id, pose: b.pose,
                              cloud: b.lidar3d_true }))
      : [{ id: "a", pose: st.pose, cloud: st.lidar3d_true }];
    const ids = new Set(bots.map((b) => b.id));
    for (const [id, r] of V.robots) if (!ids.has(id)) {
      dispose(r); V.robots.delete(id);
    }
    for (const [id, c] of V.clouds) if (!ids.has(id)) {
      dispose(c); V.clouds.delete(id);
    }
    let anyCloud = false;
    for (const b of bots) {
      if (!b.pose) continue;
      const g = robotMesh(b.id, cfg);
      g.position.set(b.pose[0], b.pose[1], 0);
      g.rotation.z = b.pose[2];
      if (b.cloud) { setCloud(b.id, b.cloud, cfg); anyCloud = true; }
      else if (V.clouds.has(b.id)) setCloud(b.id, [], cfg);
    }
    if (V.door) V.door.visible = !st.door_open;
    if (V.key) V.key.visible = !st.key_carried;
    note(anyCloud ? "" : S.running
      ? "no lidar3d in this run — walls and robots only"
      : "replay — walls and robots only (clouds are live-only)");
    const follow = document.querySelector("#chk-follow");
    if (follow && follow.checked && bots[0] && bots[0].pose) {
      const [x, y] = bots[0].pose;
      const t = V.controls.target, dx = x - t.x, dy = y - t.y;
      t.x += dx * 0.2; t.y += dy * 0.2;
      V.camera.position.x += dx * 0.2; V.camera.position.y += dy * 0.2;
    }
  }

  /* ---- view tabs ---- */

  function setView(which) {
    V.active = which === "3d";
    document.querySelector("#maze-wrap").style.display = V.active ? "none" : "";
    document.querySelector("#view3d").style.display = V.active ? "" : "none";
    document.querySelector("#follow-wrap").style.display = V.active ? "" : "none";
    document.querySelectorAll("#view-tabs button").forEach((b) =>
      b.classList.toggle("active", b.dataset.view === which));
    if (V.active && init()) {
      resize();
      startLoop();
      if (S.lastState) update(S.lastState);
      else if (S.replay && S.replay.poses.length) drawReplay();
      else if (S.maze) update({});
    }
  }
  document.querySelectorAll("#view-tabs button").forEach((b) =>
    b.addEventListener("click", () => setView(b.dataset.view)));

  window.view3dActive = () => V.active && V.ready;
  window.view3dUpdate = update;
  window.view3dReset = reset;
})();
