import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

/* ------------------------------------------------ stage content */
const STAGES = [
  { id: "intro", label: "1 · Meet the robot", title: "Meet Asimov 1", html: `
    <p>This is the <b>Menlo Asimov 1</b>, an open-source humanoid robot that costs
    about <b>$16,000</b> to build for real. In front of you is its virtual twin,
    simulated with the same physics engine (MuJoCo) Menlo uses, built from the
    real published specifications.</p>
    <table>
      <tr><th>Height</th><td>1.23 m</td></tr>
      <tr><th>Weight</th><td>35 kg</td></tr>
      <tr><th>Joints</th><td>25 motors + 2 spring-loaded toes</td></tr>
      <tr><th>Battery</th><td>491 Wh Li-ion (like ~7 laptop batteries)</td></tr>
      <tr><th>Body cost</th><td>$14,000 (motors + chassis)</td></tr>
      <tr><th>Electronics</th><td>$818</td></tr>
      <tr><th>Battery + cert</th><td>$830</td></tr>
    </table>
    <p>The robot you see is <b>alive right now</b> — a balance controller is
    holding it upright at 500 decisions per second, exactly like the real one.
    Drag to look around it.</p>
    <p>This tour builds it the way a real one is built: electronics on the
    bench first, then legs on a test fixture, then the upper body, then the
    full robot — with acceptance tests at every stage. Use the arrows below.</p>` ,
    actions: [] },

  { id: "electronics", label: "2 · Bench electronics", title: "Stage 1 — Electronics on the bench", html: `
    <p>Nothing gets bolted to a robot before it works on the bench. Here is the
    power and nervous system laid out on the table:</p>
    <ul>
      <li><b>Gold box — battery pack</b>: 52 Li-ion cells (the 18650 type used in
      laptops), wired 13 in series × 4 in parallel → <b>46.8 V, 491 Wh</b>.</li>
      <li><b>Large green board — Motion Control Board</b>: talks to all 25 motors
      over <b>CAN bus</b>, the same network protocol cars use.</li>
      <li><b>Small boards — Raspberry Pi 5 + Radxa CM5</b>: the robot's brains.</li>
    </ul>
    <h3>Why these tests matter</h3>
    <p><b>BMS test</b> — the Battery Management System must cut power if current
    exceeds 30 A, or a short circuit becomes a fire. We deliberately fake a 2 kW
    short and confirm it trips.</p>
    <p><b>CAN budget</b> — commanding 25 motors 500 times a second is a lot of
    traffic. Each leg branch carries 6 motors and runs at <b>78% of wire
    capacity</b> — over 80% and commands start arriving late, which on a walking
    robot means falling. This is real engineering: the design barely fits, on
    purpose (faster buses cost more).</p>`,
    actions: [{ id: "bench_tests", label: "▶ Run bench tests" }] },

  { id: "legs", label: "3 · Legs on fixture", title: "Stage 2 — Legs on the assembly fixture", html: `
    <p>The pelvis is clamped to a fixture (that's why it floats — the real one
    is bolted to a stand) and both legs are attached. Each leg has 6 motors:</p>
    <table>
      <tr><th>Joint</th><th>Peak torque</th><th>Motor</th></tr>
      <tr><td>Hip pitch</td><td>120 Nm</td><td>EC-A6416</td></tr>
      <tr><td>Hip roll</td><td>90 Nm</td><td>EC-A5013</td></tr>
      <tr><td>Hip yaw</td><td>60 Nm</td><td>EC-A3814</td></tr>
      <tr><td>Knee</td><td>75 Nm</td><td>EC-A4315</td></tr>
      <tr><td>Ankle ×2</td><td>145 / 58 Nm</td><td>EC-A4310 pair</td></tr>
    </table>
    <p>120 Nm is roughly the force of standing on a wrench half a meter long —
    hips work hard when a 35 kg robot squats.</p>
    <p>The <b>ankle is special</b>: two identical motors drive it together
    through push-rods (a "parallel RSU" mechanism), combining their strength.
    And the <b>toes have no motor at all</b> — just a spring, which stores and
    returns energy each step, like your own toes.</p>
    <h3>The sweep test</h3>
    <p>Every joint is driven to 80% of its travel and must arrive within
    0.05 rad (~3°). Watch the leg move — and notice the other leg swings
    outward first so they don't collide. A real technician does exactly this.</p>`,
    actions: [{ id: "leg_sweep", label: "▶ Run joint sweep test" }] },

  { id: "upper_body", label: "4 · Upper body", title: "Stage 3 — Upper body on the waist fixture", html: `
    <p>The torso is mounted on a pedestal at waist height. Each arm has 5 motors
    (shoulder ×3, elbow, wrist), the neck has 2, the waist 1.</p>
    <h3>A worked example: is the shoulder strong enough?</h3>
    <p>The spec says Asimov can carry <b>5–18 kg</b>. Let's check a 2 kg payload
    held at full reach (48 cm):</p>
    <ul>
      <li>Torque from the arm's own weight: <b>~6.2 Nm</b></li>
      <li>Torque from 2 kg at 48 cm: 2 × 9.81 × 0.48 = <b>~9.4 Nm</b></li>
      <li>Total <b>15.6 Nm</b> vs. shoulder rated torque <b>30 Nm</b> ✓ (2× margin)</li>
    </ul>
    <p>That's the entire method engineers use to size motors — multiply mass by
    gravity by lever arm, then leave margin.</p>
    <p>The arm test holds both arms straight out horizontally — the worst-case
    lever arm — and checks they don't droop. Early versions of this simulation
    <b>failed</b> this test (drooped ~5°) until the firmware got gravity
    compensation, the same fix real robots use.</p>`,
    actions: [{ id: "arm_raise", label: "▶ Hold arms horizontal" },
              { id: "neck_test", label: "▶ Neck look-at test" }] },

  { id: "full", label: "5 · Full robot", title: "Stage 4 — Integration: it stands, or it doesn't", html: `
    <p>Everything is bolted together and the robot stands on its own feet.
    Standing is not passive: a <b>balance loop</b> watches the center of mass
    and constantly trims the ankles — the "ankle strategy", the same reflex
    you use standing on a bus.</p>
    <h3>What the tests exercise</h3>
    <p><b>Squat</b> is the classic humanoid stress test: knees see near-max
    load, and balance must survive the whole motion. Watch closely: the robot
    <b>leans its torso forward</b> as it goes down — it has to, because its
    ankles only flex 20°, and without the lean its weight would fall behind its
    heels. (During this build, the first squat attempt fell over exactly that
    way. Humans lean for the same reason.)</p>
    <p><b>Power</b>: standing costs ~55 W, a squat ~78 W — the 491 Wh battery
    is good for roughly <b>9 hours</b> of this. Peak current stays under 2 A
    against the 30 A safety limit. Watch voltage sag when motors work hard.</p>
    <p>Every command here is rate-limited by the firmware: an early version
    commanded poses instantly and the current spike <b>tripped the virtual
    BMS at 54 A</b> — the robot "browned out" mid-squat. Trajectory ramping
    fixed it. These are the failures a $16k build teaches you; here they cost
    nothing.</p>`,
    actions: [{ id: "stand_check", label: "▶ Verify standing balance" },
              { id: "squat", label: "▶ Deep squat + recover" },
              { id: "wave", label: "▶ Wave hello" },
              { id: "look_around", label: "▶ Look around" }] },
];

/* ------------------------------------------------ three.js scene */
THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
const view = document.getElementById("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
view.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x14171c);
scene.fog = new THREE.Fog(0x14171c, 6, 14);
const camera = new THREE.PerspectiveCamera(42, 1, 0.05, 100);
camera.position.set(2.2, -1.8, 1.5);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.65);
controls.enableDamping = true;
scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x30281c, 1.1));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(2, -3, 4);
scene.add(sun);
const grid = new THREE.GridHelper(10, 40, 0x3a4150, 0x242a35);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.001;
scene.add(grid);

function resize() {
  const w = view.clientWidth, h = view.clientHeight;
  renderer.setSize(w, h);
  renderer.setPixelRatio(window.devicePixelRatio);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);

let meshes = [];
function buildScene(geoms) {
  for (const m of meshes) scene.remove(m);
  meshes = [];
  for (const g of geoms) {
    let geo = null;
    const [a, b, c] = g.size;
    switch (g.type) {
      case 0: geo = new THREE.PlaneGeometry(20, 20); break;              // plane
      case 2: geo = new THREE.SphereGeometry(a, 24, 16); break;          // sphere
      case 3: geo = new THREE.CapsuleGeometry(a, 2 * b, 8, 16);          // capsule (z axis)
              geo.rotateX(Math.PI / 2); break;
      case 5: geo = new THREE.CylinderGeometry(a, a, 2 * b, 24);         // cylinder
              geo.rotateX(Math.PI / 2); break;
      case 6: geo = new THREE.BoxGeometry(2 * a, 2 * b, 2 * c); break;   // box
      default: geo = new THREE.SphereGeometry(0.02, 8, 8);
    }
    const color = new THREE.Color(g.rgba[0], g.rgba[1], g.rgba[2]);
    const mat = g.type === 0
      ? new THREE.MeshStandardMaterial({ color: 0x232830, roughness: 0.95 })
      : new THREE.MeshStandardMaterial({ color, roughness: 0.55, metalness: 0.25 });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.matrixAutoUpdate = false;
    scene.add(mesh);
    meshes.push(mesh);
  }
}

function updatePoses(pos, mat) {
  for (let i = 0; i < meshes.length && i < pos.length; i++) {
    const p = pos[i], r = mat[i];
    meshes[i].matrix.set(
      r[0], r[1], r[2], p[0],
      r[3], r[4], r[5], p[1],
      r[6], r[7], r[8], p[2],
      0, 0, 0, 1);
  }
}

renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

/* ------------------------------------------------ UI wiring */
const narration = document.getElementById("narration");
const nav = document.getElementById("nav");
const gauges = document.getElementById("gauges");
const resultsEl = document.getElementById("results");
const busyEl = document.getElementById("busy");
let current = 0;
let lastStageOnServer = null;

function renderNav() {
  nav.innerHTML = "";
  const prev = document.createElement("button");
  prev.className = "arrow"; prev.textContent = "◀ Back";
  prev.disabled = current === 0;
  prev.onclick = () => go(current - 1);
  nav.appendChild(prev);
  STAGES.forEach((s, i) => {
    const el = document.createElement("div");
    el.className = "step" + (i === current ? " active" : "");
    el.textContent = s.label;
    el.onclick = () => go(i);
    nav.appendChild(el);
  });
  const next = document.createElement("button");
  next.className = "arrow"; next.textContent = "Next ▶";
  next.disabled = current === STAGES.length - 1;
  next.onclick = () => go(current + 1);
  nav.appendChild(next);
}

function renderNarration() {
  const s = STAGES[current];
  let html = `<h2>${s.title}</h2>${s.html}`;
  narration.innerHTML = html;
  if (s.actions.length) {
    const box = document.createElement("div");
    box.className = "actions";
    for (const a of s.actions) {
      const btn = document.createElement("button");
      btn.textContent = a.label;
      btn.dataset.action = a.id;
      btn.onclick = () => post("/api/action", { action: a.id });
      box.appendChild(btn);
    }
    narration.appendChild(box);
  }
}

async function go(i) {
  current = Math.max(0, Math.min(STAGES.length - 1, i));
  renderNav();
  renderNarration();
  await post("/api/stage", { stage: STAGES[current].id });
  const sc = await (await fetch("/api/scene")).json();
  buildScene(sc.geoms);
  lastStageOnServer = STAGES[current].id;
}

async function post(url, body) {
  return fetch(url, { method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body) });
}

function gauge(label, value) {
  return `<div class="gauge"><div class="v">${value}</div><div class="l">${label}</div></div>`;
}

async function poll() {
  try {
    const st = await (await fetch("/api/state")).json();
    if (st.stage !== lastStageOnServer) {
      const sc = await (await fetch("/api/scene")).json();
      buildScene(sc.geoms);
      lastStageOnServer = st.stage;
    }
    updatePoses(st.pos, st.mat);
    busyEl.style.display = st.busy ? "block" : "none";
    document.querySelectorAll(".actions button")
      .forEach(b => b.disabled = st.busy);
    const t = st.telemetry;
    if (t) {
      let h = gauge("power draw", `${t.power} W`) +
              gauge("bus voltage", `${t.voltage} V`) +
              gauge("battery", `${(t.soc * 100).toFixed(1)}%`) +
              gauge("peak current", `${t.peak_a} A`);
      if (t.head_z !== undefined) h += gauge("head height", `${t.head_z} m`);
      if (t.com_err_mm !== undefined) h += gauge("balance error", `${t.com_err_mm} mm`);
      gauges.innerHTML = h;
    } else {
      gauges.innerHTML = gauge("power draw", "—") + gauge("bus voltage", "—");
    }
    if (st.results.length) {
      resultsEl.innerHTML = st.results.map(r => {
        if (r.name.startsWith("CAN ") && !r.detail)
          return `<div class="res info">${r.name.slice(4)}</div>`;
        return `<div class="res ${r.ok ? "ok" : "fail"}">${r.ok ? "✓" : "✗"} ${r.name}` +
               (r.detail ? ` <span class="d">— ${r.detail}</span>` : "") + `</div>`;
      }).join("");
    } else {
      resultsEl.innerHTML = `<div class="res info">no tests run yet — press a ▶ button</div>`;
    }
  } catch (e) { /* server briefly busy during stage switch */ }
  setTimeout(poll, 50);
}

renderNav();
renderNarration();
go(0).then(poll);
