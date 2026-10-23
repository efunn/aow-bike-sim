// three.js view of the MuJoCo model: one mesh per geom teleop's bare look
// draws (config.render.geoms), posed each frame from geom_xpos / geom_xmat.
// The world is MuJoCo's, z up; the camera is told so rather than the scene
// being rotated, so a position read off `data` goes straight into three.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);

const GEOM = { PLANE: 0, SPHERE: 2, CAPSULE: 3, ELLIPSOID: 4, CYLINDER: 5, BOX: 6, MESH: 7 };

function meshGeometry(m, id) {
  const vadr = m.mesh_vertadr[id], vnum = m.mesh_vertnum[id];
  const fadr = m.mesh_faceadr[id], fnum = m.mesh_facenum[id];
  const vert = m.mesh_vert, face = m.mesh_face;
  // Flat shading: CAD parts and convex pieces read better faceted than with
  // normals averaged across sharp edges.
  const pos = new Float32Array(fnum * 9);
  for (let f = 0; f < fnum; f++) {
    for (let k = 0; k < 3; k++) {
      const v = face[(fadr + f) * 3 + k];
      for (let c = 0; c < 3; c++) pos[f * 9 + k * 3 + c] = vert[(vadr + v) * 3 + c];
    }
  }
  if (vnum === 0) return null;
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
  g.computeVertexNormals();
  return g;
}

function geomGeometry(m, i) {
  const t = m.geom_type[i], s = m.geom_size;
  const a = s[3 * i], b = s[3 * i + 1], c = s[3 * i + 2];
  switch (t) {
    case GEOM.SPHERE: return new THREE.SphereGeometry(a, 24, 16);
    case GEOM.ELLIPSOID: return new THREE.SphereGeometry(1, 24, 16).scale(a, b, c);
    case GEOM.CAPSULE: return new THREE.CapsuleGeometry(a, 2 * b, 8, 24).rotateX(Math.PI / 2);
    case GEOM.CYLINDER: return new THREE.CylinderGeometry(a, a, 2 * b, 32).rotateX(Math.PI / 2);
    case GEOM.BOX: return new THREE.BoxGeometry(2 * a, 2 * b, 2 * c);
    case GEOM.MESH: return meshGeometry(m, m.geom_dataid[i]);
    default: return null;
  }
}

function groundTexture() {
  // 1 m squares with 10 cm lines: enough texture to read speed by.
  const n = 512, cv = document.createElement("canvas");
  cv.width = cv.height = n;
  const g = cv.getContext("2d");
  const css = getComputedStyle(document.documentElement);
  g.fillStyle = css.getPropertyValue("--ground").trim();
  g.fillRect(0, 0, n, n);
  g.strokeStyle = css.getPropertyValue("--grid-minor").trim();
  g.lineWidth = 1;
  for (let i = 1; i < 10; i++) {
    const x = (i * n) / 10;
    g.beginPath(); g.moveTo(x, 0); g.lineTo(x, n); g.stroke();
    g.beginPath(); g.moveTo(0, x); g.lineTo(n, x); g.stroke();
  }
  g.strokeStyle = css.getPropertyValue("--grid-major").trim();
  g.lineWidth = 3;
  g.strokeRect(0, 0, n, n);
  const tex = new THREE.CanvasTexture(cv);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.anisotropy = 8;
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

// The shove arrow at scale 1: a revolved outline, tip at the origin, a 4 cm
// head on a 13 cm shaft, pointing +y. One mesh, so it fades as one object.
function arrowGeometry() {
  const V = (x, y) => new THREE.Vector2(x, y);
  return new THREE.LatheGeometry(
    [V(0, 0), V(0.02, -0.045), V(0.007, -0.045), V(0.007, -0.17), V(0, -0.17)], 24);
}

// A beach ball: six gores alternating the bike's blue and orange, a cap of
// each colour at the poles with a thin seam of the other round it, so its
// spin shows. Unindexed, so each triangle takes its own panel's colour and
// the seams stay sharp.
const CAP_DEG = 17.3;     // polar angle of the cap's edge
const SEAM_DEG = 2.5;     // the seam's width beyond it

function beachBall(r) {
  const g = new THREE.SphereGeometry(r, 96, 120).toNonIndexed();   // 1.5 deg bands
  const pos = g.attributes.position, n = pos.count;
  const col = new Float32Array(n * 3);
  const blue = new THREE.Color().setRGB(0.0, 0.55, 0.76, THREE.SRGBColorSpace);
  const orange = new THREE.Color().setRGB(0.96, 0.46, 0.09, THREE.SRGBColorSpace);
  for (let i = 0; i < n; i += 3) {
    let x = 0, z = 0, lo = Infinity, hi = -Infinity;
    for (let k = 0; k < 3; k++) {
      x += pos.getX(i + k); z += pos.getZ(i + k);
      lo = Math.min(lo, pos.getY(i + k)); hi = Math.max(hi, pos.getY(i + k));
    }
    // SphereGeometry's poles are on its y axis. Caps and seams are judged by
    // the latitude BAND (both triangles of a quad agree), so their edges are
    // circles, not sawteeth.
    const lat = (lo + hi) / 2;
    const fromPole = Math.acos(Math.min(1, Math.abs(lat) / r)) * 180 / Math.PI;
    const top = lat > 0;
    let c;
    if (fromPole < CAP_DEG) c = top ? blue : orange;
    else if (fromPole < CAP_DEG + SEAM_DEG) c = top ? orange : blue;
    else c = Math.floor((Math.atan2(z, x) + Math.PI) / (Math.PI / 3)) % 2 ? orange : blue;
    for (let k = 0; k < 3; k++) col.set([c.r, c.g, c.b], (i + k) * 3);
  }
  g.setAttribute("color", new THREE.BufferAttribute(col, 3));
  // A sphere's normal is its position over r: smooth, seams or no seams.
  const nrm = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) nrm.set([pos.getX(i) / r, pos.getY(i) / r, pos.getZ(i) / r], i * 3);
  g.setAttribute("normal", new THREE.BufferAttribute(nrm, 3));
  return g;
}

// A radial falloff for the ball's blob shadow.
function blobTexture() {
  const n = 64, cv = document.createElement("canvas");
  cv.width = cv.height = n;
  const g = cv.getContext("2d");
  const grad = g.createRadialGradient(n / 2, n / 2, 0, n / 2, n / 2, n / 2);
  grad.addColorStop(0, "rgba(255,255,255,1)");
  grad.addColorStop(0.6, "rgba(255,255,255,0.5)");
  grad.addColorStop(1, "rgba(255,255,255,0)");
  g.fillStyle = grad;
  g.fillRect(0, 0, n, n);
  const t = new THREE.CanvasTexture(cv);
  return t;
}

export class Viewer {
  constructor(canvas, model, cfg) {
    this.m = model;
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color();
    this.scene.fog = new THREE.Fog(this.scene.background, 4, 14);

    this.camera = new THREE.PerspectiveCamera(40, 1, 0.01, 50);
    this.camera.position.set(-0.75, -0.75, 0.45);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(0, 0, 0.08);
    this.controls.enableDamping = true;
    this.controls.minDistance = 0.25;
    this.controls.maxDistance = 4;
    this.controls.maxPolarAngle = Math.PI * 0.49;
    this.controls.enablePan = false;

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 2.2);
    sun.position.set(1.2, -0.8, 2.0);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024, 1024);
    const sc = sun.shadow.camera;
    sc.left = sc.bottom = -0.6; sc.right = sc.top = 0.6; sc.near = 0.1; sc.far = 5;
    sun.shadow.bias = -0.0005;
    this.sun = sun;
    this.scene.add(sun, sun.target);

    this.ground = new THREE.Mesh(
      new THREE.PlaneGeometry(200, 200),
      new THREE.MeshStandardMaterial({ roughness: 0.95 }));
    this.ground.receiveShadow = true;
    this.scene.add(this.ground);

    // Every drawn geom casts, translucent ones included: three draws a
    // caster's shadow solid whatever its opacity, so the bike throws one
    // shadow of its whole shape, and the see-through shell stays a viewing
    // aid rather than something that changes what is on the floor.
    this.geoms = [];
    this.shell = [];         // the see-through parts, which pulse while busy
    const shown = cfg.render.geoms.slice();
    const ball = cfg.render.ball_geom;
    if (!shown.some(([id]) => id === ball)) {
      const c = model.geom_rgba;
      shown.push([ball, c[4 * ball], c[4 * ball + 1], c[4 * ball + 2], 1.0]);
    }
    for (const [id, r, g, b, a] of shown) {
      const geo = id === ball ? beachBall(model.geom_size[3 * ball]) : geomGeometry(model, id);
      if (!geo) continue;
      const mat = new THREE.MeshStandardMaterial({
        color: id === ball ? 0xffffff : new THREE.Color().setRGB(r, g, b, THREE.SRGBColorSpace),
        vertexColors: id === ball,
        roughness: id === ball ? 0.45 : 0.6, metalness: 0.05, flatShading: id !== ball,
        transparent: a < 1, opacity: a, depthWrite: a >= 1,
      });
      const mesh = new THREE.Mesh(geo, mat);
      mesh.matrixAutoUpdate = false;
      // The ball is not in the shadow map: it roams past the map's +-0.6 m
      // around the bike, where its shadow would pop out. It gets a blob.
      mesh.castShadow = id !== ball;
      this.scene.add(mesh);
      this.geoms.push([id, mesh]);
      if (a < 1) this.shell.push([mat, a]);
    }

    // The commanded heading: teleop's green arc on the ground dial, 0.30 m
    // out, +-14 deg wide, riding under the bike -- capsule segments there,
    // so round ends here.
    const R = 0.30, W = 0.008, arc = 14 * Math.PI / 180;
    this.arcMat = new THREE.MeshBasicMaterial();
    this.arc = new THREE.Group();
    this.arc.add(new THREE.Mesh(
      new THREE.TorusGeometry(R, W, 8, 32, 2 * arc).rotateZ(-arc), this.arcMat));
    for (const a of [-arc, arc]) {
      const cap = new THREE.Mesh(new THREE.SphereGeometry(W, 12, 8), this.arcMat);
      cap.position.set(R * Math.cos(a), R * Math.sin(a), 0);
      this.arc.add(cap);
    }
    this.scene.add(this.arc);

    // The shove, drawn as what it is: a force on the chassis from one side.
    // An arrow that drives in toward the centre of mass for exactly as long
    // as the force acts, then fades. ONE mesh, a revolved outline, so it
    // fades as one object rather than as a shaft and a head that blend
    // against each other.
    this.pushMat = new THREE.MeshBasicMaterial({ transparent: true, depthWrite: false });
    this.pushArrow = new THREE.Mesh(arrowGeometry(), this.pushMat);
    this.pushArrow.visible = false;

    // The ball's shadow: a soft dark disc under it, fainter and wider as it
    // leaves the floor.
    this.ballId = ball;
    this.ballR = model.geom_size[3 * ball];
    this.blob = new THREE.Mesh(new THREE.CircleGeometry(1, 32), new THREE.MeshBasicMaterial({
      map: blobTexture(), transparent: true, depthWrite: false, color: 0x000000 }));
    this.scene.add(this.blob);
    this.scene.add(this.pushArrow);
    this.pushFade = 0;

    // The rear wheel's track: teleop's red trail, 2 s solid then 0.5 s
    // fading, as a flat ribbon on the floor.
    this.trailBody = cfg.render.trail_body;
    this.trail = [];                        // [sim time, x, y]
    this.trailMat = new THREE.MeshBasicMaterial({
      vertexColors: true, transparent: true, depthWrite: false, side: THREE.DoubleSide });
    this.trailMesh = new THREE.Mesh(new THREE.BufferGeometry(), this.trailMat);
    this.trailMesh.frustumCulled = false;
    this.scene.add(this.trailMesh);

    this.applyTheme();
    matchMedia("(prefers-color-scheme: dark)").addEventListener("change", () => this.applyTheme());

    this.follow = new THREE.Vector3();
    this.followed = false;
    this.resize();
  }

  // Colours from the page's CSS tokens, again whenever the scheme changes.
  applyTheme() {
    const css = getComputedStyle(document.documentElement);
    const tok = (k) => css.getPropertyValue(k).trim();
    this.scene.background.set(tok("--sky"));
    this.scene.fog.color.set(tok("--sky"));
    const old = this.ground.material.map;
    const tex = groundTexture();
    tex.repeat.set(200, 200);
    this.ground.material.map = tex;
    this.ground.material.needsUpdate = true;
    if (old) old.dispose();
    this.arcMat.color.set(tok("--cmd"));
    this.pushMat.color.set(tok("--impact"));
    this.trailColor = new THREE.Color(tok("--impact"));
    this.accent = new THREE.Color(tok("--accent"));
  }

  resize() {
    const c = this.renderer.domElement;
    const w = c.clientWidth, h = c.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    // Hold the HORIZONTAL field of view in portrait, so a phone or a tall
    // iframe widens the view instead of cropping the bike's sides.
    const fov = 40;
    this.camera.fov = w >= h ? fov
      : Math.min(80, 2 * Math.atan(Math.tan((fov * Math.PI) / 360) * (h / w) * 0.75) * 180 / Math.PI);
    this.camera.updateProjectionMatrix();
  }

  // Pose every drawn geom from `d`, and carry the camera with the bike:
  // the orbit target follows the chassis and the camera moves by the same
  // step, so a drag still orbits and the bike stays framed while driving.
  // The follow is a time constant, not a per-frame fraction, so it trails by
  // the same distance at any frame rate.
  //
  // `busy` pulses the shell (lying down / getting up); `push` is the shove
  // in progress, if any, from BikeSim.push; `com` is the chassis's centre of
  // mass, where the shove acts.
  draw(d, chassisXY, psiCmd, dt, { busy = false, push = null, com = null } = {}) {
    const xp = d.geom_xpos, xm = d.geom_xmat;
    const M = new THREE.Matrix4();
    for (const [id, mesh] of this.geoms) {
      const p = 3 * id, r = 9 * id;
      M.set(xm[r], xm[r + 1], xm[r + 2], xp[p],
            xm[r + 3], xm[r + 4], xm[r + 5], xp[p + 1],
            xm[r + 6], xm[r + 7], xm[r + 8], xp[p + 2],
            0, 0, 0, 1);
      mesh.matrix.copy(M);
    }
    const goal = new THREE.Vector3(chassisXY[0], chassisXY[1], 0.08);
    if (!this.followed) { this.follow.copy(goal); this.followed = true; }
    const step = goal.clone().sub(this.follow).multiplyScalar(1 - Math.exp(-dt / 0.12));
    this.follow.add(step);
    this.controls.target.add(step);
    this.camera.position.add(step);
    this.sun.position.set(this.follow.x + 1.2, this.follow.y - 0.8, 2.0);
    this.sun.target.position.copy(this.follow);
    this.arc.position.set(chassisXY[0], chassisXY[1], 0.004);
    this.arc.rotation.z = psiCmd;

    const now = performance.now() / 1000;
    const k = busy ? 0.5 + 0.5 * Math.sin(now * 2 * Math.PI * 1.2) : 0;
    for (const [mat, a] of this.shell) {
      mat.opacity = a * (1 + 1.6 * k);
      mat.emissive.copy(this.accent).multiplyScalar(0.55 * k);
    }

    if (push && com) {
      // Drives in from 0.30 m to just off the shell over the pulse. The
      // whole arrow scales with the force -- thicker as well as longer, which
      // reads far better than length alone: 0.6x for the nudge, 1.2x for
      // the knockdown.
      this.pushArrow.scale.setScalar(push.force / 2.5);
      const gap = 0.10 + 0.20 * push.left;
      const u = new THREE.Vector3(push.dir[0], push.dir[1], 0).normalize();
      this.pushArrow.position.set(com[0] - u.x * gap, com[1] - u.y * gap, com[2]);
      this.pushArrow.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), u);
      this.pushArrow.visible = true;
      this.pushFade = 1;
    } else if (this.pushFade > 0) {
      this.pushFade = Math.max(0, this.pushFade - dt / 0.35);
      this.pushArrow.visible = this.pushFade > 0;
    }
    this.pushMat.opacity = 0.9 * this.pushFade;
    this.drawTrail(d);

    const bp = 3 * this.ballId, h = Math.max(0, xp[bp + 2] - this.ballR);
    const spread = this.ballR * (0.9 + 2.0 * h);
    this.blob.position.set(xp[bp], xp[bp + 1], 0.003);
    this.blob.scale.setScalar(spread);
    this.blob.material.opacity = 0.45 / (1 + 12 * h);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  drawTrail(d) {
    const SOLID = 2.0, FADE = 0.5, HALF = 0.006;
    const t = d.time, b = this.trailBody, xp = d.xpos;
    const x = xp[3 * b], y = xp[3 * b + 1];
    const tr = this.trail;
    if (tr.length && t < tr[tr.length - 1][0]) tr.length = 0;      // reset
    const last = tr[tr.length - 1];
    if (!last || Math.hypot(x - last[1], y - last[2]) > 0.004) tr.push([t, x, y]);
    while (tr.length && t - tr[0][0] > SOLID + FADE) tr.shift();
    const n = tr.length;
    if (n < 2) { this.trailMesh.visible = false; return; }
    const pos = new Float32Array(n * 6), col = new Float32Array(n * 8);
    const c = this.trailColor;
    for (let i = 0; i < n; i++) {
      const [ti, xi, yi] = tr[i];
      const j = Math.min(i + 1, n - 1), k = Math.max(i - 1, 0);
      let dx = tr[j][1] - tr[k][1], dy = tr[j][2] - tr[k][2];
      const L = Math.hypot(dx, dy) || 1;
      const nx = -dy / L * HALF, ny = dx / L * HALF;
      pos.set([xi + nx, yi + ny, 0.002, xi - nx, yi - ny, 0.002], i * 6);
      const age = t - ti;
      const a = age <= SOLID ? 0.9 : 0.9 * Math.max(0, 1 - (age - SOLID) / FADE);
      col.set([c.r, c.g, c.b, a, c.r, c.g, c.b, a], i * 8);
    }
    const idx = [];
    for (let i = 0; i < n - 1; i++) {
      const a = 2 * i;
      idx.push(a, a + 1, a + 2, a + 1, a + 3, a + 2);
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute("position", new THREE.BufferAttribute(pos, 3));
    g.setAttribute("color", new THREE.BufferAttribute(col, 4));
    g.setIndex(idx);
    this.trailMesh.geometry.dispose();
    this.trailMesh.geometry = g;
    this.trailMesh.visible = true;
  }

  // Snap the camera back over the bike, e.g. after a reset.
  recentre(chassisXY) {
    const off = this.camera.position.clone().sub(this.controls.target);
    this.follow.set(chassisXY[0], chassisXY[1], 0.08);
    this.controls.target.copy(this.follow);
    this.camera.position.copy(this.follow).add(off);
  }
}
