// Three.js viewer for plane observations.
// Draws each board as an outlined quad with a normal arrow from its centroid.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const COLOR_CAM = 0x38bdf8;
const COLOR_LID = 0xfb923c;
const COLOR_ALIGNED = 0x4ade80;
const COLOR_AXES = 0x4a5663;

export class PlaneViewer {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0x060a10, 1);
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 200);
    this.camera.position.set(3.5, -3.5, 2.5);
    this.camera.up.set(0, 0, 1);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    // World axes helper (1m) — red=X, green=Y, blue=Z.
    this.scene.add(new THREE.AxesHelper(1.0));
    // Subtle grid in XY plane.
    const grid = new THREE.GridHelper(6, 12, COLOR_AXES, COLOR_AXES);
    grid.rotation.x = Math.PI / 2;  // grid in XY (default is XZ)
    this.scene.add(grid);

    // Groups for easy clear/toggle.
    this.cameraGroup = new THREE.Group();
    this.lidarGroup = new THREE.Group();
    this.alignedGroup = new THREE.Group(); // lidar after transform
    this.scene.add(this.cameraGroup, this.lidarGroup, this.alignedGroup);

    this._resize = this._resize.bind(this);
    window.addEventListener('resize', this._resize);
    this._resize();

    this._animate = this._animate.bind(this);
    this._animate();
  }

  _resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / Math.max(1, h);
    this.camera.updateProjectionMatrix();
  }

  _animate() {
    requestAnimationFrame(this._animate);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  _clear(group) {
    while (group.children.length) {
      const obj = group.children.pop();
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) obj.material.dispose();
    }
  }

  _makeBoard(corners, centroid, normal, color) {
    const g = new THREE.Group();

    // Outline quad: LineLoop through 4 corners.
    const positions = [];
    for (const c of corners) positions.push(c[0], c[1], c[2]);
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    const lineMat = new THREE.LineBasicMaterial({ color, linewidth: 2 });
    g.add(new THREE.LineLoop(geo, lineMat));

    // Filled translucent quad for body.
    const fillGeo = new THREE.BufferGeometry();
    const verts = new Float32Array([
      ...corners[0], ...corners[1], ...corners[2],
      ...corners[0], ...corners[2], ...corners[3],
    ]);
    fillGeo.setAttribute('position', new THREE.BufferAttribute(verts, 3));
    fillGeo.computeVertexNormals();
    const fillMat = new THREE.MeshBasicMaterial({
      color, transparent: true, opacity: 0.12, side: THREE.DoubleSide,
    });
    g.add(new THREE.Mesh(fillGeo, fillMat));

    // Normal arrow (0.25 m).
    const origin = new THREE.Vector3(centroid[0], centroid[1], centroid[2]);
    const dir = new THREE.Vector3(normal[0], normal[1], normal[2]).normalize();
    const arrow = new THREE.ArrowHelper(dir, origin, 0.25, color, 0.06, 0.04);
    g.add(arrow);

    return g;
  }

  // pose = { camera: {centroid, normal, corners}, lidar: {...} }
  renderInputs(poses) {
    this._clear(this.cameraGroup);
    this._clear(this.lidarGroup);
    this._clear(this.alignedGroup);
    for (const p of poses) {
      this.cameraGroup.add(this._makeBoard(
        p.camera.corners, p.camera.centroid, p.camera.normal, COLOR_CAM));
      this.lidarGroup.add(this._makeBoard(
        p.lidar.corners, p.lidar.centroid, p.lidar.normal, COLOR_LID));
    }
    this.setMode('before');
    this._frameView();
  }

  // T = { R: 3x3 array, t: [tx,ty,tz] }  (lidar -> camera)
  renderAligned(poses, T) {
    this._clear(this.alignedGroup);
    const R = T.R; const tt = T.t;
    const apply = (p) => [
      R[0][0]*p[0] + R[0][1]*p[1] + R[0][2]*p[2] + tt[0],
      R[1][0]*p[0] + R[1][1]*p[1] + R[1][2]*p[2] + tt[1],
      R[2][0]*p[0] + R[2][1]*p[1] + R[2][2]*p[2] + tt[2],
    ];
    const rot = (v) => [
      R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
      R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
      R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ];
    for (const p of poses) {
      const corners = p.lidar.corners.map(apply);
      const centroid = apply(p.lidar.centroid);
      const normal = rot(p.lidar.normal);
      this.alignedGroup.add(this._makeBoard(corners, centroid, normal, COLOR_ALIGNED));
    }
  }

  setMode(mode) {
    if (mode === 'before') {
      this.cameraGroup.visible = true;
      this.lidarGroup.visible = true;
      this.alignedGroup.visible = false;
    } else { // 'after'
      this.cameraGroup.visible = true;
      this.lidarGroup.visible = false;
      this.alignedGroup.visible = true;
    }
  }

  _frameView() {
    // Compute bounding box across visible groups and fit camera.
    const box = new THREE.Box3();
    [this.cameraGroup, this.lidarGroup].forEach((g) => {
      g.children.forEach((obj) => box.expandByObject(obj));
    });
    if (box.isEmpty()) return;
    const size = new THREE.Vector3();
    box.getSize(size);
    const center = new THREE.Vector3();
    box.getCenter(center);
    const radius = size.length() * 0.6 + 0.5;
    this.controls.target.copy(center);
    this.camera.position.copy(center.clone().add(new THREE.Vector3(radius, -radius, radius * 0.7)));
    this.camera.updateProjectionMatrix();
  }
}
