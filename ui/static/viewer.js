// Three.js viewers.
//
//   PlaneViewer — offline mode: each pose drawn as an outlined quad + normal arrow.
//   CloudViewer — live mode: the raw LiDAR sweep, the ROI box, and the detected
//                 board plane, so you can see *why* a detection failed.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const COLOR_CAM = 0x38bdf8;
const COLOR_LID = 0xfb923c;
const COLOR_ALIGNED = 0x4ade80;
const COLOR_AXES = 0x39465a;
const COLOR_CLOUD = 0x5b6b80;
const COLOR_INLIER = 0x4ade80;
const COLOR_ROI = 0x4fb3ff;

// Shared scaffolding: renderer, camera, orbit controls, resize + animation loop.
class ViewerBase {
  constructor(container, { gridSize = 6, gridDivisions = 12 } = {}) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.renderer.setClearColor(0x080c13, 1);
    container.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.05, 500);
    this.camera.position.set(3.5, -3.5, 2.5);
    // Robotics convention: Z is up, unlike three.js's default Y-up.
    this.camera.up.set(0, 0, 1);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;

    this.scene.add(new THREE.AxesHelper(1.0));
    const grid = new THREE.GridHelper(gridSize, gridDivisions, COLOR_AXES, COLOR_AXES);
    grid.rotation.x = Math.PI / 2;   // three.js grids lie in XZ; we want XY
    grid.material.opacity = 0.35;
    grid.material.transparent = true;
    this.scene.add(grid);

    this._resize = this._resize.bind(this);
    window.addEventListener('resize', this._resize);
    // The container is often hidden at construction time (inactive tab), which
    // reports a zero size; observing it fixes the sizing when it appears.
    this._observer = new ResizeObserver(this._resize);
    this._observer.observe(container);
    this._resize();

    this._animate = this._animate.bind(this);
    this._animate();
  }

  _resize() {
    const w = this.container.clientWidth;
    const h = this.container.clientHeight;
    if (!w || !h) return;
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

  _fitTo(groups) {
    const box = new THREE.Box3();
    groups.forEach((g) => g.children.forEach((obj) => box.expandByObject(obj)));
    if (box.isEmpty()) return;
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    box.getSize(size);
    box.getCenter(center);
    const radius = size.length() * 0.6 + 0.5;
    this.controls.target.copy(center);
    this.camera.position.copy(
      center.clone().add(new THREE.Vector3(radius, -radius, radius * 0.7)));
    this.camera.updateProjectionMatrix();
  }
}

function makeBoard(corners, centroid, normal, color, arrowLen = 0.25) {
  const g = new THREE.Group();

  const positions = [];
  for (const c of corners) positions.push(c[0], c[1], c[2]);
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  g.add(new THREE.LineLoop(geo, new THREE.LineBasicMaterial({ color })));

  const fillGeo = new THREE.BufferGeometry();
  fillGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array([
    ...corners[0], ...corners[1], ...corners[2],
    ...corners[0], ...corners[2], ...corners[3],
  ]), 3));
  fillGeo.computeVertexNormals();
  g.add(new THREE.Mesh(fillGeo, new THREE.MeshBasicMaterial({
    color, transparent: true, opacity: 0.14, side: THREE.DoubleSide,
  })));

  if (centroid && normal) {
    const origin = new THREE.Vector3(...centroid);
    const dir = new THREE.Vector3(...normal).normalize();
    g.add(new THREE.ArrowHelper(dir, origin, arrowLen, color, 0.06, 0.04));
  }
  return g;
}

export class PlaneViewer extends ViewerBase {
  constructor(container) {
    super(container);
    this.cameraGroup = new THREE.Group();
    this.lidarGroup = new THREE.Group();
    this.alignedGroup = new THREE.Group();
    this.scene.add(this.cameraGroup, this.lidarGroup, this.alignedGroup);
  }

  renderInputs(poses) {
    this._clear(this.cameraGroup);
    this._clear(this.lidarGroup);
    this._clear(this.alignedGroup);
    for (const p of poses) {
      this.cameraGroup.add(
        makeBoard(p.camera.corners, p.camera.centroid, p.camera.normal, COLOR_CAM));
      this.lidarGroup.add(
        makeBoard(p.lidar.corners, p.lidar.centroid, p.lidar.normal, COLOR_LID));
    }
    this.setMode('before');
    this._fitTo([this.cameraGroup, this.lidarGroup]);
  }

  // T = { R: 3x3, t: [x,y,z] }, mapping lidar -> camera.
  renderAligned(poses, T) {
    this._clear(this.alignedGroup);
    const { R, t } = T;
    const rot = (v) => [
      R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
      R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
      R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ];
    const apply = (p) => {
      const r = rot(p);
      return [r[0] + t[0], r[1] + t[1], r[2] + t[2]];
    };
    for (const p of poses) {
      this.alignedGroup.add(makeBoard(
        p.lidar.corners.map(apply), apply(p.lidar.centroid),
        rot(p.lidar.normal), COLOR_ALIGNED));
    }
  }

  setMode(mode) {
    const after = mode === 'after';
    this.cameraGroup.visible = true;
    this.lidarGroup.visible = !after;
    this.alignedGroup.visible = after;
  }
}

export class CloudViewer extends ViewerBase {
  constructor(container) {
    super(container, { gridSize: 10, gridDivisions: 20 });
    this.cloudGroup = new THREE.Group();
    this.boardGroup = new THREE.Group();
    this.roiGroup = new THREE.Group();
    this.scene.add(this.cloudGroup, this.boardGroup, this.roiGroup);
    this._framed = false;
  }

  _points(list, color, size) {
    const arr = new Float32Array(list.length * 3);
    list.forEach((p, i) => { arr[i * 3] = p[0]; arr[i * 3 + 1] = p[1]; arr[i * 3 + 2] = p[2]; });
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(arr, 3));
    return new THREE.Points(geo, new THREE.PointsMaterial({ color, size, sizeAttenuation: false }));
  }

  // data = { points, inliers, corners, roi_min, roi_max }
  render(data) {
    this._clear(this.cloudGroup);
    this._clear(this.boardGroup);

    if (data.points?.length) {
      this.cloudGroup.add(this._points(data.points, COLOR_CLOUD, 1.6));
    }
    // Plane inliers are drawn on top and brighter: seeing which points RANSAC
    // latched onto is the fastest way to diagnose a wrong ROI.
    if (data.inliers?.length) {
      this.boardGroup.add(this._points(data.inliers, COLOR_INLIER, 2.6));
    }
    if (data.corners?.length === 4) {
      this.boardGroup.add(makeBoard(data.corners, null, null, COLOR_INLIER));
    }

    if (data.roi_min && data.roi_max) this._renderRoi(data.roi_min, data.roi_max);

    // Fit once only; refitting every poll would fight the user's mouse.
    if (!this._framed && data.points?.length) {
      this._fitTo([this.cloudGroup]);
      this._framed = true;
    }
  }

  _renderRoi(min, max) {
    this._clear(this.roiGroup);
    const size = new THREE.Vector3(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
    const box = new THREE.Box3(new THREE.Vector3(...min), new THREE.Vector3(...max));
    const helper = new THREE.Box3Helper(box, COLOR_ROI);
    helper.material.transparent = true;
    helper.material.opacity = 0.55;
    this.roiGroup.add(helper);
    this._roiSize = size;
  }

  resetView() {
    this._framed = false;
  }
}
