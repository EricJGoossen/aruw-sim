/**
 * Sentry Mk2 Robot 3D Interactive URDF Visualizer.
 *
 * Implements:
 * - 7-DOF forward kinematics articulation:
 *   * 4 Omni Wheels (continuous 45° spin)
 *   * Turret Major (continuous yaw, bearing deck origin Z = 0.0776m)
 *   * Turret Minor Yaw (revolute yaw, GM6020 deck origin Z = 0.13935m, [-1.5708, +1.5708])
 *   * Turret Minor Pitch (revolute pitch, trunnion shoulder pivot Z = 0.214m, [-0.35, +0.61])
 * - Display modes: Coarse Visual Meshes, Analytical Collision Primitives, Both (superimposed)
 * - Inspection overlays: 3D Joint Axes, RGB Coordinate Frames, Ground Contact Grid (Z=0)
 * - OrbitControls with preset viewpoints (Isometric, Front, Side, Top, Rear)
 * - Joint Reset (q = 0) and smooth automated demo animation sweep
 */

(function () {
  'use strict';

  // ----------------------------------------------------------------------------
  // State and Data Model
  // ----------------------------------------------------------------------------
  const state = {
    mode: 'visual', // 'visual', 'collision', 'both'
    showAxes: true,
    showFrames: true,
    showGrid: true,
    showCOG: false,
    animating: false,
    q: {
      fl_wheel: 0.0,
      fr_wheel: 0.0,
      bl_wheel: 0.0,
      br_wheel: 0.0,
      major_yaw: 0.0,
      minor_yaw: 0.0,
      minor_pitch: 0.0,
    },
  };

  // Robot Physical Specifications
  const SPECS = {
    totalMass: 19.6313,
    linkMasses: {
      base_link: 11.5438,
      fl_wheel: 0.7170,
      fr_wheel: 0.7170,
      bl_wheel: 0.7170,
      br_wheel: 0.7170,
      turret_major: 3.4721,
      turret_minor_frame: 0.7658,
      turret_minor_firing_system: 0.9816,
    },
    cogs: {
      base_link: [-0.003613, -0.004093, 0.057905],
      fl_wheel: [0.008731, 0.008638, 0.000084],
      fr_wheel: [0.008692, -0.008677, 0.000106],
      bl_wheel: [-0.008692, 0.008677, 0.000106],
      br_wheel: [-0.008677, -0.008692, 0.000106],
      turret_major: [0.005345, -0.001700, 0.076253],
      turret_minor_frame: [0.014014, -0.004878, 0.070236],
      turret_minor_firing_system: [0.054018, -0.000068, 0.030540],
    },
  };

  // ----------------------------------------------------------------------------
  // Three.js Core Components
  // ----------------------------------------------------------------------------
  let scene, camera, renderer, controls;
  let gridHelper, contactRingsGroup;
  let axesGroup, framesGroup, cogGroup;
  let visualMeshesGroup, collisionPrimitivesGroup;

  // Joint Groups for Kinematic Articulation
  const jointNodes = {};
  const visualNodes = [];
  const collisionNodes = [];
  const frameHelpers = [];
  const axisArrows = [];
  const cogMarkers = [];

  // Animation Tweening
  let cameraTargetPosition = null;
  let cameraLookTarget = null;

  // ----------------------------------------------------------------------------
  // Initialization
  // ----------------------------------------------------------------------------
  function init() {
    const container = document.getElementById('canvas-container');

    // 1. Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0c0e14);
    scene.fog = new THREE.FogExp2(0x0c0e14, 0.15);

    // 2. Camera: REP-103 (+Z Up, +X Forward, +Y Left)
    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.01, 50.0);
    camera.up.set(0, 0, 1);
    camera.position.set(0.9, -0.9, 0.65);

    // 3. Renderer
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.outputEncoding = THREE.sRGBEncoding;
    container.appendChild(renderer.domElement);

    // 4. Orbit Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.set(0, 0, 0.15);
    controls.minDistance = 0.2;
    controls.maxDistance = 6.0;
    controls.maxPolarAngle = Math.PI / 2 + 0.1; // Allow slightly below horizon

    // 5. Lighting
    setupLighting();

    // 6. Ground Grid & Contact Circle Overlays (Z = 0)
    setupGroundGrid();

    // 7. Overlay Groups
    axesGroup = new THREE.Group();
    framesGroup = new THREE.Group();
    cogGroup = new THREE.Group();
    visualMeshesGroup = new THREE.Group();
    collisionPrimitivesGroup = new THREE.Group();

    scene.add(axesGroup);
    scene.add(framesGroup);
    scene.add(cogGroup);
    scene.add(visualMeshesGroup);
    scene.add(collisionPrimitivesGroup);

    // 8. Build Kinematic Robot Chain
    buildKinematicModel();

    // 9. Attach UI Event Listeners
    setupUIEventListeners();

    // 10. Window Resize Handler
    window.addEventListener('resize', onWindowResize, false);

    // 11. Start Render Loop
    requestAnimationFrame(animate);
  }

  // ----------------------------------------------------------------------------
  // Studio Lighting
  // ----------------------------------------------------------------------------
  function setupLighting() {
    // Ambient / Hemisphere light
    const hemiLight = new THREE.HemisphereLight(0xddeeff, 0x1a202c, 0.65);
    hemiLight.position.set(0, 0, 10);
    scene.add(hemiLight);

    // Key Directional Light
    const keyLight = new THREE.DirectionalLight(0xffffff, 0.85);
    keyLight.position.set(2.0, -2.0, 3.5);
    keyLight.castShadow = true;
    keyLight.shadow.mapSize.width = 2048;
    keyLight.shadow.mapSize.height = 2048;
    keyLight.shadow.camera.near = 0.5;
    keyLight.shadow.camera.far = 10;
    keyLight.shadow.bias = -0.0005;
    const d = 0.8;
    keyLight.shadow.camera.left = -d;
    keyLight.shadow.camera.right = d;
    keyLight.shadow.camera.top = d;
    keyLight.shadow.camera.bottom = -d;
    scene.add(keyLight);

    // Soft Fill Light
    const fillLight = new THREE.DirectionalLight(0x70a0ff, 0.4);
    fillLight.position.set(-2.5, 2.0, 2.0);
    scene.add(fillLight);

    // Blue Accent Rim Light
    const rimLight = new THREE.DirectionalLight(0x00d2ff, 0.35);
    rimLight.position.set(0, 3.0, 0.5);
    scene.add(rimLight);
  }

  // ----------------------------------------------------------------------------
  // Ground Contact Plane & Footprint (Z = 0.0m)
  // ----------------------------------------------------------------------------
  function setupGroundGrid() {
    // GridHelper in Three.js is default in XZ plane; rotate X by 90 deg for XY ground plane
    gridHelper = new THREE.GridHelper(1.6, 32, 0x00d2ff, 0x1f2937);
    gridHelper.rotation.x = Math.PI / 2;
    gridHelper.position.set(0, 0, 0);
    scene.add(gridHelper);

    // Ground plane receiver for soft contact shadows
    const shadowGeo = new THREE.PlaneGeometry(2.5, 2.5);
    const shadowMat = new THREE.ShadowMaterial({ opacity: 0.35 });
    const shadowPlane = new THREE.Mesh(shadowGeo, shadowMat);
    shadowPlane.receiveShadow = true;
    shadowPlane.position.set(0, 0, -0.001);
    scene.add(shadowPlane);

    // Omni Wheel Contact Circles at tangent Z = 0
    contactRingsGroup = new THREE.Group();
    const wheelPositions = [
      [0.15672, 0.15672],
      [0.15672, -0.15672],
      [-0.15672, 0.15672],
      [-0.15672, -0.15672],
    ];

    wheelPositions.forEach(([x, y]) => {
      const ringGeo = new THREE.RingGeometry(0.072, 0.076, 32);
      const ringMat = new THREE.MeshBasicMaterial({
        color: 0x00ff9d,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.45,
      });
      const ring = new THREE.Mesh(ringGeo, ringMat);
      ring.position.set(x, y, 0.001);
      contactRingsGroup.add(ring);
    });
    scene.add(contactRingsGroup);
  }

  // ----------------------------------------------------------------------------
  // Kinematic Tree & Geometry Builder
  // ----------------------------------------------------------------------------
  function buildKinematicModel() {
    // 1. Root: base_footprint (Ground projection at Z = 0.0)
    const baseFootprintGroup = new THREE.Group();
    baseFootprintGroup.name = 'base_footprint';
    scene.add(baseFootprintGroup);
    addFrameHelper(baseFootprintGroup, 0.1);

    // 2. Fixed Joint: base_footprint_joint (Z = +0.062679m)
    const baseLinkGroup = new THREE.Group();
    baseLinkGroup.name = 'base_link';
    baseLinkGroup.position.set(0.0, 0.0, 0.062679);
    baseFootprintGroup.add(baseLinkGroup);
    jointNodes.base_link = baseLinkGroup;
    addFrameHelper(baseLinkGroup, 0.12);
    addCOGMarker(baseLinkGroup, SPECS.cogs.base_link);

    // Chassis Geometries (origin 0, 0, 0 in base_link frame)
    const chassisVisGroup = new THREE.Group();
    chassisVisGroup.position.set(0.0, 0.0, 0.0);
    baseLinkGroup.add(chassisVisGroup);

    // Chassis Visual Mesh
    loadSTLMesh(
      'chassis.stl',
      chassisVisGroup,
      new THREE.MeshStandardMaterial({
        color: 0x384152,
        roughness: 0.35,
        metalness: 0.45,
        flatShading: true,
        side: THREE.DoubleSide,
      })
    );

    // Chassis Collision Mesh
    loadCollisionSTLMesh('chassis.stl', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0x00d2ff, baseLinkGroup);

    // --------------------------------------------------------------------------
    // 3. Four Omni Wheel Joints (Continuous 45° spin)
    // --------------------------------------------------------------------------
    const wheels = [
      {
        name: 'fl_wheel',
        joint: 'joint_fl_wheel',
        origin: [0.15672, 0.15672, 0.013181],
        axis: new THREE.Vector3(0.707107, 0.707107, 0.0),
        colRPY: [0.0, 1.570796, 0.785398],
      },
      {
        name: 'fr_wheel',
        joint: 'joint_fr_wheel',
        origin: [0.15672, -0.15672, 0.013181],
        axis: new THREE.Vector3(0.707107, -0.707107, 0.0),
        colRPY: [0.0, 1.570796, -0.785398],
      },
      {
        name: 'bl_wheel',
        joint: 'joint_bl_wheel',
        origin: [-0.15672, 0.15672, 0.013181],
        axis: new THREE.Vector3(-0.707107, 0.707107, 0.0),
        colRPY: [0.0, 1.570796, 2.356194],
      },
      {
        name: 'br_wheel',
        joint: 'joint_br_wheel',
        origin: [-0.15672, -0.15672, 0.013181],
        axis: new THREE.Vector3(-0.707107, -0.707107, 0.0),
        colRPY: [0.0, 1.570796, -2.356194],
      },
    ];

    wheels.forEach((w) => {
      // Joint Mount Pivot
      const jointGroup = new THREE.Group();
      jointGroup.name = w.joint;
      jointGroup.position.set(...w.origin);
      baseLinkGroup.add(jointGroup);
      jointNodes[w.joint] = jointGroup;
      jointNodes[w.joint]._axis = w.axis.clone().normalize();

      // Inspection Helpers
      addFrameHelper(jointGroup, 0.06);
      addAxisArrow(baseLinkGroup, w.origin, w.axis, 0.11, 0xffaa00);
      addCOGMarker(jointGroup, SPECS.cogs[w.name]);

      // Wheel Visual Mesh (omni_wheel.stl)
      const wheelVisGroup = new THREE.Group();
      if (w.colRPY) {
        wheelVisGroup.rotation.set(w.colRPY[0], w.colRPY[1], w.colRPY[2], 'ZYX');
      }
      jointGroup.add(wheelVisGroup);
      loadSTLMesh(
        'omni_wheel.stl',
        wheelVisGroup,
        new THREE.MeshStandardMaterial({
          color: 0x2b313d,
          roughness: 0.55,
          metalness: 0.35,
          flatShading: true,
          side: THREE.DoubleSide,
        })
      );

      // Wheel Collision Mesh (omni_wheel.stl)
      loadCollisionSTLMesh('omni_wheel.stl', [0.0, 0.0, 0.0], w.colRPY, 0xff9d00, jointGroup);
    });

    // --------------------------------------------------------------------------
    // 4. Turret Major Yaw Joint (continuous yaw, bearing deck Z = 0.0776m)
    // --------------------------------------------------------------------------
    const jointMajorGroup = new THREE.Group();
    jointMajorGroup.name = 'joint_major';
    jointMajorGroup.position.set(0.0, 0.0, 0.0776);
    baseLinkGroup.add(jointMajorGroup);
    jointNodes.joint_major = jointMajorGroup;
    jointNodes.joint_major._axis = new THREE.Vector3(0, 0, 1);

    addFrameHelper(jointMajorGroup, 0.08);
    addAxisArrow(baseLinkGroup, [0.0, 0.0, 0.0776], new THREE.Vector3(0, 0, 1), 0.14, 0x00d2ff);
    addCOGMarker(jointMajorGroup, SPECS.cogs.turret_major);

    // Turret Major Visual Mesh (origin 0, 0, 0 in turret_major frame)
    const majorVisGroup = new THREE.Group();
    majorVisGroup.position.set(0.0, 0.0, 0.0);
    jointMajorGroup.add(majorVisGroup);
    loadSTLMesh(
      'turret_major.stl',
      majorVisGroup,
      new THREE.MeshStandardMaterial({
        color: 0x4c566a,
        roughness: 0.35,
        metalness: 0.5,
        flatShading: true,
        side: THREE.DoubleSide,
      })
    );

    // Turret Major Collision Mesh
    loadCollisionSTLMesh('turret_major.stl', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0x7952ff, jointMajorGroup);

    // --------------------------------------------------------------------------
    // 5. Turret Minor Yaw Joint (revolute yaw, GM6020 deck origin Z = 0.13935m)
    // --------------------------------------------------------------------------
    const jointMinorYawGroup = new THREE.Group();
    jointMinorYawGroup.name = 'joint_minor_yaw';
    jointMinorYawGroup.position.set(0.0, 0.0, 0.13935);
    jointMajorGroup.add(jointMinorYawGroup);
    jointNodes.joint_minor_yaw = jointMinorYawGroup;
    jointNodes.joint_minor_yaw._axis = new THREE.Vector3(0, 0, 1);

    addFrameHelper(jointMinorYawGroup, 0.07);
    addAxisArrow(jointMajorGroup, [0.0, 0.0, 0.13935], new THREE.Vector3(0, 0, 1), 0.12, 0x00ff9d);
    addCOGMarker(jointMinorYawGroup, SPECS.cogs.turret_minor_frame);

    // Turret Minor Frame Visual Mesh (origin 0, 0, 0 in turret_minor_frame)
    const minorVisGroup = new THREE.Group();
    minorVisGroup.position.set(0.0, 0.0, 0.0);
    jointMinorYawGroup.add(minorVisGroup);
    loadSTLMesh(
      'turret_minor_frame.stl',
      minorVisGroup,
      new THREE.MeshStandardMaterial({
        color: 0x2e3440,
        roughness: 0.4,
        metalness: 0.4,
        flatShading: true,
        side: THREE.DoubleSide,
      })
    );

    // Turret Minor Frame Collision Mesh
    loadCollisionSTLMesh('turret_minor_frame.stl', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0x00ffd0, jointMinorYawGroup);

    // --------------------------------------------------------------------------
    // 6. Turret Minor Pitch Joint (revolute pitch, shoulder screw Z = 0.214m)
    // --------------------------------------------------------------------------
    const jointMinorPitchGroup = new THREE.Group();
    jointMinorPitchGroup.name = 'joint_minor_pitch';
    jointMinorPitchGroup.position.set(0.0, 0.0, 0.214);
    jointMinorYawGroup.add(jointMinorPitchGroup);
    jointNodes.joint_minor_pitch = jointMinorPitchGroup;
    jointNodes.joint_minor_pitch._axis = new THREE.Vector3(0, 1, 0); // +Y trunnion axis

    addFrameHelper(jointMinorPitchGroup, 0.07);
    addAxisArrow(jointMinorYawGroup, [0.0, 0.0, 0.214], new THREE.Vector3(0, 1, 0), 0.14, 0xff0077);
    addCOGMarker(jointMinorPitchGroup, SPECS.cogs.turret_minor_firing_system);

    // Firing System Visual Mesh (origin 0, 0, 0 in turret_minor_firing_system)
    const firingVisGroup = new THREE.Group();
    firingVisGroup.position.set(0.0, 0.0, 0.0);
    jointMinorPitchGroup.add(firingVisGroup);
    loadSTLMesh(
      'turret_minor_firing_system.stl',
      firingVisGroup,
      new THREE.MeshStandardMaterial({
        color: 0x9aa5b4,
        roughness: 0.3,
        metalness: 0.6,
        flatShading: true,
        side: THREE.DoubleSide,
      })
    );

    // Firing System Collision Mesh
    loadCollisionSTLMesh('turret_minor_firing_system.stl', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0xff4757, jointMinorPitchGroup);
  }

  // ----------------------------------------------------------------------------
  // STL Mesh Loader
  // ----------------------------------------------------------------------------
  let pendingMeshesCount = 5;

  function loadSTLMesh(filename, parentGroup, material) {
    const loader = new THREE.STLLoader();
    // Resolve relative path with cache-busting query string so browser never serves stale cached STLs
    const url = `../urdf/meshes/visual/${filename}?v=${Date.now()}`;

    loader.load(
      url,
      (geometry) => {
        geometry.computeVertexNormals();
        const mesh = new THREE.Mesh(geometry, material);
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        mesh._baseMaterial = material;
        parentGroup.add(mesh);
        visualNodes.push(mesh);

        pendingMeshesCount--;
        if (pendingMeshesCount <= 0) {
          hideLoadingOverlay();
        }
      },
      undefined,
      (error) => {
        console.warn(`Could not load STL mesh ${filename} via relative path, trying fallback:`, error);
        // Fallback to absolute workspace path
        const fallbackUrl = `/urdf/meshes/visual/${filename}?v=${Date.now()}`;
        loader.load(
          fallbackUrl,
          (geometry) => {
            geometry.computeVertexNormals();
            const mesh = new THREE.Mesh(geometry, material);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            mesh._baseMaterial = material;
            parentGroup.add(mesh);
            visualNodes.push(mesh);
            pendingMeshesCount--;
            if (pendingMeshesCount <= 0) hideLoadingOverlay();
          },
          undefined,
          (err2) => {
            console.error(`Failed to load mesh ${filename}:`, err2);
            pendingMeshesCount--;
            if (pendingMeshesCount <= 0) hideLoadingOverlay();
          }
        );
      }
    );
  }

  function hideLoadingOverlay() {
    const overlay = document.getElementById('loading-overlay');
    if (overlay) {
      overlay.classList.add('fade-out');
      setTimeout(() => {
        overlay.style.display = 'none';
      }, 500);
    }
  }

  // ----------------------------------------------------------------------------
  // Collision Convex-Hull STL Mesh Loader
  // ----------------------------------------------------------------------------
  function loadCollisionSTLMesh(filename, originXYZ, originRPY, hexColor, parentGroup) {
    const loader = new THREE.STLLoader();
    const url = `../urdf/meshes/collision/${filename}?v=${Date.now()}`;

    const onLoaded = (geometry) => {
      geometry.computeVertexNormals();
      const mat = new THREE.MeshStandardMaterial({
        color: hexColor,
        roughness: 0.3,
        metalness: 0.1,
        transparent: true,
        opacity: 0.85,
      });
      const mesh = new THREE.Mesh(geometry, mat);
      mesh.position.set(...originXYZ);
      if (originRPY) {
        mesh.rotation.set(originRPY[0], originRPY[1], originRPY[2], 'ZYX');
      }

      const wireGeo = new THREE.EdgesGeometry(geometry, 15);
      const wireMat = new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 1.0, transparent: true, opacity: 0.5 });
      const wire = new THREE.LineSegments(wireGeo, wireMat);
      mesh.add(wire);

      mesh.visible = state.mode === 'collision' || state.mode === 'both';
      parentGroup.add(mesh);
      collisionNodes.push(mesh);
    };

    loader.load(url, onLoaded, undefined, () => {
      loader.load(`/urdf/meshes/collision/${filename}`, onLoaded);
    });
  }

  // ----------------------------------------------------------------------------
  // Analytical Collision Primitives (Boxes & Cylinders)
  // ----------------------------------------------------------------------------
  function createBoxPrimitive(size, originXYZ, originRPY, hexColor, parentGroup) {
    const geo = new THREE.BoxGeometry(size[0], size[1], size[2]);
    const mat = new THREE.MeshStandardMaterial({
      color: hexColor,
      roughness: 0.3,
      metalness: 0.1,
      transparent: true,
      opacity: 0.85,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(...originXYZ);

    if (originRPY) {
      mesh.rotation.set(originRPY[0], originRPY[1], originRPY[2], 'ZYX');
    }

    // Add bright edge wireframe for crisp outline
    const wireGeo = new THREE.EdgesGeometry(geo);
    const wireMat = new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 1.5 });
    const wire = new THREE.LineSegments(wireGeo, wireMat);
    mesh.add(wire);

    mesh.visible = false; // default visual mode hides collision
    parentGroup.add(mesh);
    collisionNodes.push(mesh);
    return mesh;
  }

  function createCylinderPrimitive(radius, length, originXYZ, originRPY, hexColor, parentGroup) {
    // In Three.js, cylinder height is along Y by default.
    // In URDF, cylinder length is along Z! Rotate cylinder to align height with Z.
    const geo = new THREE.CylinderGeometry(radius, radius, length, 32);
    geo.rotateX(Math.PI / 2); // align with local Z

    const mat = new THREE.MeshStandardMaterial({
      color: hexColor,
      roughness: 0.3,
      metalness: 0.1,
      transparent: true,
      opacity: 0.85,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(...originXYZ);

    if (originRPY) {
      mesh.rotation.set(originRPY[0], originRPY[1], originRPY[2], 'ZYX');
    }

    const wireGeo = new THREE.EdgesGeometry(geo);
    const wireMat = new THREE.LineBasicMaterial({ color: 0xffffff, linewidth: 1.5 });
    const wire = new THREE.LineSegments(wireGeo, wireMat);
    mesh.add(wire);

    mesh.visible = false;
    parentGroup.add(mesh);
    collisionNodes.push(mesh);
    return mesh;
  }

  // ----------------------------------------------------------------------------
  // Inspection Overlays (Axes, Coordinate Frames, COG)
  // ----------------------------------------------------------------------------
  function addFrameHelper(parentGroup, size) {
    const axes = new THREE.AxesHelper(size);
    axes.material.linewidth = 2;
    axes.renderOrder = 2;
    axes.visible = state.showFrames;
    parentGroup.add(axes);
    frameHelpers.push(axes);
  }

  function addAxisArrow(parentGroup, origin, axisVec, length, hexColor) {
    const dir = axisVec.clone().normalize();
    const arrow = new THREE.ArrowHelper(dir, new THREE.Vector3(...origin), length, hexColor, 0.025, 0.015);
    arrow.line.material.linewidth = 3;
    arrow.visible = state.showAxes;
    parentGroup.add(arrow);
    axisArrows.push(arrow);
  }

  function addCOGMarker(parentGroup, cog) {
    const geo = new THREE.SphereGeometry(0.014, 16, 16);
    const mat = new THREE.MeshBasicMaterial({ color: 0x00ff9d, wireframe: true });
    const sphere = new THREE.Mesh(geo, mat);
    sphere.position.set(...cog);
    sphere.visible = state.showCOG;
    parentGroup.add(sphere);
    cogMarkers.push(sphere);
  }

  // ----------------------------------------------------------------------------
  // Articulation Updates
  // ----------------------------------------------------------------------------
  function updateJoints() {
    // 1. Omni Wheels
    ['fl_wheel', 'fr_wheel', 'bl_wheel', 'br_wheel'].forEach((w) => {
      const jName = `joint_${w}`;
      const node = jointNodes[jName];
      if (node && node._axis) {
        node.setRotationFromAxisAngle(node._axis, state.q[w]);
      }
    });

    // 2. Turret Major Yaw (around +Z)
    if (jointNodes.joint_major) {
      jointNodes.joint_major.setRotationFromAxisAngle(new THREE.Vector3(0, 0, 1), state.q.major_yaw);
    }

    // 3. Turret Minor Yaw (around +Z)
    if (jointNodes.joint_minor_yaw) {
      jointNodes.joint_minor_yaw.setRotationFromAxisAngle(new THREE.Vector3(0, 0, 1), state.q.minor_yaw);
    }

    // 4. Turret Minor Pitch (around +Y)
    if (jointNodes.joint_minor_pitch) {
      jointNodes.joint_minor_pitch.setRotationFromAxisAngle(new THREE.Vector3(0, 1, 0), state.q.minor_pitch);
    }
  }

  function updateSliderLabels() {
    const toDeg = (r) => ((r * 180) / Math.PI).toFixed(1);

    document.getElementById('val-fl-wheel').textContent = `${state.q.fl_wheel.toFixed(2)} rad (${toDeg(state.q.fl_wheel)}°)`;
    document.getElementById('val-fr-wheel').textContent = `${state.q.fr_wheel.toFixed(2)} rad (${toDeg(state.q.fr_wheel)}°)`;
    document.getElementById('val-bl-wheel').textContent = `${state.q.bl_wheel.toFixed(2)} rad (${toDeg(state.q.bl_wheel)}°)`;
    document.getElementById('val-br-wheel').textContent = `${state.q.br_wheel.toFixed(2)} rad (${toDeg(state.q.br_wheel)}°)`;
    document.getElementById('val-major-yaw').textContent = `${state.q.major_yaw.toFixed(2)} rad (${toDeg(state.q.major_yaw)}°)`;
    document.getElementById('val-minor-yaw').textContent = `${state.q.minor_yaw.toFixed(2)} rad (${toDeg(state.q.minor_yaw)}°)`;
    document.getElementById('val-minor-pitch').textContent = `${state.q.minor_pitch.toFixed(2)} rad (${toDeg(state.q.minor_pitch)}°)`;
  }

  function resetJoints() {
    Object.keys(state.q).forEach((k) => (state.q[k] = 0.0));

    document.getElementById('slider-fl-wheel').value = 0.0;
    document.getElementById('slider-fr-wheel').value = 0.0;
    document.getElementById('slider-bl-wheel').value = 0.0;
    document.getElementById('slider-br-wheel').value = 0.0;
    document.getElementById('slider-major-yaw').value = 0.0;
    document.getElementById('slider-minor-yaw').value = 0.0;
    document.getElementById('slider-minor-pitch').value = 0.0;

    updateJoints();
    updateSliderLabels();
  }

  // ----------------------------------------------------------------------------
  // Geometry Display Modes
  // ----------------------------------------------------------------------------
  function setGeometryMode(mode) {
    state.mode = mode;
    const hudMode = document.getElementById('hud-geom-mode');

    document.getElementById('btn-mode-visual').classList.toggle('active', mode === 'visual');
    document.getElementById('btn-mode-collision').classList.toggle('active', mode === 'collision');
    document.getElementById('btn-mode-both').classList.toggle('active', mode === 'both');

    if (mode === 'visual') {
      hudMode.textContent = 'Coarse Visual Meshes';
      visualNodes.forEach((mesh) => {
        mesh.visible = true;
        mesh.material.transparent = false;
        mesh.material.opacity = 1.0;
      });
      collisionNodes.forEach((col) => {
        col.visible = false;
      });
    } else if (mode === 'collision') {
      hudMode.textContent = 'Analytical Collision Primitives';
      visualNodes.forEach((mesh) => {
        mesh.visible = false;
      });
      collisionNodes.forEach((col) => {
        col.visible = true;
        col.material.transparent = true;
        col.material.opacity = 0.85;
      });
    } else if (mode === 'both') {
      hudMode.textContent = 'Visual & Collision Superimposed';
      visualNodes.forEach((mesh) => {
        mesh.visible = true;
        mesh.material.transparent = true;
        mesh.material.opacity = 0.55;
      });
      collisionNodes.forEach((col) => {
        col.visible = true;
        col.material.transparent = true;
        col.material.opacity = 0.85;
      });
    }
  }

  // ----------------------------------------------------------------------------
  // Viewpoint Presets
  // ----------------------------------------------------------------------------
  function setViewpoint(name) {
    const target = new THREE.Vector3(0, 0, 0.15);
    let pos;

    switch (name) {
      case 'iso':
        pos = new THREE.Vector3(0.95, -0.95, 0.65);
        break;
      case 'front':
        pos = new THREE.Vector3(1.4, 0.0, 0.2);
        break;
      case 'side':
        pos = new THREE.Vector3(0.0, -1.4, 0.2);
        break;
      case 'top':
        pos = new THREE.Vector3(0.001, 0.0, 1.45);
        break;
      case 'rear':
        pos = new THREE.Vector3(-1.4, 0.0, 0.2);
        break;
      default:
        pos = new THREE.Vector3(0.95, -0.95, 0.65);
    }

    cameraTargetPosition = pos;
    cameraLookTarget = target;
  }

  // ----------------------------------------------------------------------------
  // Motion Demo Sweep Animation
  // ----------------------------------------------------------------------------
  function toggleAnimation() {
    state.animating = !state.animating;
    const btn = document.getElementById('btn-animate-demo');
    const icon = document.getElementById('anim-icon');
    const text = document.getElementById('anim-text');

    if (state.animating) {
      btn.classList.add('active');
      icon.textContent = '⏸';
      text.textContent = 'Pause Motion Demo';
    } else {
      btn.classList.remove('active');
      icon.textContent = '▶';
      text.textContent = 'Play Motion Demo';
    }
  }

  // ----------------------------------------------------------------------------
  // UI Event Listeners
  // ----------------------------------------------------------------------------
  function setupUIEventListeners() {
    // Mode Buttons
    document.getElementById('btn-mode-visual').addEventListener('click', () => setGeometryMode('visual'));
    document.getElementById('btn-mode-collision').addEventListener('click', () => setGeometryMode('collision'));
    document.getElementById('btn-mode-both').addEventListener('click', () => setGeometryMode('both'));

    // Viewpoint Buttons
    document.getElementById('vp-iso').addEventListener('click', () => setViewpoint('iso'));
    document.getElementById('vp-front').addEventListener('click', () => setViewpoint('front'));
    document.getElementById('vp-side').addEventListener('click', () => setViewpoint('side'));
    document.getElementById('vp-top').addEventListener('click', () => setViewpoint('top'));
    document.getElementById('vp-rear').addEventListener('click', () => setViewpoint('rear'));

    // Sliders
    const bindSlider = (sliderId, stateKey) => {
      const el = document.getElementById(sliderId);
      el.addEventListener('input', (e) => {
        state.q[stateKey] = parseFloat(e.target.value);
        updateJoints();
        updateSliderLabels();
      });
    };

    bindSlider('slider-fl-wheel', 'fl_wheel');
    bindSlider('slider-fr-wheel', 'fr_wheel');
    bindSlider('slider-bl-wheel', 'bl_wheel');
    bindSlider('slider-br-wheel', 'br_wheel');
    bindSlider('slider-major-yaw', 'major_yaw');
    bindSlider('slider-minor-yaw', 'minor_yaw');
    bindSlider('slider-minor-pitch', 'minor_pitch');

    // Overlays
    document.getElementById('toggle-axes').addEventListener('change', (e) => {
      state.showAxes = e.target.checked;
      axisArrows.forEach((a) => (a.visible = state.showAxes));
    });

    document.getElementById('toggle-frames').addEventListener('change', (e) => {
      state.showFrames = e.target.checked;
      frameHelpers.forEach((f) => (f.visible = state.showFrames));
    });

    document.getElementById('toggle-grid').addEventListener('change', (e) => {
      state.showGrid = e.target.checked;
      gridHelper.visible = state.showGrid;
      contactRingsGroup.visible = state.showGrid;
    });

    document.getElementById('toggle-cog').addEventListener('change', (e) => {
      state.showCOG = e.target.checked;
      cogMarkers.forEach((c) => (c.visible = state.showCOG));
    });

    // Action Buttons
    document.getElementById('btn-animate-demo').addEventListener('click', toggleAnimation);
    document.getElementById('btn-reset-joints').addEventListener('click', resetJoints);

    // Sidebar Collapse / Expand
    const sidebar = document.getElementById('controls-sidebar');
    const openBtn = document.getElementById('btn-open-sidebar');
    const closeBtn = document.getElementById('btn-close-sidebar');

    closeBtn.addEventListener('click', () => {
      sidebar.classList.add('collapsed');
      openBtn.style.display = 'flex';
    });

    openBtn.addEventListener('click', () => {
      sidebar.classList.remove('collapsed');
      openBtn.style.display = 'none';
    });

    // Keyboard Shortcuts
    window.addEventListener('keydown', (e) => {
      if (e.target.tagName === 'INPUT') return;
      const key = e.key.toLowerCase();
      if (key === 'r') resetJoints();
      else if (key === ' ') {
        e.preventDefault();
        toggleAnimation();
      } else if (key === '1') setGeometryMode('visual');
      else if (key === '2') setGeometryMode('collision');
      else if (key === '3') setGeometryMode('both');
      else if (key === 'i') setViewpoint('iso');
      else if (key === 'f') setViewpoint('front');
      else if (key === 's') setViewpoint('side');
      else if (key === 't') setViewpoint('top');
    });
  }

  function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  }

  // ----------------------------------------------------------------------------
  // Render & Animation Loop
  // ----------------------------------------------------------------------------
  let clock = new THREE.Clock();

  function animate() {
    requestAnimationFrame(animate);

    const delta = clock.getDelta();
    const t = clock.getElapsedTime();

    // Smooth Camera Transition
    if (cameraTargetPosition) {
      camera.position.lerp(cameraTargetPosition, 0.08);
      if (cameraLookTarget) {
        controls.target.lerp(cameraLookTarget, 0.08);
      }
      if (camera.position.distanceTo(cameraTargetPosition) < 0.005) {
        cameraTargetPosition = null;
        cameraLookTarget = null;
      }
    }

    // Motion Demo Sweep
    if (state.animating) {
      // 1. Wheels spin continuously
      const wheelSpinRate = 4.0; // rad/s
      state.q.fl_wheel = (state.q.fl_wheel + wheelSpinRate * delta) % (Math.PI * 2);
      state.q.fr_wheel = (state.q.fr_wheel + wheelSpinRate * delta) % (Math.PI * 2);
      state.q.bl_wheel = (state.q.bl_wheel + wheelSpinRate * delta) % (Math.PI * 2);
      state.q.br_wheel = (state.q.br_wheel + wheelSpinRate * delta) % (Math.PI * 2);

      // 2. Turret Major Yaw: continuous 360 degree pan
      state.q.major_yaw = Math.sin(t * 0.6) * Math.PI;

      // 3. Turret Minor Yaw: ±75 deg oscillation (within ±90 deg limits)
      state.q.minor_yaw = Math.sin(t * 1.2) * 1.3;

      // 4. Turret Minor Pitch: -15 deg to +30 deg oscillation (within [-0.26, +0.61])
      state.q.minor_pitch = 0.15 + Math.sin(t * 1.5) * 0.35;

      // Update Slider Elements
      document.getElementById('slider-fl-wheel').value = state.q.fl_wheel;
      document.getElementById('slider-fr-wheel').value = state.q.fr_wheel;
      document.getElementById('slider-bl-wheel').value = state.q.bl_wheel;
      document.getElementById('slider-br-wheel').value = state.q.br_wheel;
      document.getElementById('slider-major-yaw').value = state.q.major_yaw;
      document.getElementById('slider-minor-yaw').value = state.q.minor_yaw;
      document.getElementById('slider-minor-pitch').value = state.q.minor_pitch;

      updateJoints();
      updateSliderLabels();
    }

    controls.update();
    renderer.render(scene, camera);
  }

  // Start initialization when DOM is loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
