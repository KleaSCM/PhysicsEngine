import * as THREE from 'https://unpkg.com/three@0.160.0/build/three.module.js';
import { OrbitControls } from 'https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js';

// Debug logging
const DEBUG = true;
function log(...args) {
    if (DEBUG) {
        console.log('[Renderer]:', ...args);
    }
}

// Initialize Three.js
const canvas = document.getElementById('canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x1a1a1a);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(0, 10, 20);
camera.lookAt(0, 0, 0);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.05;

// Lighting
const ambientLight = new THREE.AmbientLight(0x404040);
scene.add(ambientLight);

const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
directionalLight.position.set(5, 10, 7.5);
directionalLight.castShadow = true;
directionalLight.shadow.mapSize.width = 2048;
directionalLight.shadow.mapSize.height = 2048;
scene.add(directionalLight);

// Physics objects
const meshes = new Map(); // Store meshes by physics ID

// Create a mesh for a physics body
function createMesh(body) {
    let geometry, material;
    
    if (body.type === 'box') {
        geometry = new THREE.BoxGeometry(body.size.x, body.size.y, body.size.z);
        material = new THREE.MeshStandardMaterial({
            color: 0x00ffff,
            transparent: true,
            opacity: 0.2,
            side: THREE.DoubleSide
        });
    } else if (body.type === 'sphere') {
        geometry = new THREE.SphereGeometry(body.radius, 32, 32);
        material = new THREE.MeshStandardMaterial({ color: 0xff0000 });
    } else {
        return null;
    }

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    return mesh;
}

// Update mesh position and rotation
function updateMesh(mesh, body) {
    mesh.position.set(body.position.x, body.position.y, body.position.z);
    mesh.quaternion.set(body.rotation.x, body.rotation.y, body.rotation.z, body.rotation.w);
}

// Handle window resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Handle physics updates
window.electronAPI.on('physics-update', (event, data) => {
    try {
        // Update stats
        document.getElementById('fps').textContent = `FPS: ${data.fps.toFixed(1)}`;
        document.getElementById('bodies').textContent = `Bodies: ${data.bodies.length}`;
        document.getElementById('timeStep').textContent = `Time Step: ${data.timeStep.toFixed(4)}`;

        // Update or create meshes for each physics body
        data.bodies.forEach(body => {
            let mesh = meshes.get(body.id);
            if (!mesh) {
                mesh = createMesh(body);
                if (mesh) {
                    scene.add(mesh);
                    meshes.set(body.id, mesh);
                }
            }
            if (mesh) {
                updateMesh(mesh, body);
            }
        });

        // Remove meshes for bodies that no longer exist
        const currentIds = new Set(data.bodies.map(b => b.id));
        for (const [id, mesh] of meshes) {
            if (!currentIds.has(id)) {
                scene.remove(mesh);
                meshes.delete(id);
            }
        }

        // Update debug visualization
        updateDebugVisualization(data.debugData);
    } catch (error) {
        log('Error in physics update:', error);
    }
});

// Debug visualization
const debugLines = new THREE.LineSegments(
    new THREE.BufferGeometry(),
    new THREE.LineBasicMaterial({ vertexColors: true })
);
scene.add(debugLines);

const debugPoints = new THREE.Points(
    new THREE.BufferGeometry(),
    new THREE.PointsMaterial({ vertexColors: true, size: 0.1 })
);
scene.add(debugPoints);

function updateDebugVisualization(data) {
    // Update lines
    const linePositions = [];
    const lineColors = [];
    data.lines.forEach(line => {
        linePositions.push(line.start.x, line.start.y, line.start.z);
        linePositions.push(line.end.x, line.end.y, line.end.z);
        lineColors.push(line.color.x, line.color.y, line.color.z);
        lineColors.push(line.color.x, line.color.y, line.color.z);
    });

    const lineGeometry = debugLines.geometry;
    lineGeometry.setAttribute('position', new THREE.Float32BufferAttribute(linePositions, 3));
    lineGeometry.setAttribute('color', new THREE.Float32BufferAttribute(lineColors, 3));
    lineGeometry.attributes.position.needsUpdate = true;
    lineGeometry.attributes.color.needsUpdate = true;

    // Update points
    const pointPositions = [];
    const pointColors = [];
    data.points.forEach(point => {
        pointPositions.push(point.position.x, point.position.y, point.position.z);
        pointColors.push(point.color.x, point.color.y, point.color.z);
    });

    const pointGeometry = debugPoints.geometry;
    pointGeometry.setAttribute('position', new THREE.Float32BufferAttribute(pointPositions, 3));
    pointGeometry.setAttribute('color', new THREE.Float32BufferAttribute(pointColors, 3));
    pointGeometry.attributes.position.needsUpdate = true;
    pointGeometry.attributes.color.needsUpdate = true;
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

// Start animation
animate();

// UI Controls
document.getElementById('toggleDebug').addEventListener('click', () => {
    window.electronAPI.toggleDebug();
});

document.getElementById('toggleColliders').addEventListener('click', () => {
    window.electronAPI.toggleColliders();
});

document.getElementById('toggleGrid').addEventListener('click', () => {
    window.electronAPI.toggleGrid();
});

document.getElementById('resetScene').addEventListener('click', () => {
    window.electronAPI.resetScene();
});

document.getElementById('addBox').addEventListener('click', () => {
    window.electronAPI.addBox();
});

document.getElementById('addSphere').addEventListener('click', () => {
    window.electronAPI.addSphere();
});

// Example of using the exposed electronAPI
window.electronAPI.sendMessage('Hello from renderer!');

// Listen for messages from main process
window.electronAPI.onMessage((event, message) => {
    console.log('Received from main process:', message);
});

// Clean up listener when needed
// window.electronAPI.removeMessageListener(callback);

log('Renderer initialized');
