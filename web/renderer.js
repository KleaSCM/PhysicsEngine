const { ipcRenderer } = require('electron');
const THREE = require('three');

// Three.js setup
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
const renderer = new THREE.WebGLRenderer({ canvas: document.getElementById('canvas'), antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x1a1a1a);

// Camera setup
camera.position.set(0, 8, 15);
camera.lookAt(0, 5, 0);

// Lighting
const ambientLight = new THREE.AmbientLight(0x404040);
scene.add(ambientLight);
const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
directionalLight.position.set(1, 1, 1);
scene.add(directionalLight);

// Debug visualization
const debugGroup = new THREE.Group();
scene.add(debugGroup);

// Handle window resize
window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// Handle mouse controls
let isDragging = false;
let previousMousePosition = { x: 0, y: 0 };

document.addEventListener('mousedown', (event) => {
    isDragging = true;
    previousMousePosition = {
        x: event.clientX,
        y: event.clientY
    };
});

document.addEventListener('mousemove', (event) => {
    if (!isDragging) return;

    const deltaMove = {
        x: event.clientX - previousMousePosition.x,
        y: event.clientY - previousMousePosition.y
    };

    // Rotate camera around target
    const distance = camera.position.length();
    const target = new THREE.Vector3(0, 0, 0);
    
    camera.position.x = target.x + distance * Math.sin(camera.rotation.y) * Math.cos(camera.rotation.x);
    camera.position.y = target.y + distance * Math.sin(camera.rotation.x);
    camera.position.z = target.z + distance * Math.cos(camera.rotation.y) * Math.cos(camera.rotation.x);
    
    camera.rotation.y -= deltaMove.x * 0.01;
    camera.rotation.x -= deltaMove.y * 0.01;
    camera.rotation.x = Math.max(-Math.PI/2, Math.min(Math.PI/2, camera.rotation.x));
    
    camera.lookAt(target);

    previousMousePosition = {
        x: event.clientX,
        y: event.clientY
    };
});

document.addEventListener('mouseup', () => {
    isDragging = false;
});

// Handle wheel zoom
document.addEventListener('wheel', (event) => {
    const distance = camera.position.length();
    const target = new THREE.Vector3(0, 0, 0);
    
    const newDistance = Math.max(5, Math.min(50, distance + event.deltaY * 0.1));
    const ratio = newDistance / distance;
    
    camera.position.sub(target).multiplyScalar(ratio).add(target);
});

// UI Controls
document.getElementById('toggleDebug').addEventListener('click', () => {
    ipcRenderer.send('toggle-debug');
});

document.getElementById('toggleColliders').addEventListener('click', () => {
    ipcRenderer.send('toggle-colliders');
});

document.getElementById('toggleGrid').addEventListener('click', () => {
    ipcRenderer.send('toggle-grid');
});

document.getElementById('resetScene').addEventListener('click', () => {
    ipcRenderer.send('reset-scene');
});

document.getElementById('addBox').addEventListener('click', () => {
    ipcRenderer.send('add-box');
});

document.getElementById('addSphere').addEventListener('click', () => {
    ipcRenderer.send('add-sphere');
});

// Handle physics updates from main process
ipcRenderer.on('physics-update', (event, data) => {
    // Update stats
    document.getElementById('fps').textContent = `FPS: ${data.fps.toFixed(1)}`;
    document.getElementById('bodies').textContent = `Bodies: ${data.bodies}`;
    document.getElementById('timeStep').textContent = `Time Step: ${data.timeStep.toFixed(3)}`;

    // Update debug visualization
    updateDebugVisualization(data.debugData);
});

// Update debug visualization
function updateDebugVisualization(debugData) {
    // Clear previous debug objects
    while (debugGroup.children.length > 0) {
        debugGroup.remove(debugGroup.children[0]);
    }

    // Draw lines
    debugData.lines.forEach(line => {
        const geometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(line.start.x, line.start.y, line.start.z),
            new THREE.Vector3(line.end.x, line.end.y, line.end.z)
        ]);
        const material = new THREE.LineBasicMaterial({ color: new THREE.Color(line.color.x, line.color.y, line.color.z) });
        const lineObject = new THREE.Line(geometry, material);
        debugGroup.add(lineObject);
    });

    // Draw points
    debugData.points.forEach(point => {
        const geometry = new THREE.SphereGeometry(point.size, 8, 8);
        const material = new THREE.MeshBasicMaterial({ color: new THREE.Color(point.color.x, point.color.y, point.color.z) });
        const sphere = new THREE.Mesh(geometry, material);
        sphere.position.set(point.position.x, point.position.y, point.position.z);
        debugGroup.add(sphere);
    });
}

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}

animate();
