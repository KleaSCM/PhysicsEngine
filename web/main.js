const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');
const Physics = require('../build/Release/physics.node');

let mainWindow;
let physicsEngine;

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 1200,
        height: 800,
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false
        }
    });

    mainWindow.loadFile('index.html');
    // mainWindow.webContents.openDevTools(); // Uncomment for debugging
}

app.whenReady().then(() => {
    createWindow();

    // Initialize physics engine
    physicsEngine = new Physics.Engine();
    physicsEngine.Initialize();

    // Create initial scene
    physicsEngine.CreatePlane({ x: 0, y: 1, z: 0 }, 0.0); // Ground plane
    
    // Create the rotating cube (static)
    physicsEngine.CreateBox({ x: 0, y: 5, z: 0 }, { x: 2, y: 2, z: 2 });
    
    // Create the ball inside the cube (dynamic)
    physicsEngine.CreateSphere({ x: 0, y: 5, z: 0 }, 0.5);
    
    // Set up the rotating cube constraint
    physicsEngine.CreateHingeConstraint(
        { x: 0, y: 5, z: 0 },  // Pivot point
        { x: 0, y: 1, z: 0 },  // Axis
        0.0,                    // Angular velocity (will be set in update loop)
        true                    // Is rotating
    );

    // Start physics update loop
    let lastTime = Date.now();
    let rotationAngle = 0.0;
    const rotationSpeed = 1.0; // radians per second

    function updatePhysics() {
        const currentTime = Date.now();
        const deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
        lastTime = currentTime;

        // Update rotation angle
        rotationAngle += rotationSpeed * deltaTime;
        if (rotationAngle > 2 * Math.PI) {
            rotationAngle -= 2 * Math.PI;
        }

        // Apply rotation to the cube
        physicsEngine.SetHingeConstraintRotation(0, rotationAngle);

        physicsEngine.Update(deltaTime);

        // Send physics data to renderer
        const debugData = physicsEngine.GetDebugDrawData();
        mainWindow.webContents.send('physics-update', {
            fps: physicsEngine.GetAverageFPS(),
            bodies: physicsEngine.GetWorld().GetBodyCount(),
            timeStep: physicsEngine.GetSettings().fixedTimeStep,
            debugData: {
                lines: debugData.lines.map(line => ({
                    start: { x: line.start.x, y: line.start.y, z: line.start.z },
                    end: { x: line.end.x, y: line.end.y, z: line.end.z },
                    color: { x: line.color.x, y: line.color.y, z: line.color.z }
                })),
                points: debugData.points.map(point => ({
                    position: { x: point.position.x, y: point.position.y, z: point.position.z },
                    color: { x: point.color.x, y: point.color.y, z: point.color.z },
                    size: point.size
                }))
            }
        });

        requestAnimationFrame(updatePhysics);
    }
    updatePhysics();
});

app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
        createWindow();
    }
});

// Handle IPC messages from renderer
ipcMain.on('toggle-debug', () => {
    physicsEngine.ToggleDebugDraw();
});

ipcMain.on('toggle-colliders', () => {
    physicsEngine.ToggleColliders();
});

ipcMain.on('toggle-grid', () => {
    physicsEngine.ToggleGrid();
});

ipcMain.on('reset-scene', () => {
    physicsEngine.ResetScene();
    // Recreate initial scene
    physicsEngine.CreatePlane({ x: 0, y: 1, z: 0 }, 0.0);
    physicsEngine.CreateBox({ x: 0, y: 5, z: 0 }, { x: 2, y: 2, z: 2 });
    physicsEngine.CreateSphere({ x: 0, y: 5, z: 0 }, 0.5);
});

ipcMain.on('add-box', () => {
    const position = {
        x: (Math.random() - 0.5) * 10,
        y: 10,
        z: (Math.random() - 0.5) * 10
    };
    physicsEngine.CreateBox(position, { x: 1, y: 1, z: 1 });
});

ipcMain.on('add-sphere', () => {
    const position = {
        x: (Math.random() - 0.5) * 10,
        y: 10,
        z: (Math.random() - 0.5) * 10
    };
    physicsEngine.CreateSphere(position, 1.0);
}); 