const { app, BrowserWindow, ipcMain } = require('electron');
const path = require('path');

// Enable debug logging
const DEBUG = true;
function log(...args) {
    if (DEBUG) {
        console.log('[Main Process]:', ...args);
    }
}

let mainWindow;
let physicsEngine;

function createWindow() {
    log('Creating main window...');
    mainWindow = new BrowserWindow({
        width: 1200,
        height: 800,
        webPreferences: {
            nodeIntegration: true,
            contextIsolation: false
        }
    });

    mainWindow.loadFile('index.html');
    mainWindow.webContents.openDevTools(); // Always open DevTools for debugging

    mainWindow.webContents.on('did-finish-load', () => {
        log('Window loaded successfully');
    });

    mainWindow.webContents.on('did-fail-load', (event, errorCode, errorDescription) => {
        log('Failed to load window:', errorCode, errorDescription);
    });
}

app.whenReady().then(() => {
    log('App is ready, initializing...');
    createWindow();

    try {
        log('Loading physics engine module...');
        const Physics = require('../build/Release/physics.node');
        log('Physics engine module loaded successfully');

        // Initialize physics engine
        log('Initializing physics engine...');
        physicsEngine = new Physics.Engine();
        physicsEngine.initialize();
        log('Physics engine initialized successfully');

        // Create initial scene
        log('Creating initial scene...');
        physicsEngine.createPlane({ x: 0, y: 1, z: 0 }, 0.0, 0.0); // Ground plane (mass = 0 for static)
        
        // Create the rotating cube (static)
        physicsEngine.createBox({ x: 0, y: 5, z: 0 }, { x: 2, y: 2, z: 2 }, 0.0);
        
        // Create the ball inside the cube (dynamic)
        physicsEngine.createSphere({ x: 0, y: 5, z: 0 }, 0.5, 1.0);
        
        // Set up the rotating cube constraint
        physicsEngine.createHingeConstraint(
            { x: 0, y: 5, z: 0 },  // Pivot point
            { x: 0, y: 1, z: 0 },  // Axis
            0.0,                    // Angular velocity (will be set in update loop)
            true                    // Is rotating
        );
        log('Initial scene created successfully');

        // Start physics update loop
        let lastTime = Date.now();
        let rotationAngle = 0.0;
        const rotationSpeed = 1.0; // radians per second

        function updatePhysics() {
            try {
                const currentTime = Date.now();
                const deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
                lastTime = currentTime;

                // Update rotation angle
                rotationAngle += rotationSpeed * deltaTime;
                if (rotationAngle > 2 * Math.PI) {
                    rotationAngle -= 2 * Math.PI;
                }

                // Apply rotation to the cube
                physicsEngine.setHingeConstraintRotation(0, rotationAngle);

                physicsEngine.update(deltaTime);

                // Send physics data to renderer
                const debugData = physicsEngine.getDebugDrawData();
                mainWindow.webContents.send('physics-update', {
                    fps: physicsEngine.getAverageFPS(),
                    bodies: physicsEngine.getWorld().getBodyCount(),
                    timeStep: physicsEngine.getSettings().fixedTimeStep,
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
            } catch (error) {
                log('Error in physics update:', error);
            }
        }
        updatePhysics();
    } catch (error) {
        log('Failed to initialize physics engine:', error);
        mainWindow.webContents.send('physics-error', error.message);
    }
});

app.on('window-all-closed', () => {
    log('All windows closed');
    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('activate', () => {
    log('App activated');
    if (BrowserWindow.getAllWindows().length === 0) {
        createWindow();
    }
});

// Handle IPC messages from renderer
ipcMain.on('toggle-debug', () => {
    log('Toggle debug requested');
    try {
        physicsEngine.toggleDebugDraw();
    } catch (error) {
        log('Error toggling debug:', error);
    }
});

ipcMain.on('toggle-colliders', () => {
    log('Toggle colliders requested');
    try {
        physicsEngine.toggleColliders();
    } catch (error) {
        log('Error toggling colliders:', error);
    }
});

ipcMain.on('toggle-grid', () => {
    log('Toggle grid requested');
    try {
        physicsEngine.toggleGrid();
    } catch (error) {
        log('Error toggling grid:', error);
    }
});

ipcMain.on('reset-scene', () => {
    log('Reset scene requested');
    try {
        physicsEngine.resetScene();
        // Recreate initial scene
        physicsEngine.createPlane({ x: 0, y: 1, z: 0 }, 0.0, 0.0);
        physicsEngine.createBox({ x: 0, y: 5, z: 0 }, { x: 2, y: 2, z: 2 }, 0.0);
        physicsEngine.createSphere({ x: 0, y: 5, z: 0 }, 0.5, 1.0);
    } catch (error) {
        log('Error resetting scene:', error);
    }
});

ipcMain.on('add-box', () => {
    log('Add box requested');
    try {
        const position = {
            x: (Math.random() - 0.5) * 10,
            y: 10,
            z: (Math.random() - 0.5) * 10
        };
        physicsEngine.createBox(position, { x: 1, y: 1, z: 1 }, 1.0);
    } catch (error) {
        log('Error adding box:', error);
    }
});

ipcMain.on('add-sphere', () => {
    log('Add sphere requested');
    try {
        const position = {
            x: (Math.random() - 0.5) * 10,
            y: 10,
            z: (Math.random() - 0.5) * 10
        };
        physicsEngine.createSphere(position, 1.0, 1.0);
    } catch (error) {
        log('Error adding sphere:', error);
    }
}); 