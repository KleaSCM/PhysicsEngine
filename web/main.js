import { app, BrowserWindow, ipcMain } from 'electron';
import { startServer } from './server.js';
import path from 'path';
import { fileURLToPath } from 'url';
import { createRequire } from 'module';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const require = createRequire(import.meta.url);

// Enable debug logging
const DEBUG = true;
function log(...args) {
    if (DEBUG) {
        console.log('[Main Process]:', ...args);
    }
}

let mainWindow;
let physicsEngine;
let webServer;

function createWindow() {
    log('Creating main window...');
    mainWindow = new BrowserWindow({
        width: 1200,
        height: 800,
        webPreferences: {
            nodeIntegration: false,
            contextIsolation: true,
            preload: path.join(__dirname, 'preload.js')
        }
    });

    // Start web server
    webServer = startServer(8080);
    log('Web server started on port 8080');

    // In development, load from Vite dev server
    if (process.env.NODE_ENV === 'development') {
        mainWindow.loadURL('http://localhost:3000');
        mainWindow.webContents.openDevTools();
    } else {
        // In production, load from the built files
        mainWindow.loadFile(path.join(__dirname, 'dist', 'index.html'));
    }

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
        const rotationSpeed = 1.0;

        function updatePhysics() {
            try {
                const currentTime = Date.now();
                const deltaTime = (currentTime - lastTime) / 1000.0;
                lastTime = currentTime;

                // Update physics
                physicsEngine.update(deltaTime);

                // Update rotation angle
                rotationAngle += rotationSpeed * deltaTime;
                physicsEngine.setHingeConstraintRotation(0, rotationAngle);

                // Get physics data
                const bodies = [];
                const bodyCount = physicsEngine.getBodyCount();
                for (let i = 0; i < bodyCount; i++) {
                    const body = physicsEngine.getWorld().getBody(i);
                    const bodyData = {
                        id: i,
                        type: body.shape === 'box' ? 'box' : 'sphere',
                        position: {
                            x: body.position.x,
                            y: body.position.y,
                            z: body.position.z
                        },
                        rotation: {
                            x: body.rotation.x,
                            y: body.rotation.y,
                            z: body.rotation.z,
                            w: body.rotation.w
                        }
                    };
                    
                    // Add size or radius based on body type
                    if (body.shape === 'box') {
                        bodyData.size = {
                            x: body.halfExtents.x * 2,
                            y: body.halfExtents.y * 2,
                            z: body.halfExtents.z * 2
                        };
                    } else {
                        bodyData.radius = body.radius;
                    }
                    
                    bodies.push(bodyData);
                }

                const debugData = physicsEngine.getDebugDrawData();

                mainWindow.webContents.send('physics-update', {
                    fps: physicsEngine.getAverageFPS(),
                    bodies: bodies,
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
            } catch (error) {
                log('Error in physics update:', error);
            }
        }
        // Use setInterval instead of requestAnimationFrame
        setInterval(updatePhysics, 16); // ~60 FPS
    } catch (error) {
        log('Failed to initialize physics engine:', error);
        mainWindow.webContents.send('physics-error', error.message);
    }
});

app.on('activate', () => {
    log('App activated');
    if (BrowserWindow.getAllWindows().length === 0) {
        createWindow();
    }
});

app.on('window-all-closed', () => {
    log('All windows closed');
    if (process.platform !== 'darwin') {
        app.quit();
    }
});

app.on('before-quit', () => {
    log('App quitting, cleaning up...');
    if (webServer) {
        webServer.close();
        log('Web server stopped');
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