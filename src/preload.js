const { contextBridge, ipcRenderer } = require('electron');

// Expose protected methods that allow the renderer process to use
// the ipcRenderer without exposing the entire object
contextBridge.exposeInMainWorld(
    'electronAPI', {
        // Physics control methods
        toggleDebug: () => ipcRenderer.send('toggle-debug'),
        toggleColliders: () => ipcRenderer.send('toggle-colliders'),
        toggleGrid: () => ipcRenderer.send('toggle-grid'),
        resetScene: () => ipcRenderer.send('reset-scene'),
        addBox: () => ipcRenderer.send('add-box'),
        addSphere: () => ipcRenderer.send('add-sphere'),
        
        // Message handling
        sendMessage: (message) => ipcRenderer.send('message', message),
        onMessage: (callback) => ipcRenderer.on('message', callback),
        removeMessageListener: (callback) => ipcRenderer.removeListener('message', callback)
    }
); 