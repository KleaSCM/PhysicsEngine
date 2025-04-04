const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
    on: (channel, callback) => {
        ipcRenderer.on(channel, (event, ...args) => callback(event, ...args));
    },
    send: (channel, data) => {
        ipcRenderer.send(channel, data);
    },
    toggleDebug: () => ipcRenderer.send('toggle-debug'),
    toggleColliders: () => ipcRenderer.send('toggle-colliders'),
    toggleGrid: () => ipcRenderer.send('toggle-grid'),
    resetScene: () => ipcRenderer.send('reset-scene'),
    addBox: () => ipcRenderer.send('add-box'),
    addSphere: () => ipcRenderer.send('add-sphere')
}); 