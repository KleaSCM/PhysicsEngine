const { app: n, BrowserWindow: i, ipcMain: s } = require("electron"), a = require("path");
function o() {
  const e = new i({
    width: 1200,
    height: 800,
    webPreferences: {
      preload: a.join(__dirname, "preload.js"),
      contextIsolation: !0,
      nodeIntegration: !1
    }
  });
  e.loadFile("web/index.html"), process.env.NODE_ENV === "development" && e.webContents.openDevTools();
}
n.whenReady().then(() => {
  o(), n.on("activate", function() {
    i.getAllWindows().length === 0 && o();
  });
});
n.on("window-all-closed", function() {
  process.platform !== "darwin" && n.quit();
});
s.on("message", (e, t) => {
  console.log("Received message:", t), e.reply("message", "Message received!");
});
