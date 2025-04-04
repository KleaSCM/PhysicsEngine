import { app, BrowserWindow, ipcMain } from "electron";
import { createServer } from "http";
import { readFile } from "fs/promises";
import path, { join, extname } from "path";
import { fileURLToPath } from "url";
import { createRequire } from "module";
const __dirname$1 = fileURLToPath(new URL("data:text/javascript;base64,aW1wb3J0IHsgYXBwLCBCcm93c2VyV2luZG93LCBpcGNNYWluIH0gZnJvbSAnZWxlY3Ryb24nOw0KaW1wb3J0IHsgc3RhcnRTZXJ2ZXIgfSBmcm9tICcuL3NlcnZlci5qcyc7DQppbXBvcnQgcGF0aCBmcm9tICdwYXRoJzsNCmltcG9ydCB7IGZpbGVVUkxUb1BhdGggfSBmcm9tICd1cmwnOw0KaW1wb3J0IHsgY3JlYXRlUmVxdWlyZSB9IGZyb20gJ21vZHVsZSc7DQoNCmNvbnN0IF9fZGlybmFtZSA9IHBhdGguZGlybmFtZShmaWxlVVJMVG9QYXRoKGltcG9ydC5tZXRhLnVybCkpOw0KY29uc3QgcmVxdWlyZSA9IGNyZWF0ZVJlcXVpcmUoaW1wb3J0Lm1ldGEudXJsKTsNCg0KLy8gRW5hYmxlIGRlYnVnIGxvZ2dpbmcNCmNvbnN0IERFQlVHID0gdHJ1ZTsNCmZ1bmN0aW9uIGxvZyguLi5hcmdzKSB7DQogICAgaWYgKERFQlVHKSB7DQogICAgICAgIGNvbnNvbGUubG9nKCdbTWFpbiBQcm9jZXNzXTonLCAuLi5hcmdzKTsNCiAgICB9DQp9DQoNCmxldCBtYWluV2luZG93Ow0KbGV0IHBoeXNpY3NFbmdpbmU7DQpsZXQgd2ViU2VydmVyOw0KDQpmdW5jdGlvbiBjcmVhdGVXaW5kb3coKSB7DQogICAgbG9nKCdDcmVhdGluZyBtYWluIHdpbmRvdy4uLicpOw0KICAgIG1haW5XaW5kb3cgPSBuZXcgQnJvd3NlcldpbmRvdyh7DQogICAgICAgIHdpZHRoOiAxMjAwLA0KICAgICAgICBoZWlnaHQ6IDgwMCwNCiAgICAgICAgd2ViUHJlZmVyZW5jZXM6IHsNCiAgICAgICAgICAgIG5vZGVJbnRlZ3JhdGlvbjogZmFsc2UsDQogICAgICAgICAgICBjb250ZXh0SXNvbGF0aW9uOiB0cnVlLA0KICAgICAgICAgICAgcHJlbG9hZDogcGF0aC5qb2luKF9fZGlybmFtZSwgJ3ByZWxvYWQuanMnKQ0KICAgICAgICB9DQogICAgfSk7DQoNCiAgICAvLyBTdGFydCB3ZWIgc2VydmVyDQogICAgd2ViU2VydmVyID0gc3RhcnRTZXJ2ZXIoODA4MCk7DQogICAgbG9nKCdXZWIgc2VydmVyIHN0YXJ0ZWQgb24gcG9ydCA4MDgwJyk7DQoNCiAgICAvLyBJbiBkZXZlbG9wbWVudCwgbG9hZCBmcm9tIFZpdGUgZGV2IHNlcnZlcg0KICAgIGlmIChwcm9jZXNzLmVudi5OT0RFX0VOViA9PT0gJ2RldmVsb3BtZW50Jykgew0KICAgICAgICBtYWluV2luZG93LmxvYWRVUkwoJ2h0dHA6Ly9sb2NhbGhvc3Q6MzAwMCcpOw0KICAgICAgICBtYWluV2luZG93LndlYkNvbnRlbnRzLm9wZW5EZXZUb29scygpOw0KICAgIH0gZWxzZSB7DQogICAgICAgIC8vIEluIHByb2R1Y3Rpb24sIGxvYWQgZnJvbSB0aGUgYnVpbHQgZmlsZXMNCiAgICAgICAgbWFpbldpbmRvdy5sb2FkRmlsZShwYXRoLmpvaW4oX19kaXJuYW1lLCAnZGlzdCcsICdpbmRleC5odG1sJykpOw0KICAgIH0NCg0KICAgIG1haW5XaW5kb3cud2ViQ29udGVudHMub24oJ2RpZC1maW5pc2gtbG9hZCcsICgpID0+IHsNCiAgICAgICAgbG9nKCdXaW5kb3cgbG9hZGVkIHN1Y2Nlc3NmdWxseScpOw0KICAgIH0pOw0KDQogICAgbWFpbldpbmRvdy53ZWJDb250ZW50cy5vbignZGlkLWZhaWwtbG9hZCcsIChldmVudCwgZXJyb3JDb2RlLCBlcnJvckRlc2NyaXB0aW9uKSA9PiB7DQogICAgICAgIGxvZygnRmFpbGVkIHRvIGxvYWQgd2luZG93OicsIGVycm9yQ29kZSwgZXJyb3JEZXNjcmlwdGlvbik7DQogICAgfSk7DQp9DQoNCmFwcC53aGVuUmVhZHkoKS50aGVuKCgpID0+IHsNCiAgICBsb2coJ0FwcCBpcyByZWFkeSwgaW5pdGlhbGl6aW5nLi4uJyk7DQogICAgY3JlYXRlV2luZG93KCk7DQoNCiAgICB0cnkgew0KICAgICAgICBsb2coJ0xvYWRpbmcgcGh5c2ljcyBlbmdpbmUgbW9kdWxlLi4uJyk7DQogICAgICAgIGNvbnN0IFBoeXNpY3MgPSByZXF1aXJlKCcuLi9idWlsZC9SZWxlYXNlL3BoeXNpY3Mubm9kZScpOw0KICAgICAgICBsb2coJ1BoeXNpY3MgZW5naW5lIG1vZHVsZSBsb2FkZWQgc3VjY2Vzc2Z1bGx5Jyk7DQoNCiAgICAgICAgLy8gSW5pdGlhbGl6ZSBwaHlzaWNzIGVuZ2luZQ0KICAgICAgICBsb2coJ0luaXRpYWxpemluZyBwaHlzaWNzIGVuZ2luZS4uLicpOw0KICAgICAgICBwaHlzaWNzRW5naW5lID0gbmV3IFBoeXNpY3MuRW5naW5lKCk7DQogICAgICAgIHBoeXNpY3NFbmdpbmUuaW5pdGlhbGl6ZSgpOw0KICAgICAgICBsb2coJ1BoeXNpY3MgZW5naW5lIGluaXRpYWxpemVkIHN1Y2Nlc3NmdWxseScpOw0KDQogICAgICAgIC8vIENyZWF0ZSBpbml0aWFsIHNjZW5lDQogICAgICAgIGxvZygnQ3JlYXRpbmcgaW5pdGlhbCBzY2VuZS4uLicpOw0KICAgICAgICBwaHlzaWNzRW5naW5lLmNyZWF0ZVBsYW5lKHsgeDogMCwgeTogMSwgejogMCB9LCAwLjAsIDAuMCk7IC8vIEdyb3VuZCBwbGFuZSAobWFzcyA9IDAgZm9yIHN0YXRpYykNCiAgICAgICAgDQogICAgICAgIC8vIENyZWF0ZSB0aGUgcm90YXRpbmcgY3ViZSAoc3RhdGljKQ0KICAgICAgICBwaHlzaWNzRW5naW5lLmNyZWF0ZUJveCh7IHg6IDAsIHk6IDUsIHo6IDAgfSwgeyB4OiAyLCB5OiAyLCB6OiAyIH0sIDAuMCk7DQogICAgICAgIA0KICAgICAgICAvLyBDcmVhdGUgdGhlIGJhbGwgaW5zaWRlIHRoZSBjdWJlIChkeW5hbWljKQ0KICAgICAgICBwaHlzaWNzRW5naW5lLmNyZWF0ZVNwaGVyZSh7IHg6IDAsIHk6IDUsIHo6IDAgfSwgMC41LCAxLjApOw0KICAgICAgICANCiAgICAgICAgLy8gU2V0IHVwIHRoZSByb3RhdGluZyBjdWJlIGNvbnN0cmFpbnQNCiAgICAgICAgcGh5c2ljc0VuZ2luZS5jcmVhdGVIaW5nZUNvbnN0cmFpbnQoDQogICAgICAgICAgICB7IHg6IDAsIHk6IDUsIHo6IDAgfSwgIC8vIFBpdm90IHBvaW50DQogICAgICAgICAgICB7IHg6IDAsIHk6IDEsIHo6IDAgfSwgIC8vIEF4aXMNCiAgICAgICAgICAgIDAuMCwgICAgICAgICAgICAgICAgICAgIC8vIEFuZ3VsYXIgdmVsb2NpdHkgKHdpbGwgYmUgc2V0IGluIHVwZGF0ZSBsb29wKQ0KICAgICAgICAgICAgdHJ1ZSAgICAgICAgICAgICAgICAgICAgLy8gSXMgcm90YXRpbmcNCiAgICAgICAgKTsNCiAgICAgICAgbG9nKCdJbml0aWFsIHNjZW5lIGNyZWF0ZWQgc3VjY2Vzc2Z1bGx5Jyk7DQoNCiAgICAgICAgLy8gU3RhcnQgcGh5c2ljcyB1cGRhdGUgbG9vcA0KICAgICAgICBsZXQgbGFzdFRpbWUgPSBEYXRlLm5vdygpOw0KICAgICAgICBsZXQgcm90YXRpb25BbmdsZSA9IDAuMDsNCiAgICAgICAgY29uc3Qgcm90YXRpb25TcGVlZCA9IDEuMDsNCg0KICAgICAgICBmdW5jdGlvbiB1cGRhdGVQaHlzaWNzKCkgew0KICAgICAgICAgICAgdHJ5IHsNCiAgICAgICAgICAgICAgICBjb25zdCBjdXJyZW50VGltZSA9IERhdGUubm93KCk7DQogICAgICAgICAgICAgICAgY29uc3QgZGVsdGFUaW1lID0gKGN1cnJlbnRUaW1lIC0gbGFzdFRpbWUpIC8gMTAwMC4wOw0KICAgICAgICAgICAgICAgIGxhc3RUaW1lID0gY3VycmVudFRpbWU7DQoNCiAgICAgICAgICAgICAgICAvLyBVcGRhdGUgcGh5c2ljcw0KICAgICAgICAgICAgICAgIHBoeXNpY3NFbmdpbmUudXBkYXRlKGRlbHRhVGltZSk7DQoNCiAgICAgICAgICAgICAgICAvLyBVcGRhdGUgcm90YXRpb24gYW5nbGUNCiAgICAgICAgICAgICAgICByb3RhdGlvbkFuZ2xlICs9IHJvdGF0aW9uU3BlZWQgKiBkZWx0YVRpbWU7DQogICAgICAgICAgICAgICAgcGh5c2ljc0VuZ2luZS5zZXRIaW5nZUNvbnN0cmFpbnRSb3RhdGlvbigwLCByb3RhdGlvbkFuZ2xlKTsNCg0KICAgICAgICAgICAgICAgIC8vIEdldCBwaHlzaWNzIGRhdGENCiAgICAgICAgICAgICAgICBjb25zdCBib2RpZXMgPSBbXTsNCiAgICAgICAgICAgICAgICBjb25zdCBib2R5Q291bnQgPSBwaHlzaWNzRW5naW5lLmdldEJvZHlDb3VudCgpOw0KICAgICAgICAgICAgICAgIGZvciAobGV0IGkgPSAwOyBpIDwgYm9keUNvdW50OyBpKyspIHsNCiAgICAgICAgICAgICAgICAgICAgY29uc3QgYm9keSA9IHBoeXNpY3NFbmdpbmUuZ2V0V29ybGQoKS5nZXRCb2R5KGkpOw0KICAgICAgICAgICAgICAgICAgICBjb25zdCBib2R5RGF0YSA9IHsNCiAgICAgICAgICAgICAgICAgICAgICAgIGlkOiBpLA0KICAgICAgICAgICAgICAgICAgICAgICAgdHlwZTogYm9keS5zaGFwZSA9PT0gJ2JveCcgPyAnYm94JyA6ICdzcGhlcmUnLA0KICAgICAgICAgICAgICAgICAgICAgICAgcG9zaXRpb246IHsNCiAgICAgICAgICAgICAgICAgICAgICAgICAgICB4OiBib2R5LnBvc2l0aW9uLngsDQogICAgICAgICAgICAgICAgICAgICAgICAgICAgeTogYm9keS5wb3NpdGlvbi55LA0KICAgICAgICAgICAgICAgICAgICAgICAgICAgIHo6IGJvZHkucG9zaXRpb24ueg0KICAgICAgICAgICAgICAgICAgICAgICAgfSwNCiAgICAgICAgICAgICAgICAgICAgICAgIHJvdGF0aW9uOiB7DQogICAgICAgICAgICAgICAgICAgICAgICAgICAgeDogYm9keS5yb3RhdGlvbi54LA0KICAgICAgICAgICAgICAgICAgICAgICAgICAgIHk6IGJvZHkucm90YXRpb24ueSwNCiAgICAgICAgICAgICAgICAgICAgICAgICAgICB6OiBib2R5LnJvdGF0aW9uLnosDQogICAgICAgICAgICAgICAgICAgICAgICAgICAgdzogYm9keS5yb3RhdGlvbi53DQogICAgICAgICAgICAgICAgICAgICAgICB9DQogICAgICAgICAgICAgICAgICAgIH07DQogICAgICAgICAgICAgICAgICAgIA0KICAgICAgICAgICAgICAgICAgICAvLyBBZGQgc2l6ZSBvciByYWRpdXMgYmFzZWQgb24gYm9keSB0eXBlDQogICAgICAgICAgICAgICAgICAgIGlmIChib2R5LnNoYXBlID09PSAnYm94Jykgew0KICAgICAgICAgICAgICAgICAgICAgICAgYm9keURhdGEuc2l6ZSA9IHsNCiAgICAgICAgICAgICAgICAgICAgICAgICAgICB4OiBib2R5LmhhbGZFeHRlbnRzLnggKiAyLA0KICAgICAgICAgICAgICAgICAgICAgICAgICAgIHk6IGJvZHkuaGFsZkV4dGVudHMueSAqIDIsDQogICAgICAgICAgICAgICAgICAgICAgICAgICAgejogYm9keS5oYWxmRXh0ZW50cy56ICogMg0KICAgICAgICAgICAgICAgICAgICAgICAgfTsNCiAgICAgICAgICAgICAgICAgICAgfSBlbHNlIHsNCiAgICAgICAgICAgICAgICAgICAgICAgIGJvZHlEYXRhLnJhZGl1cyA9IGJvZHkucmFkaXVzOw0KICAgICAgICAgICAgICAgICAgICB9DQogICAgICAgICAgICAgICAgICAgIA0KICAgICAgICAgICAgICAgICAgICBib2RpZXMucHVzaChib2R5RGF0YSk7DQogICAgICAgICAgICAgICAgfQ0KDQogICAgICAgICAgICAgICAgY29uc3QgZGVidWdEYXRhID0gcGh5c2ljc0VuZ2luZS5nZXREZWJ1Z0RyYXdEYXRhKCk7DQoNCiAgICAgICAgICAgICAgICBtYWluV2luZG93LndlYkNvbnRlbnRzLnNlbmQoJ3BoeXNpY3MtdXBkYXRlJywgew0KICAgICAgICAgICAgICAgICAgICBmcHM6IHBoeXNpY3NFbmdpbmUuZ2V0QXZlcmFnZUZQUygpLA0KICAgICAgICAgICAgICAgICAgICBib2RpZXM6IGJvZGllcywNCiAgICAgICAgICAgICAgICAgICAgdGltZVN0ZXA6IHBoeXNpY3NFbmdpbmUuZ2V0U2V0dGluZ3MoKS5maXhlZFRpbWVTdGVwLA0KICAgICAgICAgICAgICAgICAgICBkZWJ1Z0RhdGE6IHsNCiAgICAgICAgICAgICAgICAgICAgICAgIGxpbmVzOiBkZWJ1Z0RhdGEubGluZXMubWFwKGxpbmUgPT4gKHsNCiAgICAgICAgICAgICAgICAgICAgICAgICAgICBzdGFydDogeyB4OiBsaW5lLnN0YXJ0LngsIHk6IGxpbmUuc3RhcnQueSwgejogbGluZS5zdGFydC56IH0sDQogICAgICAgICAgICAgICAgICAgICAgICAgICAgZW5kOiB7IHg6IGxpbmUuZW5kLngsIHk6IGxpbmUuZW5kLnksIHo6IGxpbmUuZW5kLnogfSwNCiAgICAgICAgICAgICAgICAgICAgICAgICAgICBjb2xvcjogeyB4OiBsaW5lLmNvbG9yLngsIHk6IGxpbmUuY29sb3IueSwgejogbGluZS5jb2xvci56IH0NCiAgICAgICAgICAgICAgICAgICAgICAgIH0pKSwNCiAgICAgICAgICAgICAgICAgICAgICAgIHBvaW50czogZGVidWdEYXRhLnBvaW50cy5tYXAocG9pbnQgPT4gKHsNCiAgICAgICAgICAgICAgICAgICAgICAgICAgICBwb3NpdGlvbjogeyB4OiBwb2ludC5wb3NpdGlvbi54LCB5OiBwb2ludC5wb3NpdGlvbi55LCB6OiBwb2ludC5wb3NpdGlvbi56IH0sDQogICAgICAgICAgICAgICAgICAgICAgICAgICAgY29sb3I6IHsgeDogcG9pbnQuY29sb3IueCwgeTogcG9pbnQuY29sb3IueSwgejogcG9pbnQuY29sb3IueiB9LA0KICAgICAgICAgICAgICAgICAgICAgICAgICAgIHNpemU6IHBvaW50LnNpemUNCiAgICAgICAgICAgICAgICAgICAgICAgIH0pKQ0KICAgICAgICAgICAgICAgICAgICB9DQogICAgICAgICAgICAgICAgfSk7DQogICAgICAgICAgICB9IGNhdGNoIChlcnJvcikgew0KICAgICAgICAgICAgICAgIGxvZygnRXJyb3IgaW4gcGh5c2ljcyB1cGRhdGU6JywgZXJyb3IpOw0KICAgICAgICAgICAgfQ0KICAgICAgICB9DQogICAgICAgIC8vIFVzZSBzZXRJbnRlcnZhbCBpbnN0ZWFkIG9mIHJlcXVlc3RBbmltYXRpb25GcmFtZQ0KICAgICAgICBzZXRJbnRlcnZhbCh1cGRhdGVQaHlzaWNzLCAxNik7IC8vIH42MCBGUFMNCiAgICB9IGNhdGNoIChlcnJvcikgew0KICAgICAgICBsb2coJ0ZhaWxlZCB0byBpbml0aWFsaXplIHBoeXNpY3MgZW5naW5lOicsIGVycm9yKTsNCiAgICAgICAgbWFpbldpbmRvdy53ZWJDb250ZW50cy5zZW5kKCdwaHlzaWNzLWVycm9yJywgZXJyb3IubWVzc2FnZSk7DQogICAgfQ0KfSk7DQoNCmFwcC5vbignYWN0aXZhdGUnLCAoKSA9PiB7DQogICAgbG9nKCdBcHAgYWN0aXZhdGVkJyk7DQogICAgaWYgKEJyb3dzZXJXaW5kb3cuZ2V0QWxsV2luZG93cygpLmxlbmd0aCA9PT0gMCkgew0KICAgICAgICBjcmVhdGVXaW5kb3coKTsNCiAgICB9DQp9KTsNCg0KYXBwLm9uKCd3aW5kb3ctYWxsLWNsb3NlZCcsICgpID0+IHsNCiAgICBsb2coJ0FsbCB3aW5kb3dzIGNsb3NlZCcpOw0KICAgIGlmIChwcm9jZXNzLnBsYXRmb3JtICE9PSAnZGFyd2luJykgew0KICAgICAgICBhcHAucXVpdCgpOw0KICAgIH0NCn0pOw0KDQphcHAub24oJ2JlZm9yZS1xdWl0JywgKCkgPT4gew0KICAgIGxvZygnQXBwIHF1aXR0aW5nLCBjbGVhbmluZyB1cC4uLicpOw0KICAgIGlmICh3ZWJTZXJ2ZXIpIHsNCiAgICAgICAgd2ViU2VydmVyLmNsb3NlKCk7DQogICAgICAgIGxvZygnV2ViIHNlcnZlciBzdG9wcGVkJyk7DQogICAgfQ0KfSk7DQoNCi8vIEhhbmRsZSBJUEMgbWVzc2FnZXMgZnJvbSByZW5kZXJlcg0KaXBjTWFpbi5vbigndG9nZ2xlLWRlYnVnJywgKCkgPT4gew0KICAgIGxvZygnVG9nZ2xlIGRlYnVnIHJlcXVlc3RlZCcpOw0KICAgIHRyeSB7DQogICAgICAgIHBoeXNpY3NFbmdpbmUudG9nZ2xlRGVidWdEcmF3KCk7DQogICAgfSBjYXRjaCAoZXJyb3IpIHsNCiAgICAgICAgbG9nKCdFcnJvciB0b2dnbGluZyBkZWJ1ZzonLCBlcnJvcik7DQogICAgfQ0KfSk7DQoNCmlwY01haW4ub24oJ3RvZ2dsZS1jb2xsaWRlcnMnLCAoKSA9PiB7DQogICAgbG9nKCdUb2dnbGUgY29sbGlkZXJzIHJlcXVlc3RlZCcpOw0KICAgIHRyeSB7DQogICAgICAgIHBoeXNpY3NFbmdpbmUudG9nZ2xlQ29sbGlkZXJzKCk7DQogICAgfSBjYXRjaCAoZXJyb3IpIHsNCiAgICAgICAgbG9nKCdFcnJvciB0b2dnbGluZyBjb2xsaWRlcnM6JywgZXJyb3IpOw0KICAgIH0NCn0pOw0KDQppcGNNYWluLm9uKCd0b2dnbGUtZ3JpZCcsICgpID0+IHsNCiAgICBsb2coJ1RvZ2dsZSBncmlkIHJlcXVlc3RlZCcpOw0KICAgIHRyeSB7DQogICAgICAgIHBoeXNpY3NFbmdpbmUudG9nZ2xlR3JpZCgpOw0KICAgIH0gY2F0Y2ggKGVycm9yKSB7DQogICAgICAgIGxvZygnRXJyb3IgdG9nZ2xpbmcgZ3JpZDonLCBlcnJvcik7DQogICAgfQ0KfSk7DQoNCmlwY01haW4ub24oJ3Jlc2V0LXNjZW5lJywgKCkgPT4gew0KICAgIGxvZygnUmVzZXQgc2NlbmUgcmVxdWVzdGVkJyk7DQogICAgdHJ5IHsNCiAgICAgICAgcGh5c2ljc0VuZ2luZS5yZXNldFNjZW5lKCk7DQogICAgICAgIC8vIFJlY3JlYXRlIGluaXRpYWwgc2NlbmUNCiAgICAgICAgcGh5c2ljc0VuZ2luZS5jcmVhdGVQbGFuZSh7IHg6IDAsIHk6IDEsIHo6IDAgfSwgMC4wLCAwLjApOw0KICAgICAgICBwaHlzaWNzRW5naW5lLmNyZWF0ZUJveCh7IHg6IDAsIHk6IDUsIHo6IDAgfSwgeyB4OiAyLCB5OiAyLCB6OiAyIH0sIDAuMCk7DQogICAgICAgIHBoeXNpY3NFbmdpbmUuY3JlYXRlU3BoZXJlKHsgeDogMCwgeTogNSwgejogMCB9LCAwLjUsIDEuMCk7DQogICAgfSBjYXRjaCAoZXJyb3IpIHsNCiAgICAgICAgbG9nKCdFcnJvciByZXNldHRpbmcgc2NlbmU6JywgZXJyb3IpOw0KICAgIH0NCn0pOw0KDQppcGNNYWluLm9uKCdhZGQtYm94JywgKCkgPT4gew0KICAgIGxvZygnQWRkIGJveCByZXF1ZXN0ZWQnKTsNCiAgICB0cnkgew0KICAgICAgICBjb25zdCBwb3NpdGlvbiA9IHsNCiAgICAgICAgICAgIHg6IChNYXRoLnJhbmRvbSgpIC0gMC41KSAqIDEwLA0KICAgICAgICAgICAgeTogMTAsDQogICAgICAgICAgICB6OiAoTWF0aC5yYW5kb20oKSAtIDAuNSkgKiAxMA0KICAgICAgICB9Ow0KICAgICAgICBwaHlzaWNzRW5naW5lLmNyZWF0ZUJveChwb3NpdGlvbiwgeyB4OiAxLCB5OiAxLCB6OiAxIH0sIDEuMCk7DQogICAgfSBjYXRjaCAoZXJyb3IpIHsNCiAgICAgICAgbG9nKCdFcnJvciBhZGRpbmcgYm94OicsIGVycm9yKTsNCiAgICB9DQp9KTsNCg0KaXBjTWFpbi5vbignYWRkLXNwaGVyZScsICgpID0+IHsNCiAgICBsb2coJ0FkZCBzcGhlcmUgcmVxdWVzdGVkJyk7DQogICAgdHJ5IHsNCiAgICAgICAgY29uc3QgcG9zaXRpb24gPSB7DQogICAgICAgICAgICB4OiAoTWF0aC5yYW5kb20oKSAtIDAuNSkgKiAxMCwNCiAgICAgICAgICAgIHk6IDEwLA0KICAgICAgICAgICAgejogKE1hdGgucmFuZG9tKCkgLSAwLjUpICogMTANCiAgICAgICAgfTsNCiAgICAgICAgcGh5c2ljc0VuZ2luZS5jcmVhdGVTcGhlcmUocG9zaXRpb24sIDEuMCwgMS4wKTsNCiAgICB9IGNhdGNoIChlcnJvcikgew0KICAgICAgICBsb2coJ0Vycm9yIGFkZGluZyBzcGhlcmU6JywgZXJyb3IpOw0KICAgIH0NCn0pOyA=", import.meta.url));
const MIME_TYPES = {
  ".html": "text/html",
  ".js": "text/javascript",
  ".css": "text/css",
  ".json": "application/json",
  ".png": "image/png",
  ".jpg": "image/jpeg",
  ".gif": "image/gif",
  ".svg": "image/svg+xml",
  ".wav": "audio/wav",
  ".mp4": "video/mp4",
  ".woff": "application/font-woff",
  ".ttf": "application/font-ttf",
  ".eot": "application/vnd.ms-fontobject",
  ".otf": "application/font-otf",
  ".wasm": "application/wasm",
  ".ico": "image/x-icon"
};
function startServer(port = 8080) {
  const server = createServer(async (req, res) => {
    console.log(`Request: ${req.method} ${req.url}`);
    try {
      let filePath = req.url === "/" ? "index.html" : req.url;
      filePath = join(__dirname$1, filePath);
      const ext = extname(filePath);
      const contentType = MIME_TYPES[ext] || "application/octet-stream";
      const data = await readFile(filePath);
      res.writeHead(200, { "Content-Type": contentType });
      res.end(data);
    } catch (error) {
      if (error.code === "ENOENT") {
        res.writeHead(404);
        res.end("404 Not Found");
      } else {
        res.writeHead(500);
        res.end("500 Internal Server Error");
      }
    }
  });
  server.listen(port, () => {
    console.log(`Server running at http://localhost:${port}/`);
  });
  return server;
}
const __dirname = path.dirname(fileURLToPath(import.meta.url));
const require2 = createRequire(import.meta.url);
function log(...args) {
  {
    console.log("[Main Process]:", ...args);
  }
}
let mainWindow;
let physicsEngine;
let webServer;
function createWindow() {
  log("Creating main window...");
  mainWindow = new BrowserWindow({
    width: 1200,
    height: 800,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      preload: path.join(__dirname, "preload.js")
    }
  });
  webServer = startServer(8080);
  log("Web server started on port 8080");
  if (process.env.NODE_ENV === "development") {
    mainWindow.loadURL("http://localhost:3000");
    mainWindow.webContents.openDevTools();
  } else {
    mainWindow.loadFile(path.join(__dirname, "dist", "index.html"));
  }
  mainWindow.webContents.on("did-finish-load", () => {
    log("Window loaded successfully");
  });
  mainWindow.webContents.on("did-fail-load", (event, errorCode, errorDescription) => {
    log("Failed to load window:", errorCode, errorDescription);
  });
}
app.whenReady().then(() => {
  log("App is ready, initializing...");
  createWindow();
  try {
    let updatePhysics = function() {
      try {
        const currentTime = Date.now();
        const deltaTime = (currentTime - lastTime) / 1e3;
        lastTime = currentTime;
        physicsEngine.update(deltaTime);
        rotationAngle += rotationSpeed * deltaTime;
        physicsEngine.setHingeConstraintRotation(0, rotationAngle);
        const bodies = [];
        const bodyCount = physicsEngine.getBodyCount();
        for (let i = 0; i < bodyCount; i++) {
          const body = physicsEngine.getWorld().getBody(i);
          const bodyData = {
            id: i,
            type: body.shape === "box" ? "box" : "sphere",
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
          if (body.shape === "box") {
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
        mainWindow.webContents.send("physics-update", {
          fps: physicsEngine.getAverageFPS(),
          bodies,
          timeStep: physicsEngine.getSettings().fixedTimeStep,
          debugData: {
            lines: debugData.lines.map((line) => ({
              start: { x: line.start.x, y: line.start.y, z: line.start.z },
              end: { x: line.end.x, y: line.end.y, z: line.end.z },
              color: { x: line.color.x, y: line.color.y, z: line.color.z }
            })),
            points: debugData.points.map((point) => ({
              position: { x: point.position.x, y: point.position.y, z: point.position.z },
              color: { x: point.color.x, y: point.color.y, z: point.color.z },
              size: point.size
            }))
          }
        });
      } catch (error) {
        log("Error in physics update:", error);
      }
    };
    log("Loading physics engine module...");
    const Physics = require2("../build/Release/physics.node");
    log("Physics engine module loaded successfully");
    log("Initializing physics engine...");
    physicsEngine = new Physics.Engine();
    physicsEngine.initialize();
    log("Physics engine initialized successfully");
    log("Creating initial scene...");
    physicsEngine.createPlane({ x: 0, y: 1, z: 0 }, 0, 0);
    physicsEngine.createBox({ x: 0, y: 5, z: 0 }, { x: 2, y: 2, z: 2 }, 0);
    physicsEngine.createSphere({ x: 0, y: 5, z: 0 }, 0.5, 1);
    physicsEngine.createHingeConstraint(
      { x: 0, y: 5, z: 0 },
      // Pivot point
      { x: 0, y: 1, z: 0 },
      // Axis
      0,
      // Angular velocity (will be set in update loop)
      true
      // Is rotating
    );
    log("Initial scene created successfully");
    let lastTime = Date.now();
    let rotationAngle = 0;
    const rotationSpeed = 1;
    setInterval(updatePhysics, 16);
  } catch (error) {
    log("Failed to initialize physics engine:", error);
    mainWindow.webContents.send("physics-error", error.message);
  }
});
app.on("activate", () => {
  log("App activated");
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});
app.on("window-all-closed", () => {
  log("All windows closed");
  if (process.platform !== "darwin") {
    app.quit();
  }
});
app.on("before-quit", () => {
  log("App quitting, cleaning up...");
  if (webServer) {
    webServer.close();
    log("Web server stopped");
  }
});
ipcMain.on("toggle-debug", () => {
  log("Toggle debug requested");
  try {
    physicsEngine.toggleDebugDraw();
  } catch (error) {
    log("Error toggling debug:", error);
  }
});
ipcMain.on("toggle-colliders", () => {
  log("Toggle colliders requested");
  try {
    physicsEngine.toggleColliders();
  } catch (error) {
    log("Error toggling colliders:", error);
  }
});
ipcMain.on("toggle-grid", () => {
  log("Toggle grid requested");
  try {
    physicsEngine.toggleGrid();
  } catch (error) {
    log("Error toggling grid:", error);
  }
});
ipcMain.on("reset-scene", () => {
  log("Reset scene requested");
  try {
    physicsEngine.resetScene();
    physicsEngine.createPlane({ x: 0, y: 1, z: 0 }, 0, 0);
    physicsEngine.createBox({ x: 0, y: 5, z: 0 }, { x: 2, y: 2, z: 2 }, 0);
    physicsEngine.createSphere({ x: 0, y: 5, z: 0 }, 0.5, 1);
  } catch (error) {
    log("Error resetting scene:", error);
  }
});
ipcMain.on("add-box", () => {
  log("Add box requested");
  try {
    const position = {
      x: (Math.random() - 0.5) * 10,
      y: 10,
      z: (Math.random() - 0.5) * 10
    };
    physicsEngine.createBox(position, { x: 1, y: 1, z: 1 }, 1);
  } catch (error) {
    log("Error adding box:", error);
  }
});
ipcMain.on("add-sphere", () => {
  log("Add sphere requested");
  try {
    const position = {
      x: (Math.random() - 0.5) * 10,
      y: 10,
      z: (Math.random() - 0.5) * 10
    };
    physicsEngine.createSphere(position, 1, 1);
  } catch (error) {
    log("Error adding sphere:", error);
  }
});
