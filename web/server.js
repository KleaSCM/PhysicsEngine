import { createServer } from 'http';
import { readFile } from 'fs/promises';
import { join, extname } from 'path';
import { fileURLToPath } from 'url';

const __dirname = fileURLToPath(new URL('.', import.meta.url));

const MIME_TYPES = {
    '.html': 'text/html',
    '.js': 'text/javascript',
    '.css': 'text/css',
    '.json': 'application/json',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.gif': 'image/gif',
    '.svg': 'image/svg+xml',
    '.wav': 'audio/wav',
    '.mp4': 'video/mp4',
    '.woff': 'application/font-woff',
    '.ttf': 'application/font-ttf',
    '.eot': 'application/vnd.ms-fontobject',
    '.otf': 'application/font-otf',
    '.wasm': 'application/wasm',
    '.ico': 'image/x-icon'
};

export function startServer(port = 8080) {
    const server = createServer(async (req, res) => {
        console.log(`Request: ${req.method} ${req.url}`);

        try {
            // Handle root path
            let filePath = req.url === '/' ? 'index.html' : req.url;
            filePath = join(__dirname, filePath);

            // Determine content type
            const ext = extname(filePath);
            const contentType = MIME_TYPES[ext] || 'application/octet-stream';

            // Read and serve the file
            const data = await readFile(filePath);
            res.writeHead(200, { 'Content-Type': contentType });
            res.end(data);
        } catch (error) {
            if (error.code === 'ENOENT') {
                res.writeHead(404);
                res.end('404 Not Found');
            } else {
                res.writeHead(500);
                res.end('500 Internal Server Error');
            }
        }
    });

    server.listen(port, () => {
        console.log(`Server running at http://localhost:${port}/`);
    });

    return server;
} 