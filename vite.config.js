import { defineConfig } from 'vite';
import electron from 'vite-plugin-electron';

export default defineConfig({
  plugins: [
    electron({
      entry: 'src/main.js',
    }),
  ],
  build: {
    outDir: 'dist',
    emptyOutDir: true,
  },
  server: {
    port: 3000,
  },
}); 