import { defineConfig } from 'vite';
import { resolve } from 'node:path';

export default defineConfig({
  resolve: {
    alias: {
      '@chalkbag/flipout-ts/three': resolve(__dirname, '../flipout-ts/src/three/index.ts'),
      '@chalkbag/flipout-ts/flipout': resolve(__dirname, '../flipout-ts/src/flipout/index.ts'),
      '@chalkbag/flipout-ts': resolve(__dirname, '../flipout-ts/src/index.ts'),
    },
  },
});
