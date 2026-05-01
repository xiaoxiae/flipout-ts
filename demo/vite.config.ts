import { defineConfig } from 'vite';
import { resolve } from 'node:path';

export default defineConfig({
  resolve: {
    alias: {
      'flipout-ts/three': resolve(__dirname, '../src/three/index.ts'),
      'flipout-ts/flipout': resolve(__dirname, '../src/flipout/index.ts'),
      'flipout-ts': resolve(__dirname, '../src/index.ts'),
    },
  },
});
