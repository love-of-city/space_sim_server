import { defineConfig } from "vite";
import { resolve } from "node:path";

export default defineConfig({
  base: "/static/",
  build: {
    outDir: "dist",
    emptyOutDir: true,
    sourcemap: true,
    rollupOptions: {
      input: {
        index: resolve(import.meta.dirname, "index.html"),
        rendezvous: resolve(import.meta.dirname, "rendezvous.html"),
      },
    },
  },
});
