import { defineConfig } from "vite";

export default defineConfig({
  base: "/static/",
  build: {
    outDir: "dist",
    emptyOutDir: true,
    sourcemap: true,
  },
});
