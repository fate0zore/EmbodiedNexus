import { defineConfig } from "vite";

export default defineConfig({
  base: "/rviz/",
  build: {
    outDir: "dist", emptyOutDir: true, sourcemap: false,
    rollupOptions: { output: { manualChunks(id) {
      if (id.includes("/node_modules/three/")) return "three-vendor";
      if (id.includes("/node_modules/cbor-x/")) return "cbor-vendor";
      if (id.includes("/node_modules/urdf-loader/")) return "urdf-vendor";
    } } },
  },
});
