const { defineConfig } = require('@playwright/test');

const out = process.env.VIEWER_BROWSER_OUTPUT || '/ws/build/browser';

module.exports = defineConfig({
  testDir: __dirname,
  outputDir: `${out}/results`,
  timeout: 90000,
  expect: { timeout: 30000 },
  workers: 1,
  reporter: [['list']],
  use: {
    viewport: { width: 1000, height: 700 },
    // Software WebGL: containers and CI have no GPU
    launchOptions: {
      args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-webgl', '--ignore-gpu-blocklist',
             '--enable-unsafe-swiftshader'],
    },
  },
});
