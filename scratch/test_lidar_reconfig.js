const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

const ARTIFACT_DIR = '/Users/wangfeifei/.gemini/antigravity/brain/da8ffdd7-ea5f-421b-b0c5-21f75e506e46';
const TARGET_URL = 'https://agv.cloud-ai.work';

async function main() {
  console.log(`\n======================================================`);
  console.log(`📡 Testing Dynamic LiDAR Angular Resolution & Frame Rate on ${TARGET_URL}`);
  console.log(`======================================================\n`);

  const browser = await chromium.launch({
    headless: true,
    executablePath: '/Users/wangfeifei/Library/Caches/ms-playwright/chromium-1217/chrome-mac-arm64/Google Chrome for Testing.app/Contents/MacOS/Google Chrome for Testing'
  });

  const context = await browser.newContext({
    viewport: { width: 1440, height: 900 },
    deviceScaleFactor: 1
  });
  const page = await context.newPage();

  console.log(`1. Navigating to ${TARGET_URL} ...`);
  await page.goto(TARGET_URL, { waitUntil: 'domcontentloaded', timeout: 30000 });
  await page.waitForTimeout(2500);

  // 1. Verify LiDAR card exists
  const cardExists = await page.evaluate(() => {
    return !!document.getElementById('card-lidar-config');
  });
  console.log(`2. LiDAR Parameter Card present: ${cardExists}`);

  // 2. Initial state
  const initialCfg = await page.evaluate(async () => {
    const res = await fetch('/api/lidar_config');
    return await res.json();
  });
  console.log(`3. Current LiDAR Config:`, initialCfg);

  // Capture initial screenshot (sidebar with lidar card)
  const initialShotPath = path.join(ARTIFACT_DIR, 'lidar_config_initial.png');
  await page.screenshot({ path: initialShotPath });
  console.log(`📸 Saved initial screenshot to: ${initialShotPath}`);

  // 3. Switch to 720 lines (0.5°), 20 Hz (High Density & High Frame Rate)
  console.log(`\n4. Switching LiDAR to 720 lines (0.5°) and 20 Hz ...`);
  await page.evaluate(async () => {
    const resSelect = document.getElementById('lidarResolutionSelect');
    const freqSelect = document.getElementById('lidarFreqSelect');
    if (resSelect) resSelect.value = '720';
    if (freqSelect) freqSelect.value = '20';
    if (typeof updateLidarConfig === 'function') {
      await updateLidarConfig();
    }
  });

  await page.waitForTimeout(2000);

  // Check state after 720 lines
  const state720 = await page.evaluate(async () => {
    const res = await fetch('/api/telemetry');
    const tel = await res.json();
    const resBm = await fetch('/api/bullet_metrics');
    const bm = await resBm.json();
    return {
      lidar_config: tel.lidar_config,
      ranges_count: tel.scan_ranges ? tel.scan_ranges.length : 0,
      bullet_data_volume: bm.data_volume,
      bullet_perf: bm.performance,
      ui_beams: document.getElementById('lidar-active-beams')?.textContent,
      ui_freq: document.getElementById('lidar-active-freq')?.textContent,
      ui_throughput: document.getElementById('lidar-active-throughput')?.textContent
    };
  });
  console.log(`5. 720-beam state:`, JSON.stringify(state720, null, 2));

  const shot720Path = path.join(ARTIFACT_DIR, 'lidar_config_720_20hz.png');
  await page.screenshot({ path: shot720Path });
  console.log(`📸 Saved 720-beam screenshot to: ${shot720Path}`);

  // 4. Switch to 90 lines (4.0°), 5 Hz (Minimal Low-Power Mode)
  console.log(`\n6. Switching LiDAR to 90 lines (4.0°) and 5 Hz ...`);
  await page.evaluate(async () => {
    const resSelect = document.getElementById('lidarResolutionSelect');
    const freqSelect = document.getElementById('lidarFreqSelect');
    if (resSelect) resSelect.value = '90';
    if (freqSelect) freqSelect.value = '5';
    if (typeof updateLidarConfig === 'function') {
      await updateLidarConfig();
    }
  });

  await page.waitForTimeout(2000);

  const state90 = await page.evaluate(async () => {
    const res = await fetch('/api/telemetry');
    const tel = await res.json();
    const resBm = await fetch('/api/bullet_metrics');
    const bm = await resBm.json();
    return {
      lidar_config: tel.lidar_config,
      ranges_count: tel.scan_ranges ? tel.scan_ranges.length : 0,
      bullet_data_volume: bm.data_volume,
      bullet_perf: bm.performance,
      ui_beams: document.getElementById('lidar-active-beams')?.textContent,
      ui_freq: document.getElementById('lidar-active-freq')?.textContent,
      ui_throughput: document.getElementById('lidar-active-throughput')?.textContent
    };
  });
  console.log(`7. 90-beam state:`, JSON.stringify(state90, null, 2));

  const shot90Path = path.join(ARTIFACT_DIR, 'lidar_config_90_5hz.png');
  await page.screenshot({ path: shot90Path });
  console.log(`📸 Saved 90-beam screenshot to: ${shot90Path}`);

  // 5. Restore to 360 lines (1.0°), 10 Hz (Production Baseline)
  console.log(`\n8. Restoring LiDAR to 360 lines (1.0°) and 10 Hz ...`);
  await page.evaluate(async () => {
    const resSelect = document.getElementById('lidarResolutionSelect');
    const freqSelect = document.getElementById('lidarFreqSelect');
    if (resSelect) resSelect.value = '360';
    if (freqSelect) freqSelect.value = '10';
    if (typeof updateLidarConfig === 'function') {
      await updateLidarConfig();
    }
  });

  await page.waitForTimeout(2000);

  // Open Performance Modal to verify benchmark dashboard
  await page.evaluate(() => {
    if (typeof togglePerfModal === 'function') {
      togglePerfModal();
    }
  });
  await page.waitForTimeout(1500);

  const perfBenchmarkShotPath = path.join(ARTIFACT_DIR, 'lidar_perf_benchmark_verified.png');
  await page.screenshot({ path: perfBenchmarkShotPath });
  console.log(`📸 Saved perf benchmark screenshot to: ${perfBenchmarkShotPath}`);

  await browser.close();
  console.log(`\n✅ Dynamic LiDAR resolution and frame rate testing complete!\n`);
}

main().catch(err => {
  console.error("Test failed:", err);
  process.exit(1);
});
