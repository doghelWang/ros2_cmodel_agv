const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

const ARTIFACT_DIR = '/Users/wangfeifei/.gemini/antigravity/brain/da8ffdd7-ea5f-421b-b0c5-21f75e506e46';
const TARGET_URL = 'https://agv.cloud-ai.work';

async function main() {
  console.log(`\n======================================================`);
  console.log(`⚡ Testing PyBullet Engine & Performance on ${TARGET_URL}`);
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
  await page.waitForTimeout(2000);

  // 1. Open Performance Monitor Modal
  console.log(`2. Opening Performance Monitor Modal ...`);
  await page.evaluate(() => {
    if (typeof togglePerfModal === 'function') {
      togglePerfModal();
    }
  });
  await page.waitForTimeout(1500);

  // 2. Fetch and verify performance data
  const perfData = await page.evaluate(async () => {
    const res = await fetch('/api/system_perf');
    return await res.json();
  });

  console.log(`3. Host Performance:`);
  console.log(`   - Model: ${perfData.host.model}`);
  console.log(`   - Total CPU: ${perfData.host.cpu_total_percent}%`);
  console.log(`   - Per Core: ${JSON.stringify(perfData.host.cpu_per_core)}`);
  console.log(`   - Temperature: ${perfData.host.cpu_temp_c} °C`);
  console.log(`   - RAM: ${perfData.host.memory_used_mb}MB / ${perfData.host.memory_total_mb}MB (${perfData.host.memory_percent}%)`);
  
  console.log(`\n4. Process Breakdown:`);
  for (const p of (perfData.processes || [])) {
    console.log(`   * [PID ${p.pid}] ${p.name} | CPU: ${p.cpu_percent}% | RSS: ${p.memory_mb}MB | Role: ${p.role}`);
  }

  // Take screenshot of Performance Monitor Modal (Top overview)
  const perfShotPath = path.join(ARTIFACT_DIR, 'pybullet_perf_monitor.png');
  await page.screenshot({ path: perfShotPath });
  console.log(`\n📸 Saved performance monitor snapshot to: ${perfShotPath}`);

  // Scroll down to capture the Bullet Data Volume vs Performance Benchmark Dashboard
  await page.evaluate(() => {
    const dialogBody = document.querySelector('.perf-dialog-body');
    if (dialogBody) dialogBody.scrollTop = 500;
  });
  await page.waitForTimeout(1000);

  const benchShotPath = path.join(ARTIFACT_DIR, 'pybullet_benchmark_table.png');
  await page.screenshot({ path: benchShotPath });
  console.log(`📸 Saved bullet benchmark table snapshot to: ${benchShotPath}`);

  // 5. Trigger Navigation and Capture Motion
  console.log(`\n5. Dispatching Navigation Target [3.0, 3.0] under PyBullet motor control ...`);
  await page.evaluate(async () => {
    await fetch('/api/navigate_to_pose', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x: 3.0, y: 3.0, yaw: 0.0 })
    });
  });

  // Track motion for 3 seconds
  await page.waitForTimeout(2000);
  const liveTelemetry = await page.evaluate(async () => {
    const res = await fetch('/api/telemetry');
    return await res.json();
  });

  console.log(`6. Navigation Live State:`);
  console.log(`   - Status: ${liveTelemetry.nav_status}`);
  console.log(`   - Pose: (${liveTelemetry.x.toFixed(3)}, ${liveTelemetry.y.toFixed(3)}, ${(liveTelemetry.yaw * 57.3).toFixed(1)}°)`);
  console.log(`   - Velocity: vx=${liveTelemetry.vx} m/s, wz=${liveTelemetry.wz} rad/s`);
  console.log(`   - Wheel Velocities: ${JSON.stringify(liveTelemetry.joint_states.velocities)}`);
  console.log(`   - LiDAR Beams: ${liveTelemetry.scan_ranges ? liveTelemetry.scan_ranges.length : 0} (Min Dist: ${liveTelemetry.scan_min_dist}m)`);

  const navShotPath = path.join(ARTIFACT_DIR, 'pybullet_live_navigation.png');
  await page.screenshot({ path: navShotPath });
  console.log(`📸 Saved live navigation snapshot to: ${navShotPath}`);

  await browser.close();
  console.log(`\n✅ PyBullet Engine & Performance verification complete!\n`);
}

main().catch(err => {
  console.error("Test failed:", err);
  process.exit(1);
});
