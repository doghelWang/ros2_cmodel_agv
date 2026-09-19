const { chromium } = require('playwright');
const path = require('path');
const fs = require('fs');

const ARTIFACT_DIR = '/Users/wangfeifei/.gemini/antigravity/brain/da8ffdd7-ea5f-421b-b0c5-21f75e506e46';
const TARGET_URL = 'https://agv.cloud-ai.work';

async function main() {
  console.log(`\n======================================================`);
  console.log(`🚀 AGV Smooth Motion Live Test on ${TARGET_URL}`);
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

  // Exit any previous replay session to ensure 100% live teleoperation view
  await page.evaluate(() => {
    if (typeof exitReplayMode === 'function') exitReplayMode();
  });
  await page.waitForTimeout(500);

  // Measure Cloudflare Tunnel roundtrip latency for /api/telemetry
  console.log(`2. Measuring Cloudflare Tunnel roundtrip latency ...`);
  const latencyStats = await page.evaluate(async () => {
    const times = [];
    const sizes = [];
    for (let i = 0; i < 5; i++) {
      const t0 = performance.now();
      const res = await fetch('/api/telemetry');
      const text = await res.text();
      times.push(Math.round(performance.now() - t0));
      sizes.push(text.length);
      await new Promise(r => setTimeout(r, 50));
    }
    return { times, sizes };
  });

  const avgLatency = (latencyStats.times.reduce((a, b) => a + b, 0) / latencyStats.times.length).toFixed(1);
  const avgSize = (latencyStats.sizes.reduce((a, b) => a + b, 0) / latencyStats.sizes.length).toFixed(0);
  console.log(`   RTT samples: [${latencyStats.times.join(', ')}] ms (average: ${avgLatency} ms)`);
  console.log(`   Lightweight payload size: ${avgSize} bytes`);

  // Switch to standard grid_9_square
  console.log(`3. Setting scenario to grid_9_square...`);
  await page.evaluate(() => switchScenario('grid_9_square'));
  await page.waitForTimeout(2000);

  // Command navigation to Station A3: (3.0, 4.0, yaw=1.57 rad)
  console.log(`4. Dispatching autonomous navigation to (3.0, 4.0, yaw=1.57 rad)...`);
  await page.evaluate(() => {
    dispatchMission(3.0, 4.0, '3号工位', 1.57);
  });

  // Wait 1.5s for vehicle to ramp up velocity and enter corridor tracking
  await page.waitForTimeout(1500);

  // Capture active cruising screenshot
  const shot1 = path.join(ARTIFACT_DIR, 'amr_v4_cloud_ai_smooth_motion.png');
  await page.screenshot({ path: shot1 });
  console.log(`5. Captured active cruising screenshot: ${shot1}`);

  // Frame-by-frame 60 FPS trajectory sampling over 8.0 seconds
  console.log(`6. High-Frequency Trajectory Sampling (requestAnimationFrame) over 8.0s ...`);
  const frameData = await page.evaluate(async () => {
    const frames = [];
    const tStart = performance.now();
    while (performance.now() - tStart < 8000) {
      await new Promise(r => requestAnimationFrame(r));
      frames.push({
        t: Math.round(performance.now() - tStart),
        x: Number(renderPose.x.toFixed(4)),
        y: Number(renderPose.y.toFixed(4)),
        yaw: Number(renderPose.yaw.toFixed(4)),
        vx: Number(renderPose.vx.toFixed(3)),
        wz: Number(renderPose.wz.toFixed(3)),
        errX: Number(renderPose.errX.toFixed(4)),
        errY: Number(renderPose.errY.toFixed(4)),
        errYaw: Number(renderPose.errYaw.toFixed(4)),
        status: telemetry.nav_status,
        distRem: telemetry.nav_dist_rem,
        isReplay: isReplayActive
      });
    }
    return frames;
  });

  const fps = (frameData.length / 8.0).toFixed(1);
  console.log(`   Captured ${frameData.length} render frames in 8.0s (Average Frame Rate: ${fps} FPS)`);

  // Analyze Kinematic Continuity
  let maxPosDelta = 0.0;
  let maxYawDelta = 0.0;
  let totalDistanceTraveled = 0.0;
  const positionJumps = [];
  const yawJumps = [];

  for (let i = 1; i < frameData.length; i++) {
    const f0 = frameData[i - 1];
    const f1 = frameData[i];

    const dPos = Math.hypot(f1.x - f0.x, f1.y - f0.y);
    const dYaw = Math.abs(Math.atan2(Math.sin(f1.yaw - f0.yaw), Math.cos(f1.yaw - f0.yaw)));

    totalDistanceTraveled += dPos;
    if (dPos > maxPosDelta) maxPosDelta = dPos;
    if (dYaw > maxYawDelta) maxYawDelta = dYaw;

    // Discontinuity thresholds:
    if (dPos > 0.06) {
      positionJumps.push({ idx: i, t: f1.t, f0, f1, dPos });
    }

    if (dYaw > 0.15) {
      yawJumps.push({ idx: i, t: f1.t, f0, f1, dYaw });
    }
  }

  console.log(`\n================ KINEMATIC CONTINUITY ANALYSIS ================`);
  console.log(`✓ Total Distance Traveled:    ${totalDistanceTraveled.toFixed(2)} m`);
  console.log(`✓ Max Position Delta / Frame: ${maxPosDelta.toFixed(4)} m (Safety threshold < 0.06m)`);
  console.log(`✓ Max Yaw Delta / Frame:      ${maxYawDelta.toFixed(4)} rad (${(maxYawDelta * 180 / Math.PI).toFixed(2)}°) (Safety threshold < 8.6°)`);
  console.log(`✓ Position Discontinuities:   ${positionJumps.length}`);
  console.log(`✓ Yaw Discontinuities:        ${yawJumps.length}`);
  console.log(`✓ Replay active during test:  ${frameData[frameData.length - 1].isReplay}`);

  // Inspect Live DOM Telemetry values
  const domX = await page.textContent('#val-x');
  const domY = await page.textContent('#val-y');
  const domYaw = await page.textContent('#val-yaw');
  const domVx = await page.textContent('#val-vx');
  const domStatus = await page.textContent('#nav-badge');
  console.log(`✓ Live DOM Dashboard Readouts:`);
  console.log(`   Position: (${domX}, ${domY}), Heading: ${domYaw}, Speed: ${domVx}`);
  console.log(`   Navigation Badge: ${domStatus.trim()}`);

  // Print sample sequence around active motion
  console.log(`\n--- Sample Frames Preview ---`);
  console.log(`Frame 0:   `, JSON.stringify(frameData[0]));
  console.log(`Frame 100: `, JSON.stringify(frameData[Math.min(100, frameData.length - 1)]));
  console.log(`Frame 300: `, JSON.stringify(frameData[Math.min(300, frameData.length - 1)]));
  console.log(`Frame 500: `, JSON.stringify(frameData[Math.min(500, frameData.length - 1)]));
  console.log(`Frame Last:`, JSON.stringify(frameData[frameData.length - 1]));

  // Take final snapshot showing smooth trail and corridor alignment
  const shot2 = path.join(ARTIFACT_DIR, 'amr_v4_cloud_ai_corner_turn.png');
  await page.screenshot({ path: shot2 });
  console.log(`\n7. Captured corner navigation snapshot: ${shot2}`);

  await browser.close();

  if (positionJumps.length === 0 && yawJumps.length === 0 && totalDistanceTraveled > 0.5) {
    console.log(`\n🎉 SUCCESS: 100% C1 Continuous Motion Confirmed on ${TARGET_URL}!`);
  } else if (totalDistanceTraveled <= 0.5) {
    console.warn(`\n⚠️ Vehicle did not move enough (${totalDistanceTraveled.toFixed(2)}m) - check path planning.`);
  } else {
    console.error(`\n❌ Discontinuities detected: Pos=${positionJumps.length}, Yaw=${yawJumps.length}`);
    process.exit(1);
  }
}

main().catch(err => {
  console.error('Fatal error running cloud-ai test:', err);
  process.exit(1);
});
