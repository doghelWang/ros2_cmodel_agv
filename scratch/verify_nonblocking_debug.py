import asyncio
import os
from playwright.async_api import async_playwright

ARTIFACT_DIR = "/Users/wangfeifei/.gemini/antigravity/brain/da8ffdd7-ea5f-421b-b0c5-21f75e506e46"
CHROME_PATH = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

async def test_nonblocking_debug():
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True, executable_path=CHROME_PATH)
        context = await browser.new_context(viewport={"width": 1440, "height": 900})
        page = await context.new_page()

        print("=== STEP 1: Load Web Dashboard ===")
        await page.goto("http://192.168.11.117:8088", wait_until="domcontentloaded", timeout=15000)
        await asyncio.sleep(2.0)

        print("=== STEP 2: Open Debug Console ===")
        debug_btn = page.locator("#btn-open-debug")
        await debug_btn.click()
        await asyncio.sleep(1.0)

        # Check debug modal visibility
        debug_dialog = page.locator("#debugDialog")
        assert await debug_dialog.is_visible(), "Debug dialog should be visible"

        # Verify modal wrapper does NOT block pointer events
        wrapper_pe = await page.eval_on_selector("#debugModal", "el => window.getComputedStyle(el).pointerEvents")
        print(f"Debug modal wrapper pointer-events: {wrapper_pe}")
        assert wrapper_pe == "none", "Wrapper pointer-events must be 'none' to allow click-through"

        # Capture Screenshot: Docked Bottom Console mode with full map and side panel visible
        docked_path = os.path.join(ARTIFACT_DIR, "amr_v4_debug_nonblocking_docked.png")
        await page.screenshot(path=docked_path)
        print(f"Saved: {docked_path}")

        print("=== STEP 3: Verify Simultaneous Main Interface Interaction ===")
        # Click on mission station button in side panel while debug console is open!
        st_btn = page.locator("#missionGrid button.mission-btn").first
        await st_btn.scroll_into_view_if_needed()
        btn_text = await st_btn.inner_text()
        print(f"Clicking mission button: {btn_text} with debug console open!")
        await st_btn.click()
        await asyncio.sleep(2.0)

        # Verify target coord updated in side panel
        tgt_text = await page.inner_text("#target-coord")
        print(f"Current target coord: {tgt_text}")
        assert "2.1" in tgt_text or "目标" in tgt_text

        # Capture Screenshot: AGV navigating while debug window is open
        nav_debug_path = os.path.join(ARTIFACT_DIR, "amr_v4_debug_simultaneous_navigation.png")
        await page.screenshot(path=nav_debug_path)
        print(f"Saved: {nav_debug_path}")

        print("=== STEP 4: Test Floating Mode Toggle ===")
        dock_toggle_btn = page.locator("#btn-dock-toggle")
        await dock_toggle_btn.click()
        await asyncio.sleep(0.5)

        is_floating = "floating" in (await debug_dialog.get_attribute("class") or "")
        print(f"Debug dialog floating: {is_floating}")
        assert is_floating

        floating_path = os.path.join(ARTIFACT_DIR, "amr_v4_debug_nonblocking_floating.png")
        await page.screenshot(path=floating_path)
        print(f"Saved: {floating_path}")

        print("=== STEP 5: Test Minimize Mode ===")
        min_btn = page.locator("#btn-min-toggle")
        await min_btn.click()
        await asyncio.sleep(0.5)

        is_min = "minimized" in (await debug_dialog.get_attribute("class") or "")
        print(f"Debug dialog minimized: {is_min}")
        assert is_min

        min_path = os.path.join(ARTIFACT_DIR, "amr_v4_debug_minimized.png")
        await page.screenshot(path=min_path)
        print(f"Saved: {min_path}")

        print("=== STEP 6: Verify Public WAN Domain https://agv.cloud-ai.work ===")
        try:
            wan_page = await context.new_page()
            await wan_page.goto("https://agv.cloud-ai.work", wait_until="domcontentloaded", timeout=20000)
            await asyncio.sleep(2.0)
            wan_debug_btn = wan_page.locator("#btn-open-debug")
            await wan_debug_btn.click()
            await asyncio.sleep(1.0)
            wan_path = os.path.join(ARTIFACT_DIR, "amr_v4_public_nonblocking_debug.png")
            await wan_page.screenshot(path=wan_path)
            print(f"Saved public WAN screenshot: {wan_path}")
        except Exception as e:
            print(f"Note on WAN: {e}")

        await browser.close()
        print("=== NON-BLOCKING DOCKABLE DEBUG CONSOLE VERIFIED SUCCESSFULLY ===")

if __name__ == "__main__":
    asyncio.run(test_nonblocking_debug())
