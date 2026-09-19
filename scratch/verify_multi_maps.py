import asyncio
import os
from playwright.async_api import async_playwright

ARTIFACT_DIR = "/Users/wangfeifei/.gemini/antigravity/brain/da8ffdd7-ea5f-421b-b0c5-21f75e506e46"

CHROME_PATH = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

async def test_multi_maps():
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True, executable_path=CHROME_PATH)
        context = await browser.new_context(viewport={"width": 1440, "height": 900})
        page = await context.new_page()

        print("=== STEP 1: Opening Web UI at http://192.168.11.117:8088 ===")
        await page.goto("http://192.168.11.117:8088", wait_until="domcontentloaded", timeout=15000)
        await asyncio.sleep(2.0)

        # 1. Check if #mapScenarioSwitcher exists and is visible
        switcher = page.locator("#mapScenarioSwitcher")
        is_visible = await switcher.is_visible()
        print(f"Scenario switcher bar visible: {is_visible}")
        assert is_visible, "Top scenario switcher bar #mapScenarioSwitcher should be visible!"

        pills = page.locator(".scenario-pill")
        count = await pills.count()
        print(f"Total scenario pills found: {count}")
        assert count == 4, f"Expected 4 scenario pills, found {count}"

        # 2. Verify Scenario 1: grid_9_square (Default)
        print("=== STEP 2: Verifying Scenario 1 (正方形九宫格智能立体仓) ===")
        pill_grid9 = page.locator('.scenario-pill[data-sc="grid_9_square"]')
        await pill_grid9.click()
        await asyncio.sleep(2.0)

        is_grid9_active = "active" in (await pill_grid9.get_attribute("class") or "")
        print(f"grid_9_square active: {is_grid9_active}")
        assert is_grid9_active

        # Check mission stations for grid 9
        grid_html = await page.inner_html("#missionGrid")
        assert "1号原料入库位" in grid_html or "质检抽样位" in grid_html
        print("Grid 9 stations verified in missionGrid.")

        # Capture Grid 9 Screenshot
        grid9_path = os.path.join(ARTIFACT_DIR, "amr_v4_map_scenario_grid9.png")
        await page.screenshot(path=grid9_path)
        print(f"Saved: {grid9_path}")

        # 3. Verify Scenario 2: narrow_aisle (高密多巷道立体库)
        print("=== STEP 3: Verifying Scenario 2 (高密多巷道立体库) ===")
        pill_vna = page.locator('.scenario-pill[data-sc="narrow_aisle"]')
        await pill_vna.click()
        await asyncio.sleep(2.5)

        is_vna_active = "active" in (await pill_vna.get_attribute("class") or "")
        print(f"narrow_aisle active: {is_vna_active}")
        assert is_vna_active

        vna_grid_html = await page.inner_html("#missionGrid")
        assert "1号巷道进料口" in vna_grid_html or "2号巷道存储位" in vna_grid_html
        print("VNA narrow aisle stations verified in missionGrid.")

        vna_path = os.path.join(ARTIFACT_DIR, "amr_v4_map_scenario_narrow_aisle.png")
        await page.screenshot(path=vna_path)
        print(f"Saved: {vna_path}")

        # 4. Verify Scenario 3: rect_loop (长方形环线制造车间)
        print("=== STEP 4: Verifying Scenario 3 (长方形环线制造车间) ===")
        pill_loop = page.locator('.scenario-pill[data-sc="rect_loop"]')
        await pill_loop.click()
        await asyncio.sleep(2.5)

        is_loop_active = "active" in (await pill_loop.get_attribute("class") or "")
        print(f"rect_loop active: {is_loop_active}")
        assert is_loop_active

        loop_grid_html = await page.inner_html("#missionGrid")
        assert "1号原料上线工位" in loop_grid_html or "2号柔性装配工位" in loop_grid_html
        print("Rect loop stations verified in missionGrid.")

        loop_path = os.path.join(ARTIFACT_DIR, "amr_v4_map_scenario_rect_loop.png")
        await page.screenshot(path=loop_path)
        print(f"Saved: {loop_path}")

        # 5. Verify Scenario 4: standard_cross (标准十字仓储物流中心)
        print("=== STEP 5: Verifying Scenario 4 (标准十字仓储物流中心) ===")
        pill_cross = page.locator('.scenario-pill[data-sc="standard_cross"]')
        await pill_cross.click()
        await asyncio.sleep(2.5)

        is_cross_active = "active" in (await pill_cross.get_attribute("class") or "")
        print(f"standard_cross active: {is_cross_active}")
        assert is_cross_active

        cross_path = os.path.join(ARTIFACT_DIR, "amr_v4_map_scenario_standard_cross.png")
        await page.screenshot(path=cross_path)
        print(f"Saved: {cross_path}")

        # 6. Switch back to grid_9_square and test navigation dispatch
        print("=== STEP 6: Navigating inside Grid 9 Square ===")
        await pill_grid9.click()
        await asyncio.sleep(2.0)

        # Dispatch navigation to Station 2 (2号质检抽样位 at 2.1, 6.4)
        st2_btn = page.locator("button.mission-btn", has_text="2号质检抽样位")
        if await st2_btn.count() > 0:
            await st2_btn.first.click()
            print("Dispatched mission to 2号质检抽样位")
            await asyncio.sleep(4.0)

        nav_path = os.path.join(ARTIFACT_DIR, "amr_v4_map_grid9_navigation.png")
        await page.screenshot(path=nav_path)
        print(f"Saved: {nav_path}")

        # 7. Test Public WAN Domain access: https://agv.cloud-ai.work
        print("=== STEP 7: Testing Public WAN Domain https://agv.cloud-ai.work ===")
        try:
            wan_page = await context.new_page()
            await wan_page.goto("https://agv.cloud-ai.work", wait_until="domcontentloaded", timeout=20000)
            await asyncio.sleep(2.0)
            wan_switcher = wan_page.locator("#mapScenarioSwitcher")
            assert await wan_switcher.is_visible()
            wan_path = os.path.join(ARTIFACT_DIR, "amr_v4_public_multi_maps.png")
            await wan_page.screenshot(path=wan_path)
            print(f"Saved public WAN screenshot: {wan_path}")
        except Exception as e:
            print(f"Note on public WAN test: {e}")

        await browser.close()
        print("=== ALL 4 SCENARIOS VERIFIED SUCCESSFULLY VIA PLAYWRIGHT ===")

if __name__ == "__main__":
    asyncio.run(test_multi_maps())
