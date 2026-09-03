"""Headless render check of the dashboard, including the 3D view.

Drives a running dashboard with Playwright's Chromium: selects a series,
screenshots the top-down view, switches to the 3D tab, waits for a
frame, screenshots again, and fails on any page error.  Needs a live
episode (a daemon the dashboard proxies) for the point cloud; walls and
robots render without one.

    python scripts/dashboard_render_check.py http://127.0.0.1:8080/ duo12

Env: PW_EXEC points at a Chromium binary when Playwright's own download
is absent (e.g. /opt/pw-browsers/chromium-*/chrome-linux/chrome);
OUT_DIR sets where the PNGs go (default: the current directory).
"""

import os
import sys

from playwright.sync_api import sync_playwright


def main():
    url = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8080/"
    series = sys.argv[2] if len(sys.argv) > 2 else None
    out = os.environ.get("OUT_DIR", ".")
    errors = []
    with sync_playwright() as p:
        b = p.chromium.launch(
            executable_path=os.environ.get("PW_EXEC") or None,
            args=["--use-gl=swiftshader", "--enable-webgl",
                  "--ignore-gpu-blocklist"])
        page = b.new_page(viewport={"width": 1500, "height": 900})
        page.on("pageerror", lambda e: errors.append(f"pageerror: {e}"))
        page.on("console", lambda m: errors.append(f"console: {m.text}")
                if m.type == "error" else None)
        page.goto(url, wait_until="networkidle")
        if series:
            page.select_option("#series-sel", series)
        page.wait_for_timeout(1500)
        page.screenshot(path=os.path.join(out, "dashboard_2d.png"))
        page.click('#view-tabs button[data-view="3d"]')
        page.wait_for_timeout(3000)
        page.screenshot(path=os.path.join(out, "dashboard_3d.png"))
        info = page.evaluate("""() => ({
            active: !!(window.view3dActive && window.view3dActive()),
            canvases: document.querySelectorAll('#view3d canvas').length,
            note: (document.querySelector('#view3d-note') || {}).textContent,
            three: typeof THREE !== 'undefined' ? THREE.REVISION : null,
            badge: document.querySelector('#live-badge').textContent,
        })""")
        b.close()
    print("info", info)
    ok = info["active"] and info["canvases"] == 1 and not errors
    for e in errors[:10]:
        print("  ", e[:200])
    print("PASS" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
