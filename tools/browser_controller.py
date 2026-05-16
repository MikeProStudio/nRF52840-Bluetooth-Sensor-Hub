import asyncio
import logging
import urllib.parse

logger = logging.getLogger("browser")

try:
    from playwright.async_api import async_playwright
    _playwright_available = True
except ImportError:
    _playwright_available = False
    logger.warning("playwright not installed")

_USER_AGENT = "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/125.0.0.0 Safari/537.36"
_LAUNCH_ARGS = [
    "--start-maximized",
    "--disable-blink-features=AutomationControlled",
    "--disable-automation",
    "--no-sandbox",
    "--disable-infobars",
    "--disable-dev-shm-usage",
    "--disable-web-security",
    "--disable-features=IsolateOrigins,site-per-process",
]


class BrowserController:
    def __init__(self):
        self._playwright = None
        self.browser = None
        self.page = None
        self._available = _playwright_available

    async def ensure_browser(self):
        if self.page and not self.page.is_closed():
            return
        if self.browser and self.browser.is_connected():
            ctx = self.browser.contexts[0] if self.browser.contexts else None
            if ctx:
                self.page = await ctx.new_page()
                logger.info("New page opened in existing browser")
                return
        if not self.browser or not self.browser.is_connected():
            await self.launch()

    async def launch(self, headless=False):
        if not self._available:
            raise RuntimeError("playwright not installed")
        self._playwright = await async_playwright().start()
        self.browser = await self._playwright.chromium.launch(
            headless=headless,
            args=_LAUNCH_ARGS,
        )
        ctx = await self.browser.new_context(
            no_viewport=True,
            user_agent=_USER_AGENT,
        )
        # Remove webdriver flag
        await ctx.add_init_script(
            "Object.defineProperty(navigator, 'webdriver', {get: () => undefined})"
        )
        self.page = await ctx.new_page()
        logger.info("Browser launched (stealth)")

    async def close(self):
        if self.browser:
            await self.browser.close()
        if self._playwright:
            await self._playwright.stop()
        self.page = None
        self.browser = None
        self._playwright = None

    async def open_url(self, url: str):
        await self.ensure_browser()
        if not url.startswith("http"):
            url = "https://" + url
        await self.page.goto(url, wait_until="domcontentloaded")
        logger.info("Navigated to %s", url)
        return f"Geöffnet: {url}"

    async def search(self, query: str, engine: str = "duckduckgo"):
        await self.ensure_browser()
        urls = {
            "google": f"https://www.google.com/search?q={urllib.parse.quote(query)}",
            "bing": f"https://www.bing.com/search?q={urllib.parse.quote(query)}",
            "duckduckgo": f"https://duckduckgo.com/?q={urllib.parse.quote(query)}",
            "youtube": f"https://www.youtube.com/results?search_query={urllib.parse.quote(query)}",
        }
        url = urls.get(engine, urls["duckduckgo"])
        await self.page.goto(url, wait_until="domcontentloaded")
        logger.info("Searched %s for '%s'", engine, query)
        return f"Suche nach '{query}' auf {engine}"

    async def click(self, selector: str = None):
        await self.ensure_browser()
        if selector:
            try:
                await self.page.click(selector, timeout=3000)
                return f"Geklickt: {selector}"
            except Exception:
                pass
        await self.page.keyboard.press("Enter")
        return "Enter gedrückt"

    async def scroll(self, direction: str = "down", amount: str = "half"):
        await self.ensure_browser()
        pixels = {"small": 200, "half": 500, "page": 800, "full": 2000}
        delta = pixels.get(amount, 500)
        if direction == "up":
            delta = -delta
        await self.page.evaluate(f"window.scrollBy(0, {delta})")
        return f"Gesrollt {direction} ({amount})"

    async def type_text(self, text: str):
        await self.ensure_browser()
        await self.page.keyboard.type(text, delay=50)
        return f"Text eingegeben: {text[:30]}..."

    async def go_back(self):
        await self.ensure_browser()
        await self.page.go_back()
        return "Zurück navigiert"

    async def refresh(self):
        await self.ensure_browser()
        await self.page.reload()
        return "Seite neu geladen"

    async def close_tab(self):
        if self.page and not self.page.is_closed():
            await self.page.close()
        self.page = None
        return "Tab geschlossen"

    async def screenshot(self, path: str = None):
        await self.ensure_browser()
        if not path:
            import tempfile
            path = tempfile.mktemp(suffix=".png")
        await self.page.screenshot(path=path, full_page=False)
        logger.info("Screenshot saved to %s", path)
        return f"Screenshot erstellt: {path}"

    @property
    def available(self) -> bool:
        return self._available
