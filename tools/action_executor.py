import logging

logger = logging.getLogger("executor")


class ActionExecutor:
    def __init__(self, browser, system):
        self.browser = browser
        self.system = system

    async def execute(self, cmd: dict) -> str:
        action = cmd.get("action", "none")
        logger.info("Executing action: %s %s", action, {k: v for k, v in cmd.items() if k != "action"})
        try:
            handler = getattr(self, f"_exec_{action}", None)
            if handler:
                return await handler(cmd)
            return f"Unbekannte Aktion: {action}"
        except Exception as e:
            logger.error("Action '%s' failed: %s", action, e)
            return f"Fehler bei '{action}': {e}"

    async def _exec_browser_open(self, cmd: dict) -> str:
        url = cmd.get("url", cmd.get("target", ""))
        return await self.browser.open_url(url)

    async def _exec_browser_search(self, cmd: dict) -> str:
        query = cmd.get("query", "")
        engine = cmd.get("engine", "google")
        return await self.browser.search(query, engine)

    async def _exec_browser_click(self, cmd: dict) -> str:
        return await self.browser.click(cmd.get("selector"))

    async def _exec_browser_scroll(self, cmd: dict) -> str:
        return await self.browser.scroll(
            cmd.get("direction", "down"),
            cmd.get("amount", "half")
        )

    async def _exec_browser_type(self, cmd: dict) -> str:
        return await self.browser.type_text(cmd.get("text", ""))

    async def _exec_browser_back(self, cmd: dict = None) -> str:
        return await self.browser.go_back()

    async def _exec_browser_close(self, cmd: dict = None) -> str:
        return await self.browser.close_tab()

    async def _exec_browser_refresh(self, cmd: dict = None) -> str:
        return await self.browser.refresh()

    async def _exec_app_launch(self, cmd: dict) -> str:
        app = cmd.get("app", cmd.get("target", ""))
        return self.system.launch_app(app)

    async def _exec_app_close(self, cmd: dict) -> str:
        app = cmd.get("app", cmd.get("target", ""))
        return self.system.close_app(app)

    async def _exec_keyboard_type(self, cmd: dict) -> str:
        return self.system.keyboard_type(cmd.get("text", ""))

    async def _exec_mouse_click(self, cmd: dict) -> str:
        return self.system.mouse_click(cmd.get("x"), cmd.get("y"))

    async def _exec_screenshot(self, cmd: dict = None) -> str:
        return self.system.screenshot()

    async def _exec_volume_set(self, cmd: dict) -> str:
        return self.system.volume_set(int(cmd.get("level", 50)))

    async def _exec_system_command(self, cmd: dict) -> str:
        return self.system.run_command(cmd.get("command", ""))
