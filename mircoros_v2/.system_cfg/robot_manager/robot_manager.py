#!/usr/bin/env python3
# --- Path bootstrap (must run before importing textual / serial) ----------
import os
import sys

_venv_site = os.path.join(
    os.path.expanduser("~"), "microros_ws", "python_env",
    "mircoros_v2_env", ".venv", "lib", "python3.10", "site-packages")
if os.path.isdir(_venv_site):
    sys.path.insert(0, _venv_site)

_self_dir = os.path.dirname(os.path.abspath(__file__))
if _self_dir not in sys.path:
    sys.path.insert(0, _self_dir)

# --- Real imports ----------------------------------------------------------
from textual.app import App, ComposeResult
from textual.widgets import Footer, TabbedContent, TabPane

from i18n import tr
from panels import BoardPanel, HostPanel, InfoPanel


class RobotManagerApp(App):
    """Main application with three tabbed panels."""

    TITLE = "robot_manager"
    CSS_PATH = "robot_manager.tcss"
    ENABLE_COMMAND_PALETTE = False
    BINDINGS = [
        ("q", "quit", "Quit"),
        ("r", "refresh_info", "Refresh"),
    ]

    def compose(self) -> ComposeResult:
        self.sub_title = tr("app.subtitle")
        with TabbedContent(initial="tab-info"):
            with TabPane(tr("tab.info"), id="tab-info"):
                yield InfoPanel(id="panel-info")
            with TabPane(tr("tab.host"), id="tab-host"):
                yield HostPanel(id="panel-host")
            with TabPane(tr("tab.board"), id="tab-board"):
                yield BoardPanel(id="panel-board")
        yield Footer()

    def action_refresh_info(self) -> None:
        try:
            self.query_one("#panel-info", InfoPanel).refresh_info()
        except Exception:
            pass

def main():
    RobotManagerApp().run()


if __name__ == "__main__":
    main()
