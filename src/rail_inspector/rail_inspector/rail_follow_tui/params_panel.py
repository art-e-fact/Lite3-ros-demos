"""Parameters tab: one Collapsible section per ROS node."""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import VerticalScroll
from textual.widgets import Collapsible

from rail_inspector.rail_follow_tui.param_row import ParamRow, dom_id
from rail_inspector.rail_follow_tui.ros_state import (
    RosState,
    params_by_node,
    params_tab_blacklist,
)


class ParamsPanel(VerticalScroll):
    def __init__(self, ros: RosState) -> None:
        self.ros = ros
        super().__init__()

    def compose(self) -> ComposeResult:
        grouped = params_by_node(
            self.ros.params,
            params_tab_blacklist(self.ros.follower_node),
        )
        for index, (node_name, node_params) in enumerate(sorted(grouped.items())):
            with Collapsible(title=node_name, collapsed=index > 0):
                for param in node_params:
                    yield ParamRow(param, id=dom_id(param.key))

    def sync_all(self) -> None:
        for row in self.query(ParamRow):
            row.sync_from_param()
