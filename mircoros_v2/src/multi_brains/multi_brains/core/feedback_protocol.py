"""Pure helpers for the Dify action feedback protocol."""

import json


def build_dify_inputs(robot_feedback, image_file_objects=None) -> dict:
    """Build Dify inputs for user requests and robot feedback turns."""
    return {
        "attachments": list(image_file_objects or []),
        "robot_feedback": str(robot_feedback),
    }


def is_finish_action(action) -> bool:
    """Return whether an action ends the current feedback cycle."""
    if not isinstance(action, str):
        return False
    return action.strip().lower() in ("finishtask", "finishtask()")


def partition_actions(actions: list) -> tuple:
    """Separate executable actions from the task-finish marker."""
    executable_actions = [action for action in actions if not is_finish_action(action)]
    return executable_actions, any(is_finish_action(action) for action in actions)


def select_round_action(actions: list, max_per_round: int = 10) -> tuple:
    """Select up to ``max_per_round`` executable actions per feedback round.

    The LLM may batch several sequential actions (e.g. multiple move_forward
    calls to cover a long distance). We execute them all before reporting
    back to Dify for the next round.
    """
    executable_actions, finish_requested = partition_actions(actions)
    selected = executable_actions[:max_per_round]
    remaining = executable_actions[max_per_round:]
    return selected, remaining, finish_requested


def build_action_feedback(config: dict, actions: list, success: bool) -> str:
    """Build the action result query sent back to Dify."""
    template_key = "success_query" if success else "failure_query"
    default_template = (
        "机器人反馈:执行{actions}完成" if success else "机器人反馈:执行{actions}失败"
    )
    template = config.get(template_key, default_template)
    action_text = json.dumps(actions, ensure_ascii=False)
    return template.format(actions=action_text)
