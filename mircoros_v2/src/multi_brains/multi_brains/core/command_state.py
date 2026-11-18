"""Per-command mutable state for the LLM action feedback loop."""

from dataclasses import dataclass, field


@dataclass
class CommandState:
    """State owned by one command worker thread."""

    command_id: int
    pending_feedback_messages: list[str] = field(default_factory=list)
    pending_feedback_image_paths: list[str] = field(default_factory=list)
    terminal_action_result_text: str = ""
    reset_conversation_after_response: bool = False
