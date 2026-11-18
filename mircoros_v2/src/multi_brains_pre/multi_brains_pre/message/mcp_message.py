'''this file is defined the mcp message'''
from dataclasses import dataclass

@dataclass
class common_response:
    '''simple response to LLM'''
    execution_result: str
    reason: str

@dataclass
class BoxLocalization:
    bbox: str
    verify_image : str