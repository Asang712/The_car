PROMPT='''# Core Task: Visual Localization (Bounding Box Detection)
You are required to locate the target object in the image based on the input image and target description, and output its minimum enclosing bounding box.

## Image Information
- Resolution: 640 × 480 (Width × Height)
- Coordinate System: The top-left corner is the origin (0, 0)
- The x-axis increases to the right, and the y-axis increases downward
- Coordinate Unit: Pixel (integer)
- Auxiliary grid lines are overlaid on the image with a grid size of 30 pixels

## Task Steps
1. Locate the object matching the **target description** in the image.
2. Determine the minimum enclosing rectangle of the object (must fully cover the object).
3. Extract boundary coordinates:
   - Leftmost x
   - Rightmost x
   - Topmost y
   - Bottommost y

4. Construct the bounding box:
   - (x1, y1) = (leftmost x, topmost y)
   - (x2, y2) = (rightmost x, bottommost y)

## Constraints & Requirements
- The bounding box must fully enclose the target object (no part of the object can be cropped).
- Keep the box as close to the object boundary as possible while ensuring full enclosure.
- Avoid including excessive background area.
- Coordinates must be within the image range:
  - 0 ≤ x ≤ 639
  - 0 ≤ y ≤ 479
- All coordinates must be integers.

## Special Cases
- If the target object does not exist in the image, output: {"found": false}
- If multiple qualified targets exist, only output the most primary or most prominent one.

## Output Format (Strict JSON Only, No Extra Explanations)
Output ONLY a JSON object with exactly 5 fields:
{"found": true, "x1": <int>, "y1": <int>, "x2": <int>, "y2": <int>}
Example: {"found": true, "x1": 435, "y1": 248, "x2": 617, "y2": 384}
Do NOT use arrays, nested objects, or add any other fields.'''


import base64
import json
import os
os.environ["LITELLM_LOCAL_MODEL_COST_MAP"] = "True"
import litellm
litellm.suppress_debug_info = True
from litellm import completion
from dataclasses import dataclass, field

@dataclass
class Visual_Result:
    found: bool = False
    error: bool = False
    x1: int = 0
    y1: int = 0
    x2: int = 0
    y2: int = 0
    error_message: str = ""


class RobotVisualPositioning:
    def __init__(self, api_key: str, base_url: str, model: str ):
        self.api_key = api_key
        self.base_url = base_url
        self.model = f"openai/{model}"

    def BoundingBoxDetection(self, image_path: str, target_description: str) -> Visual_Result:
        '''get bounding box of the target object in the image'''
        try:
            with open(image_path, "rb") as f:
                image_base64 = base64.b64encode(f.read()).decode("utf-8")
        except FileNotFoundError:
            return Visual_Result(error=True, error_message=f"Image not found: {image_path}")

        try:
            response = completion(
                model=self.model,
                messages=[
                    {"role": "system", "content": PROMPT},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": f"Target: {target_description}"},
                            {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_base64}"}},
                        ],
                    },
                ],
                api_base=self.base_url,
                api_key=self.api_key,
            )
        except litellm.AuthenticationError as e:
            return Visual_Result(error=True, error_message=f"Bad API key: {e}")
        except litellm.RateLimitError as e:
            return Visual_Result(error=True, error_message=f"Rate limited: {e}")
        except Exception as e:
            return Visual_Result(error=True, error_message=f"API error: {e}")

        try:
            result_text = response.choices[0].message.content.strip()
            if result_text.startswith("```"):
                result_text = result_text.split("```")[1]
                if result_text.startswith("json"):
                    result_text = result_text[4:]
            data = json.loads(result_text)
            if data.get("found"):
                return Visual_Result(found=True, x1=data["x1"], y1=data["y1"], x2=data["x2"], y2=data["y2"])
            else:
                return Visual_Result(found=False, error_message="Target not found in image")
        except (json.JSONDecodeError, KeyError) as e:
            return Visual_Result(error=True, error_message=f"The visual model returned an incorrect JSON format.Parse error: {e}, raw: {result_text}")

    def test_connection(self) -> list[bool, str]:
        '''Test if the model connection is successful'''
        try:
            response = completion(
                model=self.model,
                messages=[{"role": "user", "content": "hi"}],
                api_base=self.base_url,
                api_key=self.api_key,
                max_tokens=5,
            )
            return [True, "Connection successful"]
        except litellm.AuthenticationError:
            return [False, "API key invalid,Please check if your API key is working correctly?"]
        except litellm.RateLimitError:
            return [False, "Rate limit exceeded,Please try again later."]
        except Exception as e:
            return [False, f"Connection failed: {e}"]
        return [False, "A location error occurred while accessing the visual model. Please check your API key and try again later."]
