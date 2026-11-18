---
name: "localhost-cli"
description: "CLI for the localhost MCP server. Call tools, list resources, and get prompts."
---

# localhost CLI

## Tool Commands

### AdjustCameraView

Adjust the camera perspective by pitch and yaw angles.Return the image after adjusting the camera perspective

```bash
uv run --with fastmcp python robot_control.py call-tool AdjustCameraView --pitch <value> --yaw <value>
```

| Flag | Type | Required | Description |
|------|------|----------|-------------|
| `--pitch` | integer | no | Pitch adjustment in degrees. Positive=up, negative=down. Recommended step: 5-10 degrees |
| `--yaw` | integer | no | Yaw adjustment in degrees. Positive=right, negative=left. Recommended step: 5-10 degrees |

### SeeWhat

Capture an image from the camera and return the image save path

```bash
uv run --with fastmcp python robot_control.py call-tool SeeWhat
```

### RecordMapLocation

Save the current location to the map mapping file

```bash
uv run --with fastmcp python robot_control.py call-tool RecordMapLocation --name <value> --symbol <value>
```

| Flag | Type | Required | Description |
|------|------|----------|-------------|
| `--name` | string | yes | Name of the location |
| `--symbol` | string | yes | Symbol of the location |

### GetMapMapping

Get the mapping relationship between all place names and their corresponding symbols

```bash
uv run --with fastmcp python robot_control.py call-tool GetMapMapping
```

### Navigation

Navigate to the target location via the target point symbol

```bash
uv run --with fastmcp python robot_control.py call-tool Navigation --location <value>
```

| Flag | Type | Required | Description |
|------|------|----------|-------------|
| `--location` | string | yes | The symbol corresponding to the location in the map mapping file |

### Rotate

rotates in place by a specified angle

```bash
uv run --with fastmcp python robot_control.py call-tool Rotate --angle <value>
```

| Flag | Type | Required | Description |
|------|------|----------|-------------|
| `--angle` | number | yes | Rotation angle in degrees. Positive for counterclockwise/left, negative for clockwise/right |

## Utility Commands

```bash
uv run --with fastmcp python robot_control.py list-tools
uv run --with fastmcp python robot_control.py list-resources
uv run --with fastmcp python robot_control.py read-resource <uri>
uv run --with fastmcp python robot_control.py list-prompts
uv run --with fastmcp python robot_control.py get-prompt <name> [key=value ...]
```
