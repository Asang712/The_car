from fastmcp import FastMCP
from typing import Annotated
from pydantic import Field
from multi_brains_pre.message.mcp_message import *
from multi_brains_pre.action_controller import ActionController
from microros_v2_bringup.robot_tools.robot_tool import GetRobotType

def register_actions(mcp:FastMCP,action_controller:ActionController):
    camera_type=GetRobotType()
    # basic_car don't have camera
    if camera_type != "basic_car":
        if camera_type == "ptz":
            @mcp.tool(
                description='''Adjust the camera perspective by pitch and yaw angles.Return the image after adjusting the camera perspective''',
                timeout=5.0)
            def AdjustCameraView(
                pitch: Annotated[int,'''Pitch adjustment in degrees. Positive=up, negative=down. Recommended step: 5-10 degrees''']=0,
                yaw: Annotated[int,'''Yaw adjustment in degrees. Positive=right, negative=left. Recommended step: 5-10 degrees''']=0
                )->dict|common_response:
                res=action_controller.AdjustCameraView(pitch,yaw)
                if res[0]:
                    return common_response(execution_result="success",reason=res[1])
                return common_response(execution_result="failed",reason=res[1])

            @mcp.tool(
                description='''Supported actions: "init_camera_pose" (restore camera to initial pose), '''
                            '''"nod" (nod the camera up and down twice), "shake" (shake the camera left and right).''',
                timeout=30.0)
            def CameraActionGroup(
                action_name: Annotated[str,'''Name of the fixed camera action group: "init_camera_pose", "nod" or "shake"''']
                )->common_response:
                res=action_controller.CameraActionGroup(action_name)
                if res[0]:
                    return common_response(execution_result="success",reason=res[1])
                return common_response(execution_result="failed",reason=res[1])

        @mcp.tool(
            description='''Capture an image from the camera and return the image save path''',
            timeout=5.0)
        def SeeWhat(
            )->dict|common_response:
            res:list=action_controller.SeeWhat()
            if res[0]:
                return {"image":res[1]}
            return  common_response(execution_result="failed",reason=f"SeeWhat error:{res[1]}")

        @mcp.tool(
            description='''Detect and return the bounding box coordinates of a target object based on its description''',
            timeout=5.0)
        def GetBbox(
            query:Annotated[str,'''Description of the target object's position and visual characteristics in the current view''']
            )->BoxLocalization|common_response:
            res:list=action_controller.GetBbox(query)
            if res[0]:
                return BoxLocalization (bbox=res[1],verify_image=action_controller.verify_image_path)
            return  common_response(execution_result="failed",reason=f"GetBbox error:{res[1]}")

        @mcp.tool(description='''Visual tracking of target objects''')
        def TargetTrack(
            x1_or_cmd: Annotated[str|int, '''If input is a string "cancel", it will stop the current tracking. If input is an integer, it will be treated as the x1 coordinate of the bounding box for target tracking'''],
            y1: Annotated[int, "y1 coordinate of the top-left corner of the target object"] = None,
            x2: Annotated[int, "x2 coordinate of the bottom-right corner of the target object"] = None,
            y2: Annotated[int, "y2 coordinate of the bottom-right corner of the target object"] = None
        )-> common_response:
            
            res=action_controller.TargetTrack(x1_or_cmd,y1,x2,y2)   
            if res[0]:
                return common_response(execution_result="success",reason="")
            else:
                return common_response(execution_result="failed",reason=res[1])

        @mcp.tool(
            description='''Follow a colored line via the camera feed. The robot drives along '''
                        '''the line of the specified color (red/green/blue/yellow) and stops '''
                        '''automatically when the line disappears.''',
            timeout=300.0)
        def FollowLine(
            color: Annotated[str, "Color of the line to follow: 'red', 'green', 'blue', or 'yellow'"]
        )-> common_response:
            res=action_controller.FollowLine(color)
            if res[0]:
                return common_response(execution_result="success,follow line finished",reason=res[1])
            return common_response(execution_result="failed",reason=res[1])

#-------------------------------------------------------------------------------------------------------------------------------------
    @mcp.tool(
        description='''Save the current location to the map mapping file''',
        timeout=10.0)
    def RecordMapLocation(
        name: Annotated[str, "Name of the location"],
        symbol: Annotated[str, "Symbol of the location"]
        )->common_response:
        res=action_controller.RecordMapLocation(name,symbol)
        if not res[0]:
            return common_response(execution_result="failed",reason=res[1])
        return common_response(execution_result="success",reason="")

    @mcp.tool(
        description='''Get the mapping relationship between all place names and their corresponding symbols''',
        timeout=10.0)
    def GetMapMapping()->common_response|str:
        res=action_controller.GetMapMapping()
        if not res[0]:
            return common_response(execution_result="failed",reason=res[1])
        return res[1]

    @mcp.tool(
        description='''Navigate to the target location via the target point symbol''',
        timeout=30.0)
    def Navigation(
        location: Annotated[str, "The symbol corresponding to the location in the map mapping file"]
        )->common_response:
        res=action_controller.Navigation(location)
        if res[0]: 
            return common_response(execution_result="success",reason="")
        return common_response(execution_result="failed",reason=res[1])

    @mcp.tool(
        description='''Move the robot chassis by a specified distance or rotate by a specified angle using odometry feedback. '''
                    '''Supported directions: forward, backward, turn_left, turn_right. '''
                    '''For linear movement (forward/backward), specify distance in meters. '''
                    '''For rotation (turn_left/turn_right), specify angle in degrees (positive value).''',
        timeout=30.0)
    def Move(
        direction: Annotated[str, "Movement direction: 'forward', 'backward', 'turn_left', or 'turn_right'"],
        distance: Annotated[float, "Linear movement distance in meters. Only used when direction is 'forward' or 'backward'. Set to 0.0 for rotation."] = 0.0,
        angle: Annotated[float, "Rotation angle in degrees. Only used when direction is 'turn_left' or 'turn_right'. Set to 0.0 for linear movement."] = 0.0
        )-> common_response:
        res = action_controller.move_by_odom(distance=distance, angle=angle, direction=direction)
        if res[0]:
            return common_response(execution_result="success", reason=res[1])
        return common_response(execution_result="failed", reason=res[1])

