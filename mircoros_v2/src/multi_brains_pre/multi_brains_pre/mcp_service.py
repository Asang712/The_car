import asyncio
from fastmcp import FastMCP
from .action_controller import ROSContext
import signal
import sys
from .mcp_tools.mcp_action import register_actions
def signal_handler(signum, frame):
    '''Handle system signals for graceful shutdown'''
    ROSContext.shutdown()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
def main(args=None):
    try:
        mcp_service = FastMCP(name="ROS-MCP-Service")
        action_controller = ROSContext().create_node()
        ROSContext.start_spin()
        #--------------------------------------------MCP TOOLS---------------------------------------------------------
        register_actions(mcp=mcp_service,action_controller=action_controller)    
        mcp_service.run(host="0.0.0.0", port=8000,transport="streamable-http")

    except KeyboardInterrupt:
        print("Service interrupted by user.")
    finally:
        ROSContext.shutdown()

if __name__ == "__main__":
    main()
        
        