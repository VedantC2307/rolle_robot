import asyncio
import websockets
import ssl
import json
# async def test_ws():
#     uri = "wss://192.168.0.214:8888/xr-slam-client"
#     async with websockets.connect(uri, ssl=None) as websocket:
#         print("WebSocket connected")
#         async for message in websocket:
#             print(f"Received: {message}")




async def listen_to_websocket():
    """
    Connect to the WebSocket server and continuously listen for pose updates.
    Updates the global `latest_pose` variable with new data.
    """
    WS_SLAM_URL = "wss://192.168.0.214:8888/xr-slam-client"
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE

    while True:
        try:
            async with websockets.connect(WS_SLAM_URL, ssl=ssl_context) as websocket:
                print("Connected to WebSocket server.")

                async for message in websocket:
                    try:
                        # Parse incoming JSON messages
                        data = json.loads(message)
                        # Update pose from the received JSON data
                        print(data)
                        # print(f"WebSocket Message Received: {latest_pose}")
      
                    except json.JSONDecodeError:
                        print("Invalid JSON received:", message)
                    except Exception as e:
                        print(f"Error processing pose data: {e}")

        except websockets.exceptions.ConnectionClosed:
            print("SLAM WebSocket connection closed. Reconnecting...")
            await asyncio.sleep(1)
        except Exception as e:
            print(f"SLAM WebSocket error: {e}")
            await asyncio.sleep(1)

asyncio.run(listen_to_websocket())