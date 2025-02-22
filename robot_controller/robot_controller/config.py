# config.py
import socket

def get_local_ip():
    try:
        # Create a dummy socket to get the IP address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))  # Google's public DNS
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address
    except Exception as e:
        return f"Error: {e}"

OPEN_API_KEY = "your_api_key_here"
IP_ADDRESS = get_local_ip()
PORT = 4000
