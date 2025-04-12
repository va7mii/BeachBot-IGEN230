import socket

# Replace with the IP address of your ESP32
ESP32_IP = "192.168.137.62"
PORT = 23  # Match the port you used in ESP32 code

def send_command(command):
    try:
        # Create a TCP/IP socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((ESP32_IP, PORT))
        
        # Send command (must be bytes)
        s.sendall(command.encode())  # e.g., "1" or "LED ON"
        
        # Optionally read response
        #response = s.recv(1024)
        #print("ESP32 says:", response.decode())
        
        s.close()
    except Exception as e:
        print("Error:", e)

# Example usage:
while True:
    cmd = input("Send to ESP32: ")
    if cmd.lower() == "exit":
        break
    send_command(cmd)
