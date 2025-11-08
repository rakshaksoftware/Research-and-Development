import socket
import threading
import time
from datetime import datetime
from colorama import Fore, Style, init

init(autoreset=True)

DRONE_B_IP = '127.0.0.1'
DRONE_B_PORT = 5001
DRONE_A_IP = '127.0.0.1'
DRONE_A_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((DRONE_B_IP, DRONE_B_PORT))

def current_time():
    return datetime.now().strftime('%H:%M:%S')

def listen():
    while True:
        data, addr = sock.recvfrom(1024)
        print(f"{Fore.MAGENTA}[{current_time()}] Drone B received from {addr}: {data.decode()}{Style.RESET_ALL}")

def send():
    count = 0
    while True:
        message = f"Message {count} from Drone B"
        sock.sendto(message.encode(), (DRONE_A_IP, DRONE_A_PORT))
        print(f"{Fore.YELLOW}[{current_time()}] Drone B sent: {message}{Style.RESET_ALL}")
        count += 1
        time.sleep(3)

threading.Thread(target=listen, daemon=True).start()
threading.Thread(target=send, daemon=True).start()

while True:
    time.sleep(1)
