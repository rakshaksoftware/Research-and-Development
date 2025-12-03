import socket
import threading
import time
import sys
from datetime import datetime
import tkinter as tk
from tkinter.scrolledtext import ScrolledText

class DroneGUI:
    def __init__(self, root, local_ip, local_port, remote_ip, remote_port, drone_name):
        self.root = root
        self.root.title(drone_name)
        self.local_ip = local_ip
        self.local_port = local_port
        self.remote_ip = remote_ip
        self.remote_port = remote_port
        self.drone_name = drone_name
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.local_ip, self.local_port))
        self.sock.settimeout(0.5)
        
        self.setup_ui()
        
        self.running = True
        self.count = 0
        
        threading.Thread(target=self.listen, daemon=True).start()
        threading.Thread(target=self.send_periodically, daemon=True).start()
        
        self.update_gui()
    
    def setup_ui(self):
        self.log = ScrolledText(self.root, state='disabled', width=60, height=20, font=("Consolas", 10))
        self.log.pack(padx=10, pady=(10, 0))
        
        # Frame for input and send button
        input_frame = tk.Frame(self.root)
        input_frame.pack(padx=10, pady=10, fill=tk.X)
        
        self.input_var = tk.StringVar()
        self.input_entry = tk.Entry(input_frame, textvariable=self.input_var, font=("Consolas", 12))
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.input_entry.bind('<Return>', self.manual_send)  # Enter key sends message
        
        self.send_button = tk.Button(input_frame, text="Send", command=self.manual_send)
        self.send_button.pack(side=tk.LEFT, padx=(5,0))
    
    def log_message(self, message, tag=None):
        self.log.config(state='normal')
        self.log.insert(tk.END, message + '\n', tag)
        self.log.see(tk.END)
        self.log.config(state='disabled')
    
    def listen(self):
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                timestamp = datetime.now().strftime('%H:%M:%S')
                msg = f"[{timestamp}] {self.drone_name} received from {addr}: {data.decode()}"
                self.log_message(msg, 'receive')
            except socket.timeout:
                continue
    
    def send_periodically(self):
        time.sleep(2)
        while self.running:
            # message = f"Message {self.count} from {self.drone_name}"
            # self.sock.sendto(message.encode(), (self.remote_ip, self.remote_port))
            # timestamp = datetime.now().strftime('%H:%M:%S')
            # msg = f"[{timestamp}] {self.drone_name} sent: {message}"
            # self.log_message(msg, 'send')
            # self.count += 1
            time.sleep(5)  # slower periodic sending to reduce clutter
    
    def manual_send(self, event=None):
        message = self.input_var.get().strip()
        if message:
            self.sock.sendto(message.encode(), (self.remote_ip, self.remote_port))
            timestamp = datetime.now().strftime('%H:%M:%S')
            msg = f"[{timestamp}] {self.drone_name} sent (manual): {message}"
            self.log_message(msg, 'send_manual')
            self.input_var.set("")  # Clear input
    
    def update_gui(self):
        self.log.tag_config('send', foreground='blue')
        self.log.tag_config('receive', foreground='green')
        self.log.tag_config('send_manual', foreground='red')
        self.root.after(100, self.update_gui)
    
    def stop(self):
        self.running = False
        self.sock.close()

if __name__ == "__main__":
    if len(sys.argv) != 2 or sys.argv[1] not in ('A', 'B'):
        print("Usage: python drone_gui.py A|B")
        sys.exit(1)
    
    drone = sys.argv[1]
    
    if drone == 'A':
        LOCAL_IP = '10.61.214.215'
        LOCAL_PORT = 5001
        REMOTE_IP = '10.61.126.200'
        REMOTE_PORT = 5000
        DRONE_NAME = "Drone A"
    elif drone == 'B':
        LOCAL_IP = '127.0.0.1'
        LOCAL_PORT = 5001
        REMOTE_IP = '127.0.0.1'
        REMOTE_PORT = 5000
        DRONE_NAME = "Drone B"
    
    root = tk.Tk()
    app = DroneGUI(root, LOCAL_IP, LOCAL_PORT, REMOTE_IP, REMOTE_PORT, DRONE_NAME)
    
    def on_closing():
        app.stop()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
