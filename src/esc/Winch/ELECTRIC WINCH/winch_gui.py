#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NADIA ELECTRIC WINCH CONTROL GUI
=================================
Python Tkinter GUI for controlling the electric winch via serial.
Compatible with STM32F411CEU6 firmware.

Requirements:
- Python 3.x
- pyserial (pip install pyserial)
- tkinter (usually included with Python on Windows)
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import queue
import time
from datetime import datetime
from typing import Optional

from serial_port_utils import find_best_port


class WinchGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("NADIA ELECTRIC WINCH CONTROL")
        self.root.geometry("550x700")
        self.root.resizable(True, True)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Serial connection
        self.serial_port: Optional[serial.Serial] = None
        self.serial_thread: Optional[threading.Thread] = None
        self.stop_thread = threading.Event()
        self.serial_queue = queue.Queue()
        
        # State variables
        self.connected = False
        self.current_state = "STOPPED"
        self.top_limit = False
        self.bottom_limit = False
        self.current_pwm = 0
        
        # Build UI
        self.build_ui()
        self.refresh_ports()
        self.process_serial_queue()
        
    def build_ui(self):
        # Main container with padding
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # ============================================================
        # TITLE
        # ============================================================
        title_label = ttk.Label(
            main_frame, 
            text="NADIA ELECTRIC WINCH CONTROL",
            font=("Segoe UI", 16, "bold")
        )
        title_label.pack(pady=(0, 15))
        
        # ============================================================
        # CONNECTION FRAME
        # ============================================================
        conn_frame = ttk.LabelFrame(main_frame, text="Connection", padding="10")
        conn_frame.pack(fill=tk.X, pady=(0, 10))
        
        # COM Port selection
        port_frame = ttk.Frame(conn_frame)
        port_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_frame, text="COM Port:", width=10).pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=15, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=(5, 5))
        
        ttk.Button(port_frame, text="REFRESH PORT", command=self.refresh_ports, width=12).pack(side=tk.LEFT, padx=5)
        
        # Baud rate
        baud_frame = ttk.Frame(conn_frame)
        baud_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(baud_frame, text="Baud:", width=10).pack(side=tk.LEFT)
        self.baud_var = tk.StringVar(value="115200")
        ttk.Combobox(baud_frame, textvariable=self.baud_var, values=["9600", "19200", "38400", "57600", "115200"], width=10, state="readonly").pack(side=tk.LEFT, padx=(5, 5))
        
        # Connect/Disconnect buttons
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        self.connect_btn = ttk.Button(btn_frame, text="CONNECT", command=self.connect, width=15)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = ttk.Button(btn_frame, text="DISCONNECT", command=self.disconnect, width=15, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        # Connection status
        self.conn_status_var = tk.StringVar(value="DISCONNECTED")
        self.conn_status_label = ttk.Label(conn_frame, textvariable=self.conn_status_var, font=("Segoe UI", 10, "bold"), foreground="red")
        self.conn_status_label.pack(pady=5)
        
        # ============================================================
        # CONTROL BUTTONS FRAME
        # ============================================================
        ctrl_frame = ttk.LabelFrame(main_frame, text="Winch Control", padding="10")
        ctrl_frame.pack(fill=tk.X, pady=(0, 10))
        
        # UP Button
        self.up_btn = ttk.Button(
            ctrl_frame, 
            text="▲\nUP", 
            command=self.cmd_up,
            width=15,
            state=tk.DISABLED
        )
        self.up_btn.pack(pady=5, ipady=15)
        
        # STOP Button (prominent - safety)
        self.stop_btn = ttk.Button(
            ctrl_frame, 
            text="■\nSTOP", 
            command=self.cmd_stop,
            width=15,
            state=tk.DISABLED
        )
        self.stop_btn.pack(pady=5, ipady=15)
        # Style the stop button to be more prominent
        style = ttk.Style()
        style.configure("Stop.TButton", foreground="red", font=("Segoe UI", 12, "bold"))
        self.stop_btn.configure(style="Stop.TButton")
        
        # DOWN Button
        self.down_btn = ttk.Button(
            ctrl_frame, 
            text="▼\nDOWN", 
            command=self.cmd_down,
            width=15,
            state=tk.DISABLED
        )
        self.down_btn.pack(pady=5, ipady=15)
        
        # ============================================================
        # STATUS FRAME
        # ============================================================
        status_frame = ttk.LabelFrame(main_frame, text="Status", padding="10")
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Status grid
        status_grid = ttk.Frame(status_frame)
        status_grid.pack(fill=tk.X)
        
        # Row 0: Winch State
        ttk.Label(status_grid, text="WINCH:", font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky=tk.W, padx=5, pady=3)
        self.state_var = tk.StringVar(value="STOPPED")
        self.state_label = ttk.Label(status_grid, textvariable=self.state_var, font=("Segoe UI", 9), foreground="blue")
        self.state_label.grid(row=0, column=1, sticky=tk.W, padx=5, pady=3)
        
        # Row 1: LS ATAS
        ttk.Label(status_grid, text="LS ATAS:", font=("Segoe UI", 9, "bold")).grid(row=1, column=0, sticky=tk.W, padx=5, pady=3)
        self.top_var = tk.StringVar(value="NOT ACTIVE")
        self.top_label = ttk.Label(status_grid, textvariable=self.top_var, font=("Segoe UI", 9))
        self.top_label.grid(row=1, column=1, sticky=tk.W, padx=5, pady=3)
        
        # Row 2: LS BAWAH
        ttk.Label(status_grid, text="LS BAWAH:", font=("Segoe UI", 9, "bold")).grid(row=2, column=0, sticky=tk.W, padx=5, pady=3)
        self.bottom_var = tk.StringVar(value="NOT ACTIVE")
        self.bottom_label = ttk.Label(status_grid, textvariable=self.bottom_var, font=("Segoe UI", 9))
        self.bottom_label.grid(row=2, column=1, sticky=tk.W, padx=5, pady=3)
        
        # Row 3: PWM
        ttk.Label(status_grid, text="PWM:", font=("Segoe UI", 9, "bold")).grid(row=3, column=0, sticky=tk.W, padx=5, pady=3)
        self.pwm_var = tk.StringVar(value="0 %")
        self.pwm_label = ttk.Label(status_grid, textvariable=self.pwm_var, font=("Segoe UI", 9))
        self.pwm_label.grid(row=3, column=1, sticky=tk.W, padx=5, pady=3)
        
        # Row 4: SERVO
        ttk.Label(status_grid, text="SERVO:", font=("Segoe UI", 9, "bold")).grid(row=4, column=0, sticky=tk.W, padx=5, pady=3)
        self.servo_var = tk.StringVar(value="HOME")
        self.servo_label = ttk.Label(status_grid, textvariable=self.servo_var, font=("Segoe UI", 9))
        self.servo_label.grid(row=4, column=1, sticky=tk.W, padx=5, pady=3)
        
        # ============================================================
        # LOG/CONSOLE FRAME
        # ============================================================
        log_frame = ttk.LabelFrame(main_frame, text="Console Log", padding="10")
        log_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))
        
        self.log_text = scrolledtext.ScrolledText(
            log_frame, 
            height=12, 
            width=60,
            font=("Consolas", 9),
            state=tk.DISABLED,
            wrap=tk.WORD
        )
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
        # Configure tags for colored output
        self.log_text.tag_config("timestamp", foreground="gray")
        self.log_text.tag_config("sent", foreground="blue")
        self.log_text.tag_config("received", foreground="green")
        self.log_text.tag_config("warning", foreground="orange")
        self.log_text.tag_config("error", foreground="red")
        self.log_text.tag_config("info", foreground="black")
        self.log_text.tag_config("limit", foreground="purple", font=("Consolas", 9, "bold"))
        self.log_text.tag_config("auto", foreground="brown", font=("Consolas", 9, "bold"))
        
        # Clear log button
        ttk.Button(log_frame, text="CLEAR LOG", command=self.clear_log).pack(anchor=tk.E, pady=(5, 0))
        
    def refresh_ports(self):
        """Refresh available COM ports."""
        ports = list(serial.tools.list_ports.comports())
        detected_ports = [port.device for port in ports]
        self.port_combo['values'] = detected_ports

        if detected_ports:
            preferred = find_best_port(ports)
            if preferred:
                self.port_var.set(preferred)
            else:
                self.port_var.set(detected_ports[0])
            self.port_combo.current(detected_ports.index(self.port_var.get()) if self.port_var.get() in detected_ports else 0)
            self.log("INFO", f"Found {len(detected_ports)} port(s): {', '.join(detected_ports)}")
        else:
            self.port_var.set("")
            self.log("WARNING", "No COM ports found")
    
    def connect(self):
        """Connect to selected serial port."""
        port = self.port_var.get()
        if not port:
            messagebox.showwarning("Warning", "Please select a COM port")
            return
            
        try:
            baud = int(self.baud_var.get())
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.5)  # Wait for connection to stabilize
            
            # Start reader thread
            self.stop_thread.clear()
            self.serial_thread = threading.Thread(target=self.serial_reader, daemon=True)
            self.serial_thread.start()
            
            self.connected = True
            self.update_connection_ui(True)
            self.log("INFO", f"Connected to {port} @ {baud}")
            
            # Send STATUS to get initial state
            self.send_command("STATUS")
            
        except serial.SerialException as e:
            messagebox.showerror("Connection Error", f"Failed to connect:\n{e}")
            self.log("ERROR", f"Connection failed: {e}")
    
    def disconnect(self):
        """Disconnect from serial port."""
        if self.serial_port and self.serial_port.is_open:
            # Send STOP before closing for safety
            try:
                self.serial_port.write(b"STOP\n")
                time.sleep(0.1)
            except:
                pass
            
        self.stop_thread.set()
        if self.serial_thread:
            self.serial_thread.join(timeout=1.0)
            
        if self.serial_port:
            try:
                self.serial_port.close()
            except:
                pass
            self.serial_port = None
            
        self.connected = False
        self.update_connection_ui(False)
        self.log("INFO", "Disconnected")
    
    def update_connection_ui(self, connected: bool):
        """Update UI elements based on connection state."""
        if connected:
            self.conn_status_var.set("CONNECTED")
            self.conn_status_label.configure(foreground="green")
            self.connect_btn.configure(state=tk.DISABLED)
            self.disconnect_btn.configure(state=tk.NORMAL)
            self.up_btn.configure(state=tk.NORMAL)
            self.down_btn.configure(state=tk.NORMAL)
            self.stop_btn.configure(state=tk.NORMAL)
            self.port_combo.configure(state=tk.DISABLED)
        else:
            self.conn_status_var.set("DISCONNECTED")
            self.conn_status_label.configure(foreground="red")
            self.connect_btn.configure(state=tk.NORMAL)
            self.disconnect_btn.configure(state=tk.DISABLED)
            self.up_btn.configure(state=tk.DISABLED)
            self.down_btn.configure(state=tk.DISABLED)
            self.stop_btn.configure(state=tk.DISABLED)
            self.port_combo.configure(state=tk.NORMAL)
    
    def serial_reader(self):
        """Background thread to read serial data."""
        buffer = ""
        while not self.stop_thread.is_set() and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    data = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Process complete lines
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip('\r')
                        if line:
                            self.serial_queue.put(("rx", line))
                else:
                    time.sleep(0.01)
            except Exception as e:
                if not self.stop_thread.is_set():
                    self.serial_queue.put(("error", str(e)))
                break
    
    def process_serial_queue(self):
        """Process serial messages in main thread (Tkinter safe)."""
        try:
            while True:
                msg_type, msg = self.serial_queue.get_nowait()
                self.handle_serial_message(msg_type, msg)
        except queue.Empty:
            pass
        
        # Schedule next check
        self.root.after(50, self.process_serial_queue)
    
    def handle_serial_message(self, msg_type: str, msg: str):
        """Handle incoming serial message."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        if msg_type == "rx":
            self.log("RECEIVED", f"{msg}")
            self.parse_status(msg)
        elif msg_type == "error":
            self.log("ERROR", f"Serial error: {msg}")
            self.disconnect()
    
    def parse_status(self, msg: str):
        """Parse status messages from firmware."""
        msg_upper = msg.upper()
        
        # Parse STATE:xxx
        if msg_upper.startswith("STATE:"):
            state = msg[6:].strip()
            self.current_state = state
            self.state_var.set(state)
            self.update_state_colors()
            
        # Parse TOP:0/1
        elif msg_upper.startswith("TOP:"):
            val = msg[4:].strip()
            self.top_limit = (val == "1")
            self.top_var.set("ACTIVE" if self.top_limit else "NOT ACTIVE")
            self.top_label.configure(foreground="red" if self.top_limit else "black")
            if self.top_limit:
                self.log("LIMIT", "TOP LIMIT TRIGGERED")
                
        # Parse BOTTOM:0/1
        elif msg_upper.startswith("BOTTOM:"):
            val = msg[7:].strip()
            self.bottom_limit = (val == "1")
            self.bottom_var.set("ACTIVE" if self.bottom_limit else "NOT ACTIVE")
            self.bottom_label.configure(foreground="red" if self.bottom_limit else "black")
            if self.bottom_limit:
                self.log("LIMIT", "BOTTOM LIMIT TRIGGERED")
                
        # Parse PWM:xxx
        elif msg_upper.startswith("PWM:"):
            val = msg[4:].strip()
            try:
                self.current_pwm = int(val)
                self.pwm_var.set(f"{self.current_pwm} %")
            except:
                pass
                
        # Parse SERVO:HOME / SERVO:195
        elif msg_upper.startswith("SERVO:"):
            val = msg[6:].strip()
            self.servo_var.set(val)
            if "195" in val:
                self.servo_label.configure(foreground="purple")
            else:
                self.servo_label.configure(foreground="black")
                
        # Special messages
        elif "AUTO RETURN" in msg_upper or "AUTO RETURNING" in msg_upper:
            self.log("AUTO", msg)
        elif "LIMIT" in msg_upper:
            self.log("LIMIT", msg)
        elif "STOP" in msg_upper and ("EMERGENCY" in msg_upper or "MANUAL" in msg_upper):
            self.log("WARNING", msg)
    
    def update_state_colors(self):
        """Update state label color based on current state."""
        state = self.current_state.upper()
        if state == "STOPPED":
            self.state_label.configure(foreground="blue")
        elif state == "UP":
            self.state_label.configure(foreground="green")
        elif state == "DOWN":
            self.state_label.configure(foreground="orange")
        elif "AUTO" in state:
            self.state_label.configure(foreground="brown")
        elif "STOPPING" in state:
            self.state_label.configure(foreground="red")
        else:
            self.state_label.configure(foreground="black")
    
    def send_command(self, cmd: str):
        """Send command to firmware."""
        if not self.connected or not self.serial_port or not self.serial_port.is_open:
            return False
            
        try:
            self.serial_port.write((cmd + "\n").encode())
            self.log("SENT", cmd)
            return True
        except Exception as e:
            self.log("ERROR", f"Send failed: {e}")
            return False
    
    def cmd_up(self):
        """Send UP command."""
        if self.connected:
            self.send_command("UP")
    
    def cmd_down(self):
        """Send DOWN command."""
        if self.connected:
            self.send_command("DOWN")
    
    def cmd_stop(self):
        """Send STOP command (emergency stop)."""
        if self.connected:
            self.send_command("STOP")
    
    def log(self, level: str, message: str):
        """Add message to log with timestamp."""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.configure(state=tk.NORMAL)
        
        # Tag based on level
        tag = "info"
        if level == "SENT":
            tag = "sent"
        elif level == "RECEIVED":
            tag = "received"
        elif level == "WARNING":
            tag = "warning"
        elif level == "ERROR":
            tag = "error"
        elif level == "LIMIT":
            tag = "limit"
        elif level == "AUTO":
            tag = "auto"
            
        self.log_text.insert(tk.END, f"{timestamp} ", "timestamp")
        self.log_text.insert(tk.END, f"{message}\n", tag)
        
        self.log_text.configure(state=tk.DISABLED)
        self.log_text.see(tk.END)
    
    def clear_log(self):
        """Clear the log text."""
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.delete(1.0, tk.END)
        self.log_text.configure(state=tk.DISABLED)
    
    def on_closing(self):
        """Handle window close event."""
        if self.connected:
            # Send STOP for safety
            self.send_command("STOP")
            time.sleep(0.1)
        self.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    
    # Set Windows DPI awareness for better scaling
    try:
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except:
        pass
    
    # Configure style
    style = ttk.Style()
    style.theme_use('clam' if 'clam' in style.theme_names() else 'default')
    
    app = WinchGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()