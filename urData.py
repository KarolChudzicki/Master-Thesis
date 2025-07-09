import threading
import struct
import socket
import logging
import time

logging.basicConfig(level=logging.INFO)

class URData:
    def __init__(self, host='192.38.66.227', port=30003):
        self.host = host
        self.port = port
        self.latest_pose = [0.0] * 6
        self.running = True

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.s.connect((host, port))
            logging.info(f"Successfully connected to {self.host}:{self.port}")
        except socket.error as e:
            logging.error(f"Failed to connect to {self.host}:{self.port} - {e}")
            raise

        self.s.settimeout(0.1)
        self.lock = threading.Lock()

        # Event to signal new pose arrival
        self.pose_updated_event = threading.Event()

        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()
        logging.info("Pose thread started")

    def recv_all(self, n):
        data = b''
        while len(data) < n:
            try:
                packet = self.s.recv(n - len(data))
                if not packet:
                    return None
                data += packet
            except socket.timeout:
                return None
        return data

    def _receive_loop(self):
        START_BYTE = 72
        END_BYTE = 78
        OFFSET = 12
        DECIMALS = 5

        while self.running:
            try:
                response = self.recv_all(812)
                if response and len(response) >= 812:
                    pose = []
                    for i in range(START_BYTE, END_BYTE):
                        idx = OFFSET + 8 * i
                        val = struct.unpack('>d', response[idx:idx + 8])[0]
                        pose.append(round(val, DECIMALS))
                    with self.lock:
                        self.latest_pose = pose
                    # Signal new pose available
                    self.pose_updated_event.set()
                else:
                    logging.warning("Incomplete data or no data in receiver")
                    self.latest_pose = [None, None, None, None, None, None]
            except Exception as e:
                logging.warning(f"Exception in receive loop: {e}")
                self.running = False
                self.reconnect()
        
        
    
    def reconnect(self):
        while not self.running:
            try:
                logging.info("Attempting to reconnect...")
                self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.s.connect((self.host, self.port))
                self.s.settimeout(0.1)
                self.running = True
                self.thread = threading.Thread(target=self._receive_loop, daemon=True)
                self.thread.start()
                logging.info("Reconnected successfully.")
            except Exception as e:
                logging.error(f"Reconnect failed: {e}")
                time.sleep(5)  # wait and try again

    # Add wait_for_update flag and timeout to get_pose
    def get_pose(self, wait_for_update=False, timeout=0.2):
        return list(self.latest_pose)

    def stop(self):
        self.running = False
        self.thread.join()
        self.s.close()