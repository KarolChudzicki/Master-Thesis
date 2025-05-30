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

        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

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
                if response and len(response) >= 636:
                    pose = []
                    for i in range(START_BYTE, END_BYTE):
                        idx = OFFSET + 8 * i
                        val = struct.unpack('>d', response[idx:idx + 8])[0]
                        pose.append(round(val, DECIMALS))
                    with self.lock:
                        self.latest_pose = pose
                else:
                    logging.warning("Incomplete data or no data")
            except Exception as e:
                logging.warning(f"Exception in receive loop: {e}")


    def get_pose(self):
        with self.lock:
            return list(self.latest_pose)

    def stop(self):
        self.running = False
        self.thread.join()
        self.s.close()