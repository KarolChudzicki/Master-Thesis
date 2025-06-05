import tkinter as tk
from tkinter import Label
import cv2
from PIL import Image, ImageTk
import camera

class Gui:
    def __init__(self, window):
        self.window = window
        self.window.title("Camera GUI")
        self.window.geometry("1200x800")
        self.window.resizable(False, False)
        self.window.configure(bg="gray20")  # Helps see layout during testing
        self.camera = camera.Camera()

        # Area for camera feed
        self.feed_area = Label(window, width=640, height=480, bg="black")
        self.feed_area.place(x=10, y=10)

        # Calibrate button placed below the feed
        self.calibration_button = tk.Button(window, text="Calibrate", command=self.sliders, height=2, width=60)
        self.calibration_button.place(x=700, y=10)
        
        # Start button placed below the feed
        self.quit_button = tk.Button(window, text="Start", command=self.quit, height=2, width=60)
        self.quit_button.place(x=700, y=60)
        
        # Stop button placed below the feed
        self.quit_button = tk.Button(window, text="Stop", command=self.quit, height=2, width=60)
        self.quit_button.place(x=700, y=110)
        

    def update_image(self, frame):
        frame_resized = cv2.resize(frame, (640, 480))
        img = Image.fromarray(frame_resized)
        imgtk = ImageTk.PhotoImage(image=img)
        self.feed_area.imgtk = imgtk
        self.feed_area.config(image=imgtk)

    def quit(self):
        self.window.destroy()
        
    def sliders(self):
        self.camera.initSlider()

# Run the app
root = tk.Tk()
app = Gui(root)

# Optional: simulate image updates
import numpy as np
def simulate_frame():
    dummy = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    app.update_image(dummy)
    root.after(100, simulate_frame)

simulate_frame()
root.mainloop()
