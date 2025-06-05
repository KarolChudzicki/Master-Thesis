import tkinter as tk
from tkinter import Label
import cv2
from PIL import Image, ImageTk
import camera
import json

class Gui:
    def __init__(self, window):
        self.window = window
        self.window.title("Camera GUI")
        self.window.geometry("1200x500")
        self.window.resizable(False, False)
        self.window.configure(bg="white")  # Helps see layout during testing
        self.camera = camera.Camera()
        

        # Area for camera feed
        self.feed_area = Label(self.window, width=640, height=480, bg="black")
        self.feed_area.place(x=10, y=10)

        # Calibrate button (opens a window with sliders for parameter setup)
        self.calibration_button = tk.Button(self.window, text="Calibrate", command=self.sliders_popup, height=2, width=60)
        self.calibration_button.place(x=700, y=10)
        
        # Start button placed below the feed
        self.start_button = tk.Button(self.window, text="Start", command=self.quit, height=2, width=60)
        self.start_button.place(x=700, y=60)
        
        # Stop button placed below the feed
        self.stop_button = tk.Button(self.window, text="Stop", command=self.quit, height=2, width=60)
        self.stop_button.place(x=700, y=110)
        
        self.rect_size = 90
        self.rect_padding = 26
        self.storage_indicators = []
        
        # Storage station indicators
        self.indicator_canvas = tk.Canvas(self.window, width=380, height=250, bg="gray", relief="flat", bd=0, highlightthickness=0)
        self.indicator_canvas.place(x=725, y=200) 
        self.indicator_canvas.create_rectangle(0,0, 430, 25, fill="white", outline="")
        
        for i in range(3):
            x0 = i * (self.rect_size + self.rect_padding) + self.rect_padding
            y0 = self.rect_padding
            x1 = x0 + self.rect_size
            y1 = y0 + self.rect_size
            self.rect = self.indicator_canvas.create_rectangle(x0, y0, x1, y1, fill="blue", outline="lightblue")
            self.storage_indicators.append(self.rect)
        
        self.text_box_content = ["BottomCasing", "IntegratedCircuit", "TopCover"]
        
        for i in range(3):
            x0 = i * (self.rect_size + self.rect_padding) + self.rect_padding
            y0 = self.rect_padding + self.rect_size
            x1 = x0 + self.rect_size
            y1 = y0 + self.rect_size
            self.rect = self.indicator_canvas.create_rectangle(x0, y0, x1, y1, fill="blue", outline="lightblue")
            self.storage_indicators.append(self.rect)
            
            x_center = i * (self.rect_size + self.rect_padding) + self.rect_padding + self.rect_size / 2
            y_position = self.rect_padding * 1.5 + self.rect_size * 2  # Adjust as needed
            self.indicator_canvas.create_text(
                x_center,
                y_position,
                text=self.text_box_content[i],
                fill="white",  # or "black", depending on canvas background
                font=("Helvetica", 10),
                anchor="n"  # Anchor the top of the text to y_position
            )
    
    def update_indicators(self, vector):
        if len(vector) == 6:
            for r in range(len(vector)):
                if vector[r] == 0:
                    self.indicator_canvas.itemconfig(self.storage_indicators[r], fill="green")
                elif vector[r] == 1:
                    self.indicator_canvas.itemconfig(self.storage_indicators[r], fill="red")
    
    def sliders_popup(self):
        self.sliders_window = tk.Toplevel(self.window)
        self.sliders_window.title("Calibration")
        self.sliders_window.geometry("600x600")
        self.sliders_window.resizable(False, False)
        self.sliders_window.configure(bg="gray20")
        
        self.sliders = {}
        self.default_params = [60,55,70,255,255,255,3,2]
        self.maximal_params = [255,255,255,255,255,255,20,20]
        self.slider_names = ["H low", "S low", "V low", "H upper", "S upper", "V upper", "Erosion", "Dilation"]
        
        
        self.text_box_content_number = 0
        self.sliders = []
        
        for i in range(len(self.default_params)):
            frame = tk.Frame(self.sliders_window, bg="gray30")
            frame.pack(fill="x", padx=10, pady=5)

            label = tk.Label(frame, text=self.slider_names[i], width=10, bg="gray30", fg="white")
            label.pack(side="left")

            slider = tk.Scale(frame, from_= 0, to=self.maximal_params[i], orient="horizontal", bg="gray20", fg="white")
            slider.set(self.default_params[i])
            slider.pack(side="right", fill="x", expand=True)
            
            self.sliders.append(slider)
        
        self.prev_button = tk.Button(self.sliders_window, text="Previous", command=lambda: self.update_part(-1), height=2, width=10)
        self.prev_button.place(x=10, y=450)
        
        self.next_button = tk.Button(self.sliders_window, text="Next", command=lambda: self.update_part(1), height=2, width=10)
        self.next_button.place(x=510, y=450)
        
        
        
        self.text_box = tk.Text(self.sliders_window, 
                   width=30, height=1, 
                   bg="#1e1e1e", fg="#f0f0f0", 
                   font=("Consolas", 16),
                   borderwidth=0, relief="flat",
                   highlightthickness=0,
                   highlightbackground="#555555",
                   highlightcolor="#3399ff",
                   wrap="word")
        
        self.text_box.tag_configure("center", justify="center")
        
        self.text_box.insert("1.0", self.text_box_content[0], "center")
        self.text_box.config(state="disabled")
        self.text_box.place(x=120, y=458)
        
        
        self.close_button = tk.Button(self.sliders_window, text="Close", command=self.sliders_window.destroy, height=2, width=10)
        self.close_button.place(x=10, y=550)
        
        self.save_button = tk.Button(self.sliders_window, text="Save", command=self.save_params, height=2, width=10)
        self.save_button.place(x=110, y=550)
    
    def save_params(self):
        for i in range(len(self.sliders)):
            params = {name: slider.get() for name, slider in zip(self.slider_names, self.sliders)}
            version = "_" + str(self.text_box_content[self.text_box_content_number])
            with open(f"slider_params{version}.json", "w") as f:
                json.dump(params, f, indent=4)
            print("Saved to file.")

    def update_part(self, direction):
        self.text_box.config(state="normal")
        self.text_box.delete("1.0", "end")
        next_number = self.text_box_content_number + direction
        if next_number < 0: next_number = 2
        elif next_number > 2: next_number = 0
        self.text_box_content_number = next_number

        self.text_box.insert("1.0", self.text_box_content[self.text_box_content_number], "center")
        self.text_box.config(state="disabled")

    def update_image(self, frame):
        frame_resized = cv2.resize(frame, (640, 480))
        img = Image.fromarray(frame_resized)
        imgtk = ImageTk.PhotoImage(image=img)
        self.feed_area.imgtk = imgtk
        self.feed_area.config(image=imgtk)

    def quit(self):
        self.window.destroy()
        

# Run the app
root = tk.Tk()
app = Gui(root)

# Optional: simulate image updates
import numpy as np
def simulate_frame():
    dummy = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    app.update_image(dummy)
    root.after(100, simulate_frame)

app.update_indicators([0,1,0,1,1,0])

simulate_frame()
root.mainloop()
