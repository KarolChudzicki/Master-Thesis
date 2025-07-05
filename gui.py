import tkinter as tk
from tkinter import Label
import cv2 as cv
from PIL import Image, ImageTk
import json
import conveyor_belt
import threading
import time


# camera = camera.Camera()
# camera.connect(1, 1280, 720)

conveyor = conveyor_belt.conveyorBelt()


class Gui:
    def __init__(self, window, camera_instance, robot_instance):
        self.window = window
        self.window.title("Camera GUI")
        self.window.geometry("700x900")
        self.window.resizable(False, False)
        self.window.configure(bg="white")  # Helps see layout during testing
        
        self.main_window()
        
        self.is_stopping = False
        self.is_starting = False

        self.camera = camera_instance
        self.robot = robot_instance
        
    def main_window(self):
        
        # # Area for camera feed
        # self.feed_area = Label(self.window, width=1000, height=700, bg="black")
        # self.feed_area.place(x=10, y=10)

        # Calibrate button (opens a window with sliders for parameter setup)
        self.calibration_button = tk.Button(self.window, text="Calibrate", command=self.sliders_popup, height=2, width=60)
        self.calibration_button.pack(side='top', padx=10, pady=10)
        
        # Start button - starts the system
        self.start_button = tk.Button(self.window, text="Start", command=self.drop1, height=2, width=60)
        self.start_button.pack(side='top', padx=10, pady=10)
        
        # Stop button - halts the program
        self.stop_button = tk.Button(self.window, text="Stop", command=self.quit, height=2, width=60)
        self.stop_button.pack(side='top', padx=10, pady=10)

        # Home button - homes the robot
        self.home_button = tk.Button(self.window, text="Home", command=self.quit, height=2, width=60)
        self.home_button.pack(side='top', padx=10, pady=10)
        
        # Start conveyor
        self.start_conv_button = tk.Button(self.window, text="Start conveyor", command=self.start_conveyor, height=2, width=60)
        self.start_conv_button.pack(side='top', padx=10, pady=10)
        
        # Stop conveyor
        self.stop_conv_button = tk.Button(self.window, text="Stop conveyor", command=self.stop_conveyor, height=2, width=60)
        self.stop_conv_button.pack(side='top', padx=10, pady=10)
        
        # Stop conveyor
        self.speed100_button = tk.Button(self.window, text="Speed 10.0 Hz", command=self.set_speed_conveyor100, height=2, width=60)
        self.speed100_button.pack(side='top', padx=10, pady=10)
        
        # Stop conveyor
        self.speed150_button = tk.Button(self.window, text="Speed 15.0 Hz", command=self.set_speed_conveyor150, height=2, width=60)
        self.speed150_button.pack(side='top', padx=10, pady=10)
        
        # Stop conveyor
        self.speed200_button = tk.Button(self.window, text="Speed 20.0 Hz", command=self.set_speed_conveyor200, height=2, width=60)
        self.speed200_button.pack(side='top', padx=10, pady=10)
        
        # Stop conveyor
        self.speed250_button = tk.Button(self.window, text="Speed 25.0 Hz", command=self.set_speed_conveyor250, height=2, width=60)
        self.speed250_button.pack(side='top', padx=10, pady=10)
        
        # Stop conveyor
        self.speed300_button = tk.Button(self.window, text="Speed 30.0 Hz", command=self.set_speed_conveyor300, height=2, width=60)
        self.speed300_button.pack(side='top', padx=10, pady=10)
        
        self.rect_size = 90
        self.rect_padding = 26
        self.storage_indicators = []
        
        # Storage station indicators
        self.indicator_canvas = tk.Canvas(self.window, width=380, height=250, bg="gray", relief="flat", bd=0, highlightthickness=0)
        self.indicator_canvas.pack(side='top', padx=10, pady=10)
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
            
    def start(self):
        pass
    
    def home(self):
        self.robot.home()
        
    def drop1(self):
        self.robot.move_drop1(4)
        
        
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
        self.sliders_window.geometry("1500x1000")
        self.sliders_window.resizable(False, False)
        self.sliders_window.configure(bg="gray20")
        
        self.sliders = {}
        
        with open("slider_params_BottomCasing.json", "r") as file:
            self.default0 = json.load(file)
            
        
        self.default_params = [self.default0["H low"], 
                               self.default0["S low"],
                               self.default0["V low"],
                               self.default0["H up"],
                               self.default0["S up"],
                               self.default0["V up"],
                               self.default0["Erosion"],
                               self.default0["Dilation"]]
        self.maximal_params = [255,255,255,255,255,255,20,20]
        self.slider_names = ["H low", "S low", "V low", "H up", "S up", "V up", "Erosion", "Dilation"]
        
        
        self.text_box_content_number = 0
        self.sliders = []

        for i in range(len(self.default_params)):
            frame = tk.Frame(self.sliders_window, bg="gray30")
            frame.grid(row=i, column=0, padx=5, pady=5, sticky="w")

            label = tk.Label(frame, text=self.slider_names[i], width=10, bg="gray30", fg="white")
            label.pack(side="left")

            slider = tk.Scale(frame, from_=0, to=self.maximal_params[i], orient="horizontal", bg="gray20", fg="white", length=400)
            slider.set(self.default_params[i])
            slider.pack(side="left")

            self.sliders.append(slider)

        # Camera feed area (right side)
        self.feed_calib_area = Label(self.sliders_window, width=900, height=600, bg="black")
        self.feed_calib_area.grid(row=0, column=1, rowspan=len(self.default_params), padx=5, pady=5)


        # Row index after sliders (bottom row)
        controls_row = len(self.default_params)

        self.prev_button = tk.Button(self.sliders_window, text="Previous", command=lambda: self.update_part(-1), height=2, width=10)
        self.prev_button.grid(row=controls_row, column=1, sticky="w", padx=5, pady=5)

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
        self.text_box.grid(row=controls_row, column=1, padx=5)

        self.next_button = tk.Button(self.sliders_window, text="Next", command=lambda: self.update_part(1), height=2, width=10)
        self.next_button.grid(row=controls_row, column=1, sticky="e", padx=5)

        # Save and Close buttons below the previous row
        self.save_button = tk.Button(self.sliders_window, text="Save", command=self.save_params, height=2, width=10)
        self.save_button.grid(row=controls_row + 1, column=1, sticky="e", padx=5, pady=10)

        self.close_button = tk.Button(self.sliders_window, text="Close", command=self.sliders_window.destroy, height=2, width=10)
        self.close_button.grid(row=controls_row + 1, column=1, sticky="w", padx=5, pady=10)
        
        self.update_calib_feed()
    
    def update_calib_feed(self):
        if self.sliders_window.winfo_exists():
            params = [slider.get() for slider in self.sliders]
            _, edges, dilated, _, _ = self.camera.capture(width=600, part_number=self.text_box_content_number, show_or_not=False, from_json=False, params=params)
            img = Image.fromarray(dilated)
            imgtk = ImageTk.PhotoImage(image=img)
            self.feed_calib_area.imgtk = imgtk
            self.feed_calib_area.config(image=imgtk)
            
            self.window.after(30, self.update_calib_feed)  # repeat after 100 ms
    
    def save_params(self):
        for i in range(len(self.sliders)):
            params = {name: slider.get() for name, slider in zip(self.slider_names, self.sliders)}
            version = "_" + str(self.text_box_content[self.text_box_content_number])
            with open(f"slider_params{version}.json", "w") as f:
                json.dump(params, f, indent=4)
        self.camera.update_params()
        

    def update_part(self, direction):
        self.text_box.config(state="normal")
        self.text_box.delete("1.0", "end")
        next_number = self.text_box_content_number + direction
        if next_number < 0: next_number = 2
        elif next_number > 2: next_number = 0
        self.text_box_content_number = next_number

        self.text_box.insert("1.0", self.text_box_content[self.text_box_content_number], "center")
        self.text_box.config(state="disabled")
        
        self.data = [None]*3
        with open("slider_params_BottomCasing.json", "r") as file:
            self.data[0] = json.load(file)
            
        with open("slider_params_IntegratedCircuit.json", "r") as file:
            self.data[1] = json.load(file)
        
        with open("slider_params_TopCover.json", "r") as file:
            self.data[2] = json.load(file)
            
        data_from_json = [self.data[self.text_box_content_number]["H low"], 
                          self.data[self.text_box_content_number]["S low"],
                          self.data[self.text_box_content_number]["V low"],
                          self.data[self.text_box_content_number]["H up"],
                          self.data[self.text_box_content_number]["S up"],
                          self.data[self.text_box_content_number]["V up"],
                          self.data[self.text_box_content_number]["Erosion"],
                          self.data[self.text_box_content_number]["Dilation"]]
        for slider in range(len(self.sliders)):
            self.sliders[slider].set(data_from_json[slider])

    def quit(self):
        #self.stop_conveyor()
        self.window.destroy()
        
    def start_conveyor(self):
        threading.Thread(target=self._start_conveyor_worker, daemon=True).start()
    
    def set_speed_conveyor100(self):
        threading.Thread(target=self._set_speed_conveyor_worker100, daemon=True).start()
    
    def set_speed_conveyor150(self):
        threading.Thread(target=self._set_speed_conveyor_worker150, daemon=True).start()
    
    def set_speed_conveyor200(self):
        threading.Thread(target=self._set_speed_conveyor_worker200, daemon=True).start()
      
    def set_speed_conveyor250(self):
        threading.Thread(target=self._set_speed_conveyor_worker250, daemon=True).start()
    
    def set_speed_conveyor300(self):
        threading.Thread(target=self._set_speed_conveyor_worker300, daemon=True).start()
    
      
    def _start_conveyor_worker(self):
        try:
            conveyor.setDirection(1)
        except Exception as e:
            print(f"Error starting conveyor: {e}")
            
        try:
            conveyor.start()
        except Exception as e:
            print(f"Error starting conveyor: {e}")
        
    def _set_speed_conveyor_worker100(self):
        try:
            conveyor.setSpeed(100)
        except Exception as e:
            print(f"Error starting conveyor: {e}")
        
    def _set_speed_conveyor_worker150(self):
        try:
            conveyor.setSpeed(150)
        except Exception as e:
            print(f"Error starting conveyor: {e}")
    
    def _set_speed_conveyor_worker200(self):
        try:
            conveyor.setSpeed(200)
        except Exception as e:
            print(f"Error starting conveyor: {e}")
    
    def _set_speed_conveyor_worker250(self):
        try:
            conveyor.setSpeed(250)
        except Exception as e:
            print(f"Error starting conveyor: {e}")
            
    def _set_speed_conveyor_worker300(self):
        try:
            conveyor.setSpeed(300)
        except Exception as e:
            print(f"Error starting conveyor: {e}")
    
    def stop_conveyor(self):
        threading.Thread(target=self._stop_conveyor_worker, daemon=True).start()
        
    def _stop_conveyor_worker(self):
        try:
            conveyor.stop()
        except Exception as e:
            print(f"Error starting conveyor: {e}")    



