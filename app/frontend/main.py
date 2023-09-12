import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time

# Function to update the sine wave chart
def update_chart():
    # Get frequency, amplitude, and max data points from the text boxes
    try:
        frequency = float(freq_entry.get())
    except ValueError:
        frequency = default_frequency
        freq_entry.delete(0, tk.END)
        freq_entry.insert(0, str(default_frequency))
    
    try:
        amplitude = float(amp_entry.get())
    except ValueError:
        amplitude = default_amplitude
        amp_entry.delete(0, tk.END)
        amp_entry.insert(0, str(default_amplitude))
    
    try:
        max_data_points = int(max_data_entry.get())
    except ValueError:
        max_data_points = default_max_data
        max_data_entry.delete(0, tk.END)
        max_data_entry.insert(0, str(default_max_data))
    
    x_data.append(x_data[-1] + 0.1)
    y_data.append(amplitude * np.sin(2 * np.pi * frequency * x_data[-1]))
    
    if len(x_data) > max_data_points:  # Keep a maximum number of data points
        x_data.pop(0)
        y_data.pop(0)
    
    # Clear the previous plot and update with new data
    ax.clear()
    ax.plot(x_data, y_data)
    
    # Set chart title and labels
    ax.set_title("Live Sine Wave Chart")
    ax.set_xlabel("Time")
    ax.set_ylabel("Amplitude")
    
    # Update the canvas
    canvas.draw()
    
    # Schedule the next update
    root.after(100, update_chart)

# Create a Tkinter window
root = tk.Tk()
root.title("Live Sine Wave Chart")

# Create a frame to hold the Matplotlib chart
frame = ttk.Frame(root)
frame.pack(padx=10, pady=10)

# Create Matplotlib figure and axis
fig, ax = plt.subplots(figsize=(6, 3))
ax.grid()

# Initialize data
x_data = [0]
y_data = [0]

# Create a Matplotlib canvas
canvas = FigureCanvasTkAgg(fig, master=frame)
canvas.get_tk_widget().pack()

# Default values
default_frequency = 1.0
default_amplitude = 1.0
default_max_data = 100

# Frequency entry with default value
freq_label = ttk.Label(root, text="Frequency:")
freq_label.pack()
freq_entry = ttk.Entry(root)
freq_entry.pack()
freq_entry.insert(0, str(default_frequency))

# Amplitude entry with default value
amp_label = ttk.Label(root, text="Amplitude:")
amp_label.pack()
amp_entry = ttk.Entry(root)
amp_entry.pack()
amp_entry.insert(0, str(default_amplitude))

# Max data points entry with default value
max_data_label = ttk.Label(root, text="Max Data Points:")
max_data_label.pack()
max_data_entry = ttk.Entry(root)
max_data_entry.pack()
max_data_entry.insert(0, str(default_max_data))

# Start updating the chart
root.after(100, update_chart)

# Run the Tkinter main loop
root.mainloop()
