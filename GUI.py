import tkinter as tk
import serial

ser = serial.Serial('COM6', 9600)  # Open serial port that Arduino is using

controller_deadband = 80

def update_motor_speeds():
    ser.write(b"GET\n")  # Send a command to request motor speeds
    response = ser.readline().decode().strip()

    # Print the received response for debugging
    print("Received response:", repr(response))

    try:
        speeds = list(map(int, response.split(',')))

        speeds = speeds[:4]  # Only use the first 4 values (in case Arduino sends more)
        speeds[1], speeds[3] = -speeds[1], -speeds[3]  # Reverse the direction of motors 2 and 4

        for i, motor_speed in enumerate(speeds):
            if i < len(arrows):
                if motor_speed > controller_deadband/1.2:
                    arrows[i]['text'] = '↑'  # Up arrow for positive speeds
                elif motor_speed < -controller_deadband/1.2:
                    arrows[i]['text'] = '↓'  # Down arrow for negative speeds
                else:
                    arrows[i]['text'] = 'O'  # Empty for zero speed
            else:
                # If there are more values than arrows, you may need to handle this situation
                pass

    except ValueError as e:
        print("Error:", e)

    root.after(1, update_motor_speeds)  # Repeat the update every 1 millisecond(s)

root = tk.Tk()
root.title("Arduino Motor Visualization")

# Create labels for each motor with initial empty values
arrows = []
for i in range(4):
    label = tk.Label(root, text='', font=('Arial', 24))
    # Update the grid positions based on the motor order
    if i == 0:
        label.grid(row=0, column=0, padx=10)
    elif i == 1:
        label.grid(row=0, column=1, padx=10)
    elif i == 2:
        label.grid(row=1, column=0, padx=10)
    elif i == 3:
        label.grid(row=1, column=1, padx=10)
    arrows.append(label)

# Start the function to update motor speeds
update_motor_speeds()

root.mainloop()