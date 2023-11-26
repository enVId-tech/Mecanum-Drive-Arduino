import pygame
from pygame.locals import *
import serial
import time

# Set your Arduino serial port and baud rate
arduino_port = "COM6"  # Change this to your Arduino port
baud_rate = 9600

# Initialize Pygame
pygame.init()

# Set up the screen
screen_width, screen_height = 800, 600
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Arduino Wheel Directions")

# Set up colors
white = (255, 255, 255)

# Load arrow images
arrow_up = pygame.image.load("arrow_up.png")  # Replace with the path to your arrow image
arrow_down = pygame.image.load("arrow_down.png")
arrow_none = pygame.image.load("arrow_none.png")

# Resize arrow images
desired_width, desired_height = 50, 50
arrow_up = pygame.transform.scale(arrow_up, (desired_width, desired_height))
arrow_down = pygame.transform.scale(arrow_down, (desired_width, desired_height))
arrow_none = pygame.transform.scale(arrow_none, (desired_width, desired_height))

# Set initial arrow positions
arrow_positions = [(100, 100), (300, 100), (100, 300), (300, 300)]

# Set the font for displaying text
font = pygame.font.Font(None, 36)

# Connect to Arduino
arduino = serial.Serial(arduino_port, baud_rate, timeout=1)
time.sleep(2)  # Allow time for the Arduino to reset

# Set the desired frame rate (frames per second)
fps = 10
clock = pygame.time.Clock()

# Time interval for fetching data (in milliseconds)
data_fetch_interval = 100
last_fetch_time = pygame.time.get_ticks()

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == QUIT:
            running = False

    current_time = pygame.time.get_ticks()

    # Fetch data from Arduino every 100 ms
    if current_time - last_fetch_time >= data_fetch_interval:
        data = arduino.readline().decode().strip().split(',')

        print(data)

        # Set background color only once per frame
        screen.fill(white)

        # Display arrows based on Arduino data
        if len(data) == 4:  # Ensure valid data is received
            for i in range(4):
                arrow_direction = int(data[i]) if data[i] else 0
                screen.blit(arrow_up if arrow_direction > 0 else
                            arrow_down if arrow_direction < 0 else arrow_none,
                            arrow_positions[i])

        # Update the display only once per frame
        pygame.display.flip()

        # Update the last fetch time
        last_fetch_time = current_time

    # Cap the frame rate
    clock.tick(fps)

# Close the serial connection and quit Pygame
arduino.close()
pygame.quit()