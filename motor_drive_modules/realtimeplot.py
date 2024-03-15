import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

# Simulated function to get motor encoder reading
def get_simulated_encoder_reading():
    # This function simulates encoder reading; replace it with bot.get_motor_encoder()[0]
    return np.random.randint(0, 1000)

# Function to update the plot
def update(frame):
    global prev_reading
    current_reading = get_simulated_encoder_reading()
    velocity = current_reading - prev_reading  # Calculate velocity
    prev_reading = current_reading
    
    times.append(time.time() - start_time)  # Update time
    velocities.append(velocity)  # Update velocity
    
    ax.clear()
    ax.plot(times, velocities)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    plt.title('Wheel Velocity Over Time')

# Initial setup
prev_reading = get_simulated_encoder_reading()
times = [0]  # Store times
velocities = [0]  # Store velocities
start_time = time.time()

fig, ax = plt.subplots()
ani = FuncAnimation(fig, update, interval=100)  # Update every 100 ms

plt.show()
