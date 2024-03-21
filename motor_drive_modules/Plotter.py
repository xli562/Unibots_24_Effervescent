import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


class Plotter:
    def __init__(self):
        pass


    def test_plot(self):
        """ Plots a sine wave to test plotting functionalities. """ 

        # Generate 100 x values from 0 to 2*pi
        x = np.linspace(0, 2 * np.pi, 100)

        # Calculate the sine of each x value
        y = np.sin(x)

        # Create a new figure with a single subplot
        plt.figure()
        plt.plot(x, y)  # Plot the sine wave

        # Set the title of the plot and labels for the x and y axes
        plt.title("Sine Wave")
        plt.xlabel("x")
        plt.ylabel("sin(x)")

        # Display the plot
        plt.show()


    def plot_lidar_fit_square(self, fit_square, get_last_cycle_readings, axes_range):
        """ Plots the point cloud and the fitted square from the lidar
        :param fit_square: a function
        :param get_last_cycle_readings: a function """
        
        def update_plot(frame):
            plt.cla()  # Clear the current axes
            square_vertices = fit_square()
            print(square_vertices)
            cycle_readings = get_last_cycle_readings()
            # Add the first point at the end to close the square
            square_vertices.append(square_vertices[0])
            # This unpacks the vertices into x and y coordinates
            square_x, square_y = zip(*square_vertices)
            plt.plot(square_x, square_y, 'r-')  # Plot the square in red

            # Plot the points from the last cycle readings, with improved error handling
            if cycle_readings:
                # Handle error caused by abnormal cycle_readings data
                try:
                    _, readings_r, readings_theta = zip(*cycle_readings)
                    readings_r = np.array(readings_r)
                    readings_theta = np.array(readings_theta)
                    plt.scatter(readings_r * np.cos(np.radians(readings_theta)), 
                                readings_r * np.sin(np.radians(readings_theta)), s=10, color='blue')  # Ensure the scatter plot is colored as expected
                except ValueError as e:
                    print(f'cycle_readings = {cycle_readings}, \nError: {e}')

            plt.xlim(axes_range[0], axes_range[1])
            plt.ylim(axes_range[0], axes_range[1])

        # Setup the figure and axis
        plt.figure(figsize=(8, 8))

        # Create an animation that updates the plot
        ani = FuncAnimation(plt.gcf(), update_plot, interval=1000, frames=20, blit=True)  # Update every 1000 ms

        # ani.save('myanimation.mp4', writer='ffmpeg')
        plt.show()