import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Plotter:
    def __init__(self):
        pass


    def plot_lidar_fit_square(fit_square:function, get_last_cycle_readings:function):
        def update_plot(frame):
            plt.cla()  # Clear the current axes
            square_vertices:list = fit_square()
            cycle_readings = get_last_cycle_readings()

            # Plot the square by connecting the vertices and closing the loop
            square_vertices.append(square_vertices[0])  # Add the first point at the end to close the square
            square_x, square_y = zip(*square_vertices)  # This unpacks the vertices into x and y coordinates
            plt.plot(square_x, square_y, 'r-')  # Plot the square in red

            # Plot the points from the last cycle readings
            if cycle_readings:
                readings_x, readings_y = zip(*cycle_readings)  # This unpacks the cycle readings into x and y coordinates
                plt.scatter(readings_x, readings_y, s=10)  # Plot the points as small dots

            plt.xlim(-10, 10)  # Adjust as needed
            plt.ylim(-10, 10)  # Adjust as needed

        # Setup the figure and axis
        plt.figure(figsize=(8, 8))

        # Create an animation that updates the plot
        ani = FuncAnimation(plt.gcf(), update_plot, interval=1000)  # Update every 1000 ms

        plt.show()