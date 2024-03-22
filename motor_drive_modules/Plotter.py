class Plotter:
    def __init__(self):
        pass


    def test_plot(self):
        """ Test plotting functionalities. 
        Plots a simple sine wave. """ 

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


    
    

    def plot_lidar_clustering(self, clusters:list, axes_range, 
                              colors=('red','yellow','blue','green')):
        """ Plots the edges' clustering result in 4 different colors. """
        
        def update_plot(frame):
            plt.cla()  # Clear the current axes
            i = 0
            for cluster in clusters:
                if cluster:
                    # Handle error caused by abnormal cycle_readings data
                    try:
                        x, y = zip(*cluster)
                        x = np.array(x)
                        y = np.array(y)
                        plt.scatter(x, y, s=10, color=colors[i])
                    except ValueError as e:
                        print(f'cluster = {cluster}, \nError: {e}')
                i += 1

            plt.xlim(axes_range[0], axes_range[1])
            plt.ylim(axes_range[0], axes_range[1])

        # Setup the figure and axis
        plt.figure(figsize=(8, 8))

        # Create an animation that updates the plot

        ani = FuncAnimation(plt.gcf(), update_plot, interval=1000, frames=20, blit=False)  # Update every 1000 ms

        # ani.save('myanimation.mp4', writer='ffmpeg')
        plt.show()