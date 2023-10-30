import numpy as np
import matplotlib.pyplot as plt

class VelocityTypes:
    def __init__(self, max_value, max_derivative):
        """Initialize the VelocityTypes class.

        Args:
            max_value (float): The maximum value for the velocity.
            max_derivative (float): The maximum derivative value.
        """
        self.max_value = max_value
        self.max_derivative = max_derivative

    def linear(self) -> np.ndarray:
        """Generate a linear acceleration array.

        Returns:
            np.ndarray: An array representing linear acceleration.
        """
        acceleration_time = self.max_value / self.max_derivative

        t_values = np.unique(np.concatenate((
            np.linspace(0, 0.25 * acceleration_time, int(2 * acceleration_time)),
            np.linspace(0.25 * acceleration_time, 0.75 * acceleration_time, int(acceleration_time)),
            np.linspace(0.75 * acceleration_time, acceleration_time, int(2 * acceleration_time))
        )))

        scaled_values = (t_values - t_values[0]) / (t_values[-1] - t_values[0])
        return scaled_values

    def sigmoid(self, start_point=(0, 0), end_point=(1, 1)):
        """Generate a sigmoid-like acceleration array.

        Args:
            start_point (tuple, optional): Starting point coordinates. Defaults to (0, 0).
            end_point (tuple, optional): Ending point coordinates. Defaults to (1, 1).

        Returns:
            np.ndarray: An array representing sigmoid-like acceleration.
        """
        def custom_sigmoid(x, growth_rate, mid_point, max_value_change):
            """Compute the custom sigmoid function.

            Args:
                x (float): The input value.
                growth_rate (float): The growth rate parameter.
                mid_point (float): The midpoint of the sigmoid function.
                max_value_change (float): The maximum value change.

            Returns:
                float: The result of the custom sigmoid function.
            """
            return max_value_change / (1 + np.exp(-growth_rate * (x - mid_point)))

        acceleration_time = self.max_value / self.max_derivative

        growth_rate = 4 * self.max_derivative / (end_point[0] - start_point[0])
        mid_point = (start_point[0] + end_point[0]) / 2
        max_value_change = end_point[1] - start_point[1]

        x_values = np.linspace(start_point[0], end_point[0], int(5 * acceleration_time))
        y_values = custom_sigmoid(x_values, growth_rate, mid_point, max_value_change) + start_point[1]

        return y_values


def main():
    velocity_obj = VelocityTypes(100, 5)  # Example values

    sigmoid_values = velocity_obj.sigmoid()
    linear_values = velocity_obj.linear()

    print(len(sigmoid_values))

    # Plotting the velocity over time
    plt.plot(sigmoid_values, label='Sigmoid', color='blue')
    plt.plot(linear_values, label='Linear', color='red')

    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
