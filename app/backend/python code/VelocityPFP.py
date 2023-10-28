import math
import numpy as np
  
class Linear:
    def __init__(self, slope: float, offset: float):
        """
        Initialize a linear function.

        :param slope: The slope of the linear function.
        :param offset: The offset of the linear function.
        """
        self.slope = slope
        self.offset = offset

    def velocity(self, time: float) -> float:
        """
        Calculate the velocity at a given time using the linear function.

        :param time: The time at which to calculate the velocity.
        :return: The calculated velocity.
        """
        return self.slope * time + self.offset

class Sigmoid:
    def __init__(self, growth_rate: float, time_shift: float):
        """
        Initialize a sigmoid function.

        :param growth_rate: The growth rate of the sigmoid function.
        :param time_shift: The time shift of the sigmoid function.
        """
        self.growth_rate = growth_rate
        self.time_shift = time_shift

    def velocity(self, time: float) -> float:
        """
        Calculate the velocity at a given time using the sigmoid function.

        :param time: The time at which to calculate the velocity.
        :return: The calculated velocity.
        """
        try:
            return 1 / math.exp(-self.growth_rate * (time - self.time_shift))
        except OverflowError as e:
            raise ValueError(f"OverflowError: {e}. Growth rate too high for the given time and time shift")
        
import numpy as np
import matplotlib.pyplot as plt

# Trapezoidal Velocity Profile
def trapezoidal_velocity_profile(distance, max_velocity, acceleration, deceleration):
    acceleration_time = max_velocity / acceleration
    deceleration_time = max_velocity / deceleration

    if acceleration_time + deceleration_time >= distance / max_velocity:
        acceleration_time = distance / max_velocity - deceleration_time

    t1 = acceleration_time
    t2 = distance / max_velocity - deceleration_time
    t3 = distance / max_velocity

    time = np.arange(0, t3, 0.01)
    velocity = np.zeros_like(time)

    for i, t in enumerate(time):
        if t <= t1:
            velocity[i] = acceleration * t
        elif t <= t2:
            velocity[i] = max_velocity
        else:
            velocity[i] = max_velocity - deceleration * (t - t2)

    return time, velocity

# S-Curve (Seven-Segment) Motion Profile
def s_curve_motion_profile(distance, max_velocity, acceleration, jerk):
    t1 = acceleration / jerk
    t2 = max_velocity / acceleration - t1
    t3 = t1

    if t1 + t2 + t3 >= distance / max_velocity:
        t2 = distance / max_velocity - t1 - t3

    time = np.arange(0, t1 + t2 + t3, 0.01)
    velocity = np.zeros_like(time)
    acceleration_values = np.zeros_like(time)

    for i, t in enumerate(time):
        if t <= t1:
            acceleration_values[i] = jerk * t
            velocity[i] = 0.5 * jerk * t ** 2
        elif t <= t1 + t2:
            acceleration_values[i] = acceleration
            velocity[i] = max_velocity * (t - 0.5 * t1)
        else:
            acceleration_values[i] = -jerk * (t - t1 - t2)
            t_rel = t - t1 - t2
            velocity[i] = max_velocity - 0.5 * jerk * (t_rel) ** 2 - jerk * t_rel * (t_rel - t3)

    return time, velocity, acceleration_values

# Combined Profile
def combined_velocity_profile(distance, max_velocity, acceleration, deceleration, jerk):
    t_trap, vel_trap = trapezoidal_velocity_profile(distance, max_velocity, acceleration, deceleration)
    t_s_curve, vel_s_curve, _ = s_curve_motion_profile(distance, max_velocity, acceleration, jerk)

    min_len = min(len(vel_trap), len(vel_s_curve))
    vel_trap = vel_trap[:min_len]
    vel_s_curve = vel_s_curve[:min_len]

    combined_velocity = np.minimum(vel_trap, vel_s_curve)

    return t_trap[:min_len], combined_velocity

# Plotting the Combined Profile
def plot_combined_profile(time, velocity, title):
    plt.plot(time, velocity, label='Combined Velocity')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title(title)
    plt.legend()
    plt.show()

# Example usage for the combined profile
distance = 1000
max_velocity = 10
acceleration = 2
deceleration = 2
jerk = 1

time_combined, velocity_combined = combined_velocity_profile(distance, max_velocity, acceleration, deceleration, jerk)
plot_combined_profile(time_combined, velocity_combined, 'Combined Velocity Profile')

