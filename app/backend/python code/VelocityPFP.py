import numpy as np
import matplotlib.pyplot as plt

class LSPB:
    def __init__(self, initial_time, final_time, initial_velocity, final_velocity, max_acceleration):
        """
        Initialize an LSPB (Limited S-shaped Trapezoidal Velocity Profile) object.

        Args:
            initial_time (float): Initial time.
            final_time (float): Final time.
            initial_velocity (float): Initial velocity.
            final_velocity (float): Final velocity.
            max_acceleration (float): Maximum acceleration.

        """
        self.initial_time = initial_time
        self.final_time = final_time
        self.initial_velocity = initial_velocity
        self.final_velocity = final_velocity
        self.max_acceleration = max_acceleration

        self.blend_time = (self.initial_velocity - self.final_velocity + self.max_acceleration * self.final_time) / (self.max_acceleration)

    def velocity(self, time: float) -> float:
        """
        Calculate the velocity at a given time.

        Args:
            time (float): Time at which to calculate velocity.

        Returns:
            float: Velocity at the specified time.

        """
        if 0 <= time <= self.blend_time:
            return self.initial_velocity + (self.max_acceleration * time ** 2) / (2 * self.blend_time)

        elif self.blend_time < time <= (self.final_time - self.blend_time):
            return ((self.final_velocity + self.initial_velocity - self.max_acceleration * self.final_time) / 2) + self.max_acceleration * time

        elif (self.final_time - self.blend_time) < time <= self.final_time:
            return self.final_velocity - ((self.max_acceleration * self.final_time ** 2) / (2 * self.blend_time)) + (
                        (self.max_acceleration * self.final_time * time) / self.blend_time) - (
                               self.max_acceleration * time ** 2) / (2 * self.blend_time)

def main():
    initial_time = 0
    final_time = 1
    initial_velocity = 0
    final_velocity = 30
    max_acceleration = 40

    vel_plan = LSPB(initial_time, final_time, initial_velocity, final_velocity, max_acceleration)
    times = np.linspace(initial_time, final_time, 100)
    velocities = [vel_plan.velocity(time) for time in times]

    plt.plot(times, velocities, label='Velocity Profile')
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.legend()
    plt.grid(True)
    plt.title('Limited S-shaped Trapezoidal Velocity Profile')
    plt.show()

if __name__ == "__main__":
    main()
