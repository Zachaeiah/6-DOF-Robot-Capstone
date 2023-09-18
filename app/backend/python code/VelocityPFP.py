import math
  
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
        

def main():
    print(__name__)
        
if __name__ == "__main__":
    main()