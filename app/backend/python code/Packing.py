import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import random

class Box:
    def __init__(self, name: str, width, height):
        """Initialize a Box object.

        Args:
            name (str): The name of the box.
            width (int): The width of the box.
            height (int): The height of the box.
        """
        self.name = name
        self.width = width
        self.height = height
        self.FullWeight = None
        self.HalfWeight = None
        self.EmptyWeight = None
        self.CurrentWeight = None
        self.InService = False
        self.position = None
    
    
    def __str__(self):
        return f"Name: {self.name}, Width = {self.width}, Height = {self.height}, Position = {self.position})"

    def __repr__(self):
        return f"Name: {self.name}, Width = {self.width}, Height = {self.height}, Position={self.position})"

def generate_random_boxes(num_boxes, max_width, max_height):
    """Generate a list of random Box objects.

    Args:
        num_boxes (int): The number of boxes to generate.
        max_width (int): The maximum width of the boxes.
        max_height (int): The maximum height of the boxes.

    Returns:
        List[Box]: A list of Box objects.
    """
    return [Box(f"Box-{i+1}", random.randint(1, max_width), max_height) for i in range(num_boxes)]

def generate__boxes(num_boxes: int, widths: list[float], heights: list[float], names: list[str]):
    """_summary_

    Args:
        num_boxes (int): _description_
        widths (list[float]): _description_
        heights (list[float]): _description_
        names (list[str]): _description_

    Returns:
        _type_: _description_
    """

    return [Box(names[i], heights[i], widths[i]) for i in range(num_boxes)]


class BoxPlacer:
    def __init__(self, grid_size, shelf_height=0.05, margin=0.05, offset=0.0):
        """Initialize a BoxPlacer object.

        Args:
            grid_size (Tuple[int, int]): The size of the grid (width, height).
            shelf_height (int, optional): The height of the shelves. Defaults to 20.
            margin (int, optional): The margin between boxes. Defaults to 10.
            offset (int, optional): The offset from the edge of the grid. Defaults to 10.
        """
        self.grid_size = grid_size
        self.shelf_height = shelf_height
        self.margin = margin
        self.offset = offset
        self.boxes = []
        self.current_position = [offset, offset]
        self.no_go_rectangles = []  # List to store "NO_GO" rectangles

    def add_no_go_rectangle(self, x, y, width, height):
        """Add a "NO_GO" rectangle to the grid.

        Args:
            x (int): X-coordinate of the rectangle.
            y (int): Y-coordinate of the rectangle.
            width (int): Width of the rectangle.
            height (int): Height of the rectangle.
        """
        self.no_go_rectangles.append(patches.Rectangle((x, y), width, height, facecolor="gray"))

    def add_box(self, new_box):
        """Add a new box to the list and attempt to place only the new box is in it locashion .

        Args:
            new_box (Box): The new box to add.

        Returns:
            bool: True if the box was placed, False otherwise.
        """
        unplaced_boxes = []

        if new_box.position is not None:
            # Box already has a position, no need to place it again
            self.boxes.append(new_box) 
            return True
        
        elif ( # add it to the end of the plaed boxes
            self.current_position[0] + new_box.width + self.margin > self.grid_size[0]
            or self.current_position[1] + new_box.height + self.margin > self.grid_size[1]
            or any(
                self.is_overlap(
                    self.current_position[0], self.current_position[1], new_box.width, new_box.height, rect
                )
                for rect in self.no_go_rectangles
            )
        ):
            # Move to the next row if the new_box doesn't fit in the current row or overlaps with "NO_GO" space
            self.current_position[0] = self.offset
            self.current_position[1] += new_box.height + self.shelf_height

            # If the next row is outside the grid, mark the box as unplaced and break
            if self.current_position[1] > self.grid_size[1]:
                unplaced_boxes.append(new_box)
                return False

            # Check if the current position is just before the "NO_GO" rectangle
            overlapping_rectangles = [
                rect for rect in self.no_go_rectangles if self.is_overlap(
                    self.current_position[0], self.current_position[1], 0, new_box.height, rect
                )
            ]

            if overlapping_rectangles:
                # Adjust the current position to be just after the "NO_GO" rectangle
                self.current_position[0] = overlapping_rectangles[0].get_x() + overlapping_rectangles[0].get_width() + self.margin

        if new_box not in unplaced_boxes:
            new_box.position = tuple(self.current_position)
            self.boxes.append(new_box)
            self.current_position[0] += new_box.width + self.margin
        
    def get_box_positions(self):
        """Get the positions of all placed boxes.

        Returns:
            Dict[str, Tuple[int, int]]: A dictionary mapping box names to their positions (x, y).
        """
        box_positions = {}
        for box in self.boxes:
            if box.position is not None:
                box_positions[box.name] = box.position
        return box_positions
    
    def remove_box_by_name(self, box_name):
        """Remove a box by its name.

        Args:
            box_name (str): The name of the box to be removed.

        Returns:
            Box or None: The removed box or None if the box with the given name is not found.
        """
        for box in self.boxes:
            if box.name == box_name:
                self.boxes.remove(box)
                return box
        return None

    def remove_box_by_position(self, position):
        """Remove a box by its position.

        Args:
            position (Tuple[int, int]): The position (x, y) of the box to be removed.

        Returns:
            Box or None: The removed box or None if a box with the given position is not found.
        """
        for box in self.boxes:
            if box.position == position:
                self.boxes.remove(box)
                return box
        return None
    
    def clear_all_boxes(self):
        """Clear all boxes from the grid."""
        self.boxes = []
        self.current_position = [self.offset, self.offset]
        return self.current_position
    
    def get_total_occupied_space(self):
        """Get the total occupied space by the placed boxes.

        Returns:
            Tuple[int, int]: Total width and height occupied by the boxes.
        """
        total_width = max(box.position[0] + box.width for box in self.boxes) - self.offset
        total_height = max(box.position[1] + box.height for box in self.boxes) - self.offset
        return total_width, total_height
    
    def get_total_boxes_count(self):
        """Get the total number of placed boxes.

        Returns:
            int: Total number of placed boxes.
        """
        return len(self.boxes)
    
    def get_no_go_rectangles(self):
        """Get the coordinates and dimensions of the "NO_GO" rectangles.

        Returns:
            List[Tuple[int, int, int, int]]: List of (x, y, width, height) for each "NO_GO" rectangle.
        """
        return [(rect.get_x(), rect.get_y(), rect.get_width(), rect.get_height()) for rect in self.no_go_rectangles]

    def place_boxes(self):
        """Attempt to place all boxes on the grid.

        Returns:
            List[Box]: Unplaced boxes.
        """
        sorted_boxes = sorted(self.boxes, key=lambda x: x.width, reverse=True)
        self.current_position = [self.offset, self.offset]
        unplaced_boxes = []

        for box in sorted_boxes:
            while (
                self.current_position[0] + box.width + self.margin > self.grid_size[0]
                or self.current_position[1] + box.height + self.margin > self.grid_size[1]
                or any(
                    self.is_overlap(
                        self.current_position[0], self.current_position[1], box.width, box.height, rect
                    )
                    for rect in self.no_go_rectangles
                )
            ):
                # Move to the next row if the box doesn't fit in the current row or overlaps with "NO_GO" space
                self.current_position[0] = self.offset
                self.current_position[1] += box.height + self.shelf_height

                # If the next row is outside the grid, mark the box as unplaced and break
                if self.current_position[1] > self.grid_size[1]:
                    unplaced_boxes.append(box)
                    break

                # Check if the current position is just before the "NO_GO" rectangle
                overlapping_rectangles = [
                    rect for rect in self.no_go_rectangles if self.is_overlap(
                        self.current_position[0], self.current_position[1], 0, box.height, rect
                    )
                ]

                if overlapping_rectangles:
                    # Adjust the current position to be just after the "NO_GO" rectangle
                    self.current_position[0] = overlapping_rectangles[0].get_x() + overlapping_rectangles[0].get_width() + self.margin

            if box not in unplaced_boxes:
                box.position = tuple(self.current_position)
                self.current_position[0] += box.width + self.margin

        return unplaced_boxes

    def is_overlap(self, x, y, width, height, rect):
        """Check if a box overlaps with a rectangle.

        Args:
            x (int): X-coordinate of the box.
            y (int): Y-coordinate of the box.
            width (int): Width of the box.
            height (int): Height of the box.
            rect (patches.Rectangle): The rectangle to check for overlap.

        Returns:
            bool: True if there is an overlap, False otherwise.
        """
        return (
            x < rect.get_x() + rect.get_width()
            and x + width > rect.get_x()
            and y < rect.get_y() + rect.get_height()
            and y + height > rect.get_y()
            and not (
                x >= rect.get_x()
                and y >= rect.get_y()
                and x + width <= rect.get_x() + rect.get_width()
                and y + height <= rect.get_y() + rect.get_height()
            )
        )

    def plot_boxes(self):
        """Plot the boxes and "NO_GO" rectangles on the grid."""
        fig, ax = plt.subplots()
        ax.set_xlim(0, self.grid_size[0])
        ax.set_ylim(0, self.grid_size[1])

        for rect in self.no_go_rectangles:
            ax.add_patch(rect)

        for box in self.boxes:
            if box.position is not None:
                rect = patches.Rectangle(
                    box.position, box.width, box.height, linewidth=1, edgecolor="r", facecolor="none"
                )
                ax.add_patch(rect)
                ax.text(
                    box.position[0] + 0.5 * box.width,
                    box.position[1] + 0.5 * box.height,
                    box.name,
                    ha="center",
                    va="center",
                    color="b",
                )

        plt.show()
        plt.close(fig)

    def __str__(self):
        return f"BoxPlacer(grid_size={self.grid_size}, shelf_height={self.shelf_height}, margin={self.margin}, offset={self.offset})"

    def __repr__(self):
        return f"BoxPlacer(grid_size={self.grid_size}, shelf_height={self.shelf_height}, margin={self.margin}, offset={self.offset}, boxes={self.boxes}, no_go_rectangles={self.no_go_rectangles})"

    def __len__(self):
        return len(self.boxes)
    
    def __getitem__(self, index):
        return self.boxes[index]
    
    def __contains__(self, box):
        return box in self.boxes
    
    def __bool__(self):
        return bool(self.boxes)


def main():
    # Example usage
    grid_size = (1.40, 1.40)
    box_placer = BoxPlacer(grid_size, shelf_height=0.1, margin=0.01, offset=0.01)

    num_boxes = 50
    box_widths = 0.1*np.ones(num_boxes)
    box_hights = 0.15*np.ones(num_boxes)
    box_names = [f"Box:{num}" for num in np.arange(0, num_boxes)]

    # Initial boxes
    initial_boxes = generate__boxes(num_boxes, box_widths, box_hights, box_names)

    box_placer.boxes.extend(initial_boxes)

    # Place and plot initial boxes
    remaining_boxes = box_placer.place_boxes()

    box_placer.plot_boxes()

if __name__ == "__main__":
    main()



