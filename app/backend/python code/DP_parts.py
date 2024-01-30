import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import inspect
import random


class Box:
    def __init__(self, ID: str, name: str, width: float, height: float, FullWeight: float, HalfWeight: float, EmptyWeight: float, CurrentWeight: float, InService: bool, position: list, orientation: list):
        self.ID = ID
        self.name = name
        self.width = width
        self.height = height
        self.FullWeight = FullWeight
        self.HalfWeight = HalfWeight
        self.EmptyWeight = EmptyWeight
        self.CurrentWeight = CurrentWeight
        self.InService = InService
        self.position = position
        self.orientation = orientation


    
    def __str__(self):
        return f"ID: {self.ID}\nName: {self.name}\nWidth: {self.width}\nHeight: {self.height}\nFull Weight: {self.FullWeight}\nHalf Weight: {self.HalfWeight}\nEmpty Weight: {self.EmptyWeight}\nCurrent Weight: {self.CurrentWeight}\nIn Service: {self.InService}\nPosition: {self.position}\nOrientation: {self.orientation}"


class PartsDatabase:
    def __init__(self, db_name="parts.db", grid_size: tuple = (1.40, 1.40), shelf_height: float = 0.01, margin: float = 0.01, offset_x: float = 0.01, offset_y: float = 0.01):
        """Initialize PartsDatabase.

        Args:
            db_name (str, optional): Name of the database file. Defaults to "parts.db".
            grid_size (tuple, optional): Size of the grid. Defaults to (1.40, 1.40).
            shelf_height (float, optional): Height of each shelf. Defaults to 0.01.
            margin (float, optional): Margin between boxes. Defaults to 0.01.
            offset_x (float, optional): Offset from the origin along the x-axis. Defaults to 0.01.
            offset_y (float, optional): Offset from the origin along the y-axis. Defaults to 0.01.
        """
        self.db_name = db_name
        self.grid_size = grid_size
        self.shelf_height = shelf_height
        self.margin = margin
        self.offset_x = offset_x
        self.offset_y = offset_y
        self.boxes = []
        self.current_position = [offset_x, offset_y]
        self.no_go_rectangles = []  # List to store "NO_GO" rectangles

    def connect(self):
        """Establish connection to the database."""
        try:
            self.conn = sqlite3.connect(self.db_name)
            self.cursor = self.conn.cursor()
        except sqlite3.Error as e:
            print(f"Error connecting to the database: {e}")

    def disconnect(self):
        """Close connection to the database."""
        try:
            if self.conn:
                self.conn.close()
        except sqlite3.Error as e:
            print(f"Error disconnecting from the database: {e}")

    def create_parts_table(self):
        """Create 'Parts' table in the database if it doesn't exist."""
        try:
            self.connect()
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS Parts (
                    ID TEXT PRIMARY KEY,
                    Name TEXT NOT NULL,
                    Width REAL NOT NULL,
                    Height REAL NOT NULL,
                    FullWeight REAL NOT NULL,
                    HalfWeight REAL NOT NULL,
                    EmptyWeight REAL NOT NULL,
                    CurrentWeight REAL,
                    InService INTEGER,
                    Position TEXT NOT NULL,
                    Orientation TEXT NOT NULL
                )
            """)
            self.conn.commit()
        except sqlite3.Error as e:
            print(f"Error creating 'Parts' table: {e}")
        finally:
            self.disconnect()

    def add_no_go_rectangle(self, x, y, width, height):
        """Add a 'NO_GO' rectangle to the grid.

        Args:
            x (float): X-coordinate of the rectangle.
            y (float): Y-coordinate of the rectangle.
            width (float): Width of the rectangle.
            height (float): Height of the rectangle.
        """
        self.no_go_rectangles.append(patches.Rectangle((x, y), width, height, facecolor="gray"))

    def add_part(self, box: Box) -> bool:
        """Add a part to the PartsDatabase.

        Args:
            box (Box): The Box object to be added.

        Returns:
            bool: True if the part is successfully added, False otherwise.
        """
        try:
            self.connect()
            unplaced_boxes = []

            # If the box already has a position, check if it exists in the database
            if box.position is not []:
                if box not in self.boxes:
                    self.boxes.append(box)             
                
            # If the box does not have a position, attempt to place it in the grid
            elif (
                self.current_position[0] + box.width + self.margin > self.grid_size[0]
                or self.current_position[1] + box.height + self.margin > self.grid_size[1]
                or any(
                    self.is_overlap(
                        self.current_position[0], self.current_position[1], box.width, box.height, rect
                    )
                    for rect in self.no_go_rectangles
                )
            ):
                # Move to the next row if the new box doesn't fit in the current row or overlaps with "NO_GO" space
                self.current_position[0] = round(self.offset_x, 3)
                self.current_position[1] += round(box.height + self.shelf_height,3)

                # If the next row is outside the grid, mark the box as unplaced and return False
                if self.current_position[1] > self.grid_size[1]:
                    unplaced_boxes.append(box)
                    return False

                # Check if the current position is just before the "NO_GO" rectangle
                overlapping_rectangles = [
                    rect for rect in self.no_go_rectangles if self.is_overlap(
                        self.current_position[0], self.current_position[1], 0, box.height, rect
                    )
                ]

                if overlapping_rectangles:
                    # Adjust the current position to be just after the "NO_GO" rectangle
                    self.current_position[0] = round(overlapping_rectangles[0].get_x() + overlapping_rectangles[0].get_width() + self.margin, 3)

            # If the box is not marked as unplaced, assign its position and add it to the list of placed boxes
            if box not in unplaced_boxes:
                box.position = tuple(self.current_position)
                self.boxes.append(box)
                self.store_part(box)
                self.current_position[0] += round(box.width + self.margin, 3)
                return True
        except Exception as e:
            print(f"Error adding part: {e}")
            return False
        finally:
            self.disconnect()

    def is_overlap(self, x, y, width, height, rect):
        """Check if a box overlaps with a rectangle.

        Args:
            x (float): X-coordinate of the box.
            y (float): Y-coordinate of the box.
            width (float): Width of the box.
            height (float): Height of the box.
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

    def store_part(self, box: Box) -> bool:
        """Store a part in the database.

        Args:
            box (Box): The Box object to store in the database.

        Returns:
            bool: True if the part is successfully stored, False otherwise.
        """
        try:
            self.connect()
            # Execute SQL query to insert the part into the database
            self.cursor.execute("""
                INSERT INTO Parts (ID, Name, Width, Height, FullWeight, HalfWeight, EmptyWeight, CurrentWeight, InService, Position, Orientation)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                box.ID,
                box.name,
                box.width,
                box.height,
                box.FullWeight,
                box.HalfWeight,
                box.EmptyWeight,
                box.CurrentWeight,
                box.InService,
                str(box.position), 
                str(box.orientation)
            ))
            self.conn.commit()
            return True
        except sqlite3.Error as e:
            print(f"Error storing part: {e}")
            return False
        finally:
            self.disconnect()

    def get_part_by_name(self, part_name: str) -> Box:
        """Retrieve a part from the database by its name.

        Args:
            part_name (str): The name of the part to retrieve.

        Returns:
            Box: The part with the specified name if found, None otherwise.
        """
        try:
            self.connect()
            # Execute SQL query to retrieve the part with the specified name
            self.cursor.execute("SELECT * FROM Parts WHERE name=?", (part_name,))
            # Fetch the first row returned by the query
            row = self.cursor.fetchone()
            # If a row is found, construct a Box object from the data and return it
            if row:
                return Box(*row)
            else:
                print(f"No part with name '{part_name}' found.")
                return None
        except sqlite3.Error as e:
            print(f"Error retrieving part: {e}")
            return None
        finally:
            self.disconnect()

    def get_part_by_ID(self, part_ID: str) -> Box:
        """Retrieve a part from the database by its ID.

        Args:
            part_ID (str): The ID of the part to retrieve.

        Returns:
            Box: The part with the specified ID if found, None otherwise.
        """
        try:
            self.connect()
            # Execute SQL query to retrieve the part with the specified ID
            self.cursor.execute("SELECT * FROM Parts WHERE ID=?", (part_ID,))
            # Fetch the first row returned by the query
            row = self.cursor.fetchone()
            # If a row is found, construct a Box object from the data and return it
            if row:
                return Box(*row)
            else:
                print(f"No part with ID '{part_ID}' found.")
                return None
        except sqlite3.Error as e:
            print(f"Error retrieving part: {e}")
            return None
        finally:
            self.disconnect()

    def update_part_by_name(self, part_name: str, **kwargs) -> bool:
        """Update properties of a part by its name.

        Args:
            part_name (str): The name of the part to update.
            **kwargs: Keyword arguments representing properties to update.

        Returns:
            bool: True if the part is successfully updated, False otherwise.
        """
        try:
            self.connect()
            # Generate SQL UPDATE query
            set_clause = ", ".join([f"{key} = ?" for key in kwargs.keys()])
            query = f"UPDATE Parts SET {set_clause} WHERE Name = ?"
            # Extract values to update and append part_name
            values = tuple(kwargs.values()) + (part_name,)
            # Execute the update query
            self.cursor.execute(query, values)
            self.conn.commit()
            return True
        except sqlite3.Error as e:
            print(f"Error updating part: {e}")
            return False
        finally:
            self.disconnect()

    def update_part_by_ID(self, part_ID: str, **kwargs) -> bool:
        """Update properties of a part by its ID.

        Args:
            part_ID (str): The ID of the part to update.
            **kwargs: Keyword arguments representing properties to update.

        Returns:
            bool: True if the part is successfully updated, False otherwise.
        """
        try:
            self.connect()
            # Generate SQL UPDATE query
            set_clause = ", ".join([f"{key} = ?" for key in kwargs.keys()])
            query = f"UPDATE Parts SET {set_clause} WHERE ID = ?"
            # Extract values to update and append part_ID
            values = tuple(kwargs.values()) + (part_ID,)
            # Execute the update query
            self.cursor.execute(query, values)
            self.conn.commit()
            return True
        except sqlite3.Error as e:
            print(f"Error updating part: {e}")
            return False
        finally:
            self.disconnect()

def main():
    box_w = 0.2
    box_h = 0.25
    # Initialize the PartsDatabase instance
    db = PartsDatabase("Box_testing_DB", shelf_height= 0.015, margin=0.015, offset_x=(box_w/2), offset_y=(box_h/2))
    
    # Create the Parts table
    db.create_parts_table()


    print(db.get_part_by_ID("ID1"))
    print(db.get_part_by_ID("ID2"))

    # # Example usage:
    # box1 = Box("ID1", "Box 1", box_w, box_h, 20.0, 10.0, 5.0, 15.0, True, [], [1.0, 0.0, 0.0])
    # box2 = Box("ID2", "Box 2", box_w, box_h, 20.0, 10.0, 5.0, 15.0, True, [], [1.0, 0.0, 0.0])

    # db.add_part(box1)
    # db.add_part(box2)

    # print("\nbefor chages\n")

    # # Retrieve and print the part
    # part = db.get_part_by_name("Box 1")
    # print(part.CurrentWeight, part.InService)
    # part = db.get_part_by_name("Box 2")
    # print(part.CurrentWeight, part.InService)

    # db.update_part_by_ID("ID1", CurrentWeight=20, InService = False)
    # db.update_part_by_name("Box 2", CurrentWeight=10, InService = False)

    # print("\nafter chages\n")

    # # Retrieve and print the part
    # part = db.get_part_by_name("Box 1")
    # print(part.CurrentWeight, part.InService)
    # part = db.get_part_by_name("Box 2")
    # print(part.CurrentWeight, part.InService)

if __name__ == "__main__":
    main()

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

def demo_add_parts(parts_db):
    # Input: List of new part data
    parts_to_add = [
        {   
            'PartID': "0",
            'PartName': 'Part0',
            'LocationX': 0.50,
            'LocationY': 0,
            'LocationZ':  0.85,
            'Orientation': [1.0, 0.0, 0.0],
            'FullWeight': 80.0,
            'HalfWeight': 40.0,
            'EmptyWeight': 8.0,
            'CurrentWeight': 75.0,
            'InService': 0,
        },
        {   
            'PartID': "1",
            'PartName': 'Part1',
            'LocationX': 0,
            'LocationY':  0.50,
            'LocationZ':  0.85,
            'Orientation': [0.0, 1.0, 0.0],
            'FullWeight': 50.0,
            'HalfWeight': 25.0,
            'EmptyWeight': 5.0,
            'CurrentWeight': 45.0,
            'InService': 0,
        },
        {   
            'PartID': "2",
            'PartName': 'Part2',
            'LocationX':  -0.50,
            'LocationY':  0.0,
            'LocationZ':  0.85,
            'Orientation': [-1.0, 0, 0],
            'FullWeight': 60.0,
            'HalfWeight': 30.0,
            'EmptyWeight': 6.0,
            'CurrentWeight': 55.0,
            'InService': 0,
        },
        {   
            'PartID': "3",
            'PartName': 'Part3',
            'LocationX':  0.0,
            'LocationY': -0.50,
            'LocationZ':  0.85,
            'Orientation': [0.0, -1.0, 0.0],
            'FullWeight': 70.0,
            'HalfWeight': 35.0,
            'EmptyWeight': 7.0,
            'CurrentWeight': 60.0,
            'InService': 0,
        },
        {   
            'PartID': "4",
            'PartName': 'Part4',
            'LocationX': 0.50,
            'LocationY': 0,
            'LocationZ':  -0.3,
            'Orientation': [1.0, 0.0, 0.0],
            'FullWeight': 80.0,
            'HalfWeight': 40.0,
            'EmptyWeight': 8.0,
            'CurrentWeight': 75.0,
            'InService': 0,
        },
        {   
            'PartID': "5",
            'PartName': 'Part5',
            'LocationX': 0,
            'LocationY':  0.50,
            'LocationZ':  -0.30,
            'Orientation': [0.0, 1.0, 0.0],
            'FullWeight': 50.0,
            'HalfWeight': 25.0,
            'EmptyWeight': 5.0,
            'CurrentWeight': 45.0,
            'InService': 0,
        },
        {   
            'PartID': "6",
            'PartName': 'Part6',
            'LocationX':  -0.50,
            'LocationY':  0.0,
            'LocationZ':  -0.30,
            'Orientation': [-1.0, 0.0, 0.0],
            'FullWeight': 60.0,
            'HalfWeight': 30.0,
            'EmptyWeight': 6.0,
            'CurrentWeight': 55.0,
            'InService': 0,
        },
        {   
            'PartID': "7",
            'PartName': 'Part7',
            'LocationX':  0.0,
            'LocationY': -0.50,
            'LocationZ':  -0.30,
            'Orientation': [0.0, -1.0, 0.0],
            'FullWeight': 70.0,
            'HalfWeight': 35.0,
            'EmptyWeight': 7.0,
            'CurrentWeight': 60.0,
            'InService': 0,
        }
    ]

    # Add the list of parts to the database
    for part_data in parts_to_add:
        parts_db.add_part(part_data)



    # self.plaserNW = BoxPlacer(grid_size, shelf_height, margin, offset) # north wall
        # self.plaserEW = BoxPlacer(grid_size, shelf_height, margin, offset) # east wall
        # self.plaserWW = BoxPlacer(grid_size, shelf_height, margin, offset) # west wall
        # self.plaserSW = BoxPlacer(grid_size, shelf_height, margin, offset) # south wall

        # self.create_parts_table()

        # NORTH_WALL = (0.0, 1.0, 0.0)
        # EAST_WALL = (1.0, 0.0, 0.0)
        # WEST_WALL = (-1.0, 0.0, 0.0)
        # SOUNTH_WALL = (0.0, -1.0, 0.0)


        # self.create_parts_table()

        # # Execute a SELECT query to fetch all records from a table
        # self.connect()
        # self.cursor.execute(f"SELECT * FROM Parts")

        # # Fetch one record at a time and print it
        # for part in self.cursor.fetchall():
        #     Orientation = (part[6], part[7], part[8])
        #     if Orientation == NORTH_WALL:
        #         print("NORTH_WALL")
        #         C_boc = Box(part[0], part[1], part[2])
    