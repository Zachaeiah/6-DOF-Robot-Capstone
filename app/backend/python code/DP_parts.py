import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random

class Box:
    def __init__(self, ID:str , name: str, width: float, height:float, FullWeight: float, HalfWeight:float, EmptyWeight:float, CurrentWeight:float, InService:bool, position: list, orientation: float):
        """Initialize a Box object.

        Args:
            name (str): The name of the box.
            width (int): The width of the box.
            height (int): The height of the box.
        """
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
        return f"Name: {self.name}, Width = {self.width}, Height = {self.height}, Position = {self.position})"

    def __repr__(self):
        return f"Name: {self.name}, Width = {self.width}, Height = {self.height}, Position={self.position})"

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

class PartsDatabase:
    def __init__(self, db_name="parts.db", grid_size: tuple = (1.40, 1.40), shelf_height:float=0.01, margin:float=0.01, offset:float=0.01):
        self.db_name = db_name
        self.conn = None
        self.cursor = None

        self.grid_size = grid_size
        self.shelf_height = shelf_height
        self.margin = margin
        self.offset = offset
        self.boxes = []
        self.current_position = [offset, offset]
        self.no_go_rectangles = []  # List to store "NO_GO" rectangles

        self.plaserNW = BoxPlacer(grid_size, shelf_height, margin, offset) # north wall
        self.plaserEW = BoxPlacer(grid_size, shelf_height, margin, offset) # east wall
        self.plaserWW = BoxPlacer(grid_size, shelf_height, margin, offset) # west wall
        self.plaserSW = BoxPlacer(grid_size, shelf_height, margin, offset) # south wall

        self.create_parts_table()

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



    def connect(self):
        try:
            self.conn = sqlite3.connect(self.db_name)  
            self.cursor = self.conn.cursor()
            
        except sqlite3.Error as e:
            print(f"SQLite error during connection: {e}")

    def disconnect(self):
        try:
            if self.conn:
                self.conn.close()
                
        except sqlite3.Error as e:
            print(f"SQLite error during disconnection: {e}")

    def create_parts_table(self):
        self.connect()
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS Parts (
                PartID TEXT NOT NULL,
                PartName TEXT NOT NULL,
                LocationX REAL NOT NULL,
                LocationY REAL NOT NULL,
                LocationZ REAL NOT NULL,
                OrientationX REAL,
                OrientationY REAL,
                OrientationZ REAL,
                FullWeight REAL NOT NULL,
                HalfWeight REAL NOT NULL,
                EmptyWeight REAL NOT NULL,
                CurrentWeight REAL,
                InService INTEGER
            )
        """)
        self.conn.commit()
        self.disconnect()

    def add_part(self, part_data):
        self.connect()
        orientation = part_data['Orientation']
        self.cursor.execute("""
            INSERT INTO Parts (PartID, PartName, LocationX, LocationY, LocationZ,
            OrientationX, OrientationY, OrientationZ,
            FullWeight, HalfWeight, EmptyWeight, CurrentWeight, InService)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            part_data['PartID'],
            part_data['PartName'],
            part_data['LocationX'],
            part_data['LocationY'],
            part_data['LocationZ'],
            orientation[0],
            orientation[1],
            orientation[2],
            part_data['FullWeight'],
            part_data['HalfWeight'],
            part_data['EmptyWeight'],
            part_data.get('CurrentWeight', 0),
            part_data.get('InService', 1),
        ))
        self.conn.commit()
        self.disconnect()
        print("Part added successfully.")

    def get_part_by_name(self, part_name):
        self.connect()
        self.cursor.execute("SELECT * FROM Parts WHERE PartName = ?", (part_name,))
        part = self.cursor.fetchone()
        self.disconnect()
        if part:
            return part
        else:
            return False

    def print_part_info(self, part):
        if part:
            print("Part found:")
            for i, column_name in enumerate(['PartID', 'PartName', 'LocationX', 'LocationY', 'LocationZ', 'OrientationX', 'OrientationY', 'OrientationZ', 'FullWeight', 'HalfWeight', 'EmptyWeight', 'CurrentWeight', 'InService']):
                print(f"{column_name}: {part[i]}")
        else:
            print("Part not found")

    def edit_part(self, part_name, updated_data):
        try:
            existing_part = self.get_part_by_name(part_name)
            self.connect()
            if existing_part:
                updated_part_data = {key: updated_data.get(key, existing_part[i]) for i, key in enumerate(['PartName', 'NumberOfParts', 'LocationX', 'LocationY', 'LocationZ', 'OrientationX', 'OrientationY', 'OrientationZ', 'FullWeight', 'HalfWeight', 'EmptyWeight', 'CurrentWeight', 'InService'])}

                self.cursor.execute("""
                    UPDATE Parts SET
                    PartName=?, NumberOfParts=?, LocationX=?, LocationY=?, LocationZ=?,
                    OrientationX=?, OrientationY=?, OrientationZ=?,
                    FullWeight=?, HalfWeight=?, EmptyWeight=?, CurrentWeight=?, InService=?
                    WHERE PartName=?
                """, (updated_part_data['PartName'], updated_part_data['NumberOfParts'], updated_part_data['LocationX'], updated_part_data['LocationY'], updated_part_data['LocationZ'], updated_part_data['OrientationX'], updated_part_data['OrientationY'], updated_part_data['OrientationZ'], updated_part_data['FullWeight'], updated_part_data['HalfWeight'], updated_part_data['EmptyWeight'], updated_part_data['CurrentWeight'], updated_part_data['InService'], part_name))

                self.conn.commit()
                print(f"Part '{part_name}' updated successfully.")
            else:
                print("Part not found. Cannot update.")

        except sqlite3.Error as e:
            print(f"SQLite error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

        finally:
            self.disconnect()

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

def main():
    parts_db = PartsDatabase()

    demo_add_parts(parts_db)


if __name__ == "__main__":
    main()
    