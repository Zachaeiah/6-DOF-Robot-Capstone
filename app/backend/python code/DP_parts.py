import sqlite3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json



class Box:
    def __init__(self, ID: str, name: str, width: float, height: float, FullWeight: float, HalfWeight: float, EmptyWeight: float, CurrentWeight: float, InService: bool, position: list, orientation: tuple):
        """Initialize a Box object.

        Args:
            ID (str): The unique identifier of the box.
            name (str): The name or label of the box.
            width (float): The width of the box.
            height (float): The height of the box.
            FullWeight (float): The weight of the box when full.
            HalfWeight (float): The weight of the box when half-full.
            EmptyWeight (float): The weight of the box when empty.
            CurrentWeight (float): The current weight of the box.
            InService (bool): Indicates whether the box is in service.
            position (list): The position of the box in the grid.
            orientation (tuple): The orientation of the box, represented as a tuple.
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
    
    def __str__(self) -> str:
        """Return a string representation of the Box object."""
        return f"ID: {self.ID}\nName: {self.name}\nWidth: {self.width}\nHeight: {self.height}\nFull Weight: {self.FullWeight}\nHalf Weight: {self.HalfWeight}\nEmpty Weight: {self.EmptyWeight}\nCurrent Weight: {self.CurrentWeight}\nIn Service: {self.InService}\nPosition: {self.position}\nOrientation: {self.orientation}"

class stack_pointer:
    def __init__(self,xcor:float=0, ycor:float=0, priority:int=0, NumBoxes:int=0):
        """_summary_

        Args:
            xcor (float, optional): X position of the pointer
            ycor (float, optional): Y position of the pointer
            priority (int, optional): priority to use this pointer
            NumBoxes (int, optional): how may boxes are stored with this pointer
        """
        self.xcor = xcor
        self.ycor = ycor
        self.priority = priority
        self.NumBoxes = NumBoxes

class PartsDatabase:
    def __init__(self, db_name="parts_db", grid_size: tuple = (1.40, 1.40), shelf_height: float = 0.01, margin: float = 0.01, offset_x: float = 0.01, offset_y: float = 0.01):
        """Initialize the PartsDatabase.

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
        self.load_pointers_from_json()  # Load pointers from JSON
        self.stacking_pointer = stack_pointer()
        self.no_go_rectangles = []  # List to store "NO_GO" rectangles

        self.NORTH_WALL = (0.0, 1.0, 0.0)
        self.EAST_WALL = (1.0, 0.0, 0.0)
        self.WEST_WALL = (-1.0, 0.0, 0.0)
        self.SOUTH_WALL = (0.0, -1.0, 0.0)
    
    def SetWall_tranforms(self, NW:list[list], SW:list[list], EW:list[list], WW:list[list]) -> None:
        """_summary_

        Args:
            NW (list[list]): _description_
            SW (list[list]): _description_
            EW (list[list]): _description_
            WW (list[list]): _description_
        """

        self.NORTH_WALL_tranform = NW
        self.EAST_WALL_tranform  = SW
        self.WEST_WALL_tranform  = EW
        self.SOUTH_WALL_tranform  = WW
        
    
    def load_pointers_from_json(self):
        """Load pointers from a JSON file."""
        try:
            with open("pointers.json", "r") as file:
                pointers_data = json.load(file)
            
            self.NorthWall_pointer = stack_pointer(pointers_data["NorthWall_pointer"]["xcor"], pointers_data["NorthWall_pointer"]["ycor"], pointers_data["NorthWall_pointer"]["priority"], pointers_data["NorthWall_pointer"]["NumBoxes"])
            self.SouthWall_pointer = stack_pointer(pointers_data["SouthWall_pointer"]["xcor"], pointers_data["SouthWall_pointer"]["ycor"], pointers_data["SouthWall_pointer"]["priority"], pointers_data["SouthWall_pointer"]["NumBoxes"])
            self.EastWall_pointer = stack_pointer(pointers_data["EastWall_pointer"]["xcor"], pointers_data["EastWall_pointer"]["ycor"], pointers_data["EastWall_pointer"]["priority"], pointers_data["EastWall_pointer"]["NumBoxes"])
            self.WestWall_pointer = stack_pointer(pointers_data["WestWall_pointer"]["xcor"], pointers_data["WestWall_pointer"]["ycor"], pointers_data["WestWall_pointer"]["priority"], pointers_data["WestWall_pointer"]["NumBoxes"])
        except FileNotFoundError:
            print("Pointer JSON file not found. Using default values.")
            self.NorthWall_pointer = stack_pointer(self.offset_x, self.offset_y, 0, 0)
            self.SouthWall_pointer = stack_pointer(self.offset_x, self.offset_y, 0, 0)
            self.EastWall_pointer = stack_pointer(self.offset_x, self.offset_y, 0, 0)
            self.WestWall_pointer = stack_pointer(self.offset_x, self.offset_y, 0, 0)

    def update_pointers_to_json(self):
        """Update pointers' information to a JSON file."""
        pointers_data = {
            "NorthWall_pointer": {"xcor": self.NorthWall_pointer.xcor, "ycor": self.NorthWall_pointer.ycor, "priority": self.NorthWall_pointer.priority, "NumBoxes": self.NorthWall_pointer.NumBoxes},
            "SouthWall_pointer": {"xcor": self.SouthWall_pointer.xcor, "ycor": self.SouthWall_pointer.ycor, "priority": self.SouthWall_pointer.priority, "NumBoxes": self.SouthWall_pointer.NumBoxes},
            "EastWall_pointer": {"xcor": self.EastWall_pointer.xcor, "ycor": self.EastWall_pointer.ycor, "priority": self.EastWall_pointer.priority, "NumBoxes": self.EastWall_pointer.NumBoxes},
            "WestWall_pointer": {"xcor": self.WestWall_pointer.xcor, "ycor": self.WestWall_pointer.ycor, "priority": self.WestWall_pointer.priority, "NumBoxes": self.WestWall_pointer.NumBoxes}
        }

        try:
            with open("pointers.json", "w") as file:
                json.dump(pointers_data, file)
        except Exception as e:
            print(f"Error updating pointers to JSON file: {e}")

    def connect(self) -> bool:
        """Establish connection to the database."""
        try:
            self.conn = sqlite3.connect(self.db_name)
            self.cursor = self.conn.cursor()
            return True
        except sqlite3.Error as e:
            print(f"Error connecting to the database: {e}")
            return False

    def disconnect(self) -> bool:
        """Close connection to the database."""
        try:
            if self.conn:
                self.conn.close()
                return True
        except sqlite3.Error as e:
            print(f"Error disconnecting from the database: {e}")
            return False

    def create_parts_table(self) -> bool:
        """Create 'Parts' table in the database if it doesn't exist."""
        try:
            self.connect()
            self.cursor.execute("""
                CREATE TABLE IF NOT EXISTS Parts (
                    ID TEXT NOT NULL,
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
            return True
        except sqlite3.Error as e:
            print(f"Error creating 'Parts' table: {e}")
            return False
        finally:
            self.disconnect()

    def add_no_go_rectangle(self, x, y, width, height) -> bool:
        """Add a 'NO_GO' rectangle to the grid.

        Args:
            x (float): X-coordinate of the rectangle.
            y (float): Y-coordinate of the rectangle.
            width (float): Width of the rectangle.
            height (float): Height of the rectangle.
        """
        try:
            self.no_go_rectangles.append(patches.Rectangle((x, y), width, height, facecolor="gray"))
            return True
        except Exception as e:
            print(f"Error NO Go zone: {e}")
            return False
        finally:
            self.disconnect()

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

            box_pointer = []
            if box.orientation == self.NORTH_WALL:
                self.stacking_pointer = self.NorthWall_pointer
            elif box.orientation == self.EAST_WALL:
                self.stacking_pointer = self.EastWall_pointer
            elif box.orientation == self.WEST_WALL:
                self.stacking_pointer = self.WestWall_pointer
            elif box.orientation == self.SOUTH_WALL:  # Corrected spelling of SOUTH_WALL
                self.stacking_pointer = self.SouthWall_pointer

            # If the box already has a position, check if it exists in the database
            if box.position:
                if box not in self.boxes:
                    self.boxes.append(box)

            # If the box does not have a position, attempt to place it in the grid
            elif (
                self.stacking_pointer.xcor + box.width + self.margin > self.grid_size[0]
                or self.stacking_pointer.ycor + box.height + self.margin > self.grid_size[1]
                or any(
                    self.is_overlap(
                        self.stacking_pointer.xcor, self.stacking_pointer.ycor, box.width, box.height, rect
                    )
                    for rect in self.no_go_rectangles
                )
            ):
                # Move to the next row if the new box doesn't fit in the current row or overlaps with "NO_GO" space
                self.stacking_pointer.xcor = round(self.offset_x, 3)
                self.stacking_pointer.ycor += round(box.height + self.shelf_height, 3)

                # If the next row is outside the grid, mark the box as unplaced and return False
                if self.stacking_pointer.ycor > self.grid_size[1]:
                    unplaced_boxes.append(box)
                    return False

                # Check if the current position is just before the "NO_GO" rectangle
                overlapping_rectangles = [
                    rect for rect in self.no_go_rectangles if self.is_overlap(
                        self.stacking_pointer.xcor, self.stacking_pointer.ycor, 0, box.height, rect
                    )
                ]

                if overlapping_rectangles:
                    # Adjust the current position to be just after the "NO_GO" rectangle
                    self.stacking_pointer.xcor = round(overlapping_rectangles[0].get_x() + overlapping_rectangles[0].get_width() + self.margin, 3)

            # If the box is not marked as unplaced, assign its position and add it to the list of placed boxes
            if box not in unplaced_boxes:
                box.position = (self.stacking_pointer.xcor, self.stacking_pointer.ycor, 0)
                self.boxes.append(box)
                self.store_part(box)
                self.stacking_pointer.xcor += round(box.width + self.margin, 3)

                # Update pointers' info to JSON after adding part
                self.update_pointers_to_json()  

                return True
        except Exception as e:
            print(f"Error adding part: {e}")
            return False
        finally:
            self.disconnect()


    def is_overlap(self, x, y, width, height, rect) -> bool:
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

NORTH_WALL = (0.0, 1.0, 0.0)
EAST_WALL = (1.0, 0.0, 0.0)
WEST_WALL = (-1.0, 0.0, 0.0)
SOUTH_WALL = (0.0, -1.0, 0.0)

def main():
    box_w = 0.100
    box_h = 0.092
    num_boxes = 6

    # Initialize the PartsDatabase instance
    db = PartsDatabase("parts_db", shelf_height=0.12, margin=0.100, offset_x=(box_w/2), offset_y=0.3)
    
    # Create the Parts table
    db.create_parts_table()

    
    first_box_cor = [-0.30, 0.70, 0.2]
    x_offset = 0.30
    z_offset = 0.46
    Box_1 = Box("box 1", "box 1", box_w, box_h, 0, 0, 0, 0, False, [-0.30,              0.73, 0.215], NORTH_WALL)
    Box_2 = Box("box 2", "box 2", box_w, box_h, 0, 0, 0, 0, False, [-0.30 + x_offset,   0.73, 0.215], NORTH_WALL)
    Box_3 = Box("box 3", "box 3", box_w, box_h, 0, 0, 0, 0, False, [-0.30 + 2*x_offset, 0.73, 0.215], NORTH_WALL)
    Box_4 = Box("box 4", "box 4", box_w, box_h, 0, 0, 0, 0, False, [-0.30,              0.73, 0.215 + z_offset], NORTH_WALL)
    Box_5 = Box("box 5", "box 5", box_w, box_h, 0, 0, 0, 0, False, [-0.30 + 2*x_offset, 0.73, 0.215 + z_offset], NORTH_WALL)
    db.store_part(Box_1)
    db.store_part(Box_2)
    db.store_part(Box_3)
    db.store_part(Box_4)
    db.store_part(Box_5)

    # # Create and add 30 boxes
    # for orientation in orientations:
    #     for i in range(num_boxes):
    #         box_id = f"box_{Box_n + 1}"
    #         name = f"Box {Box_n + 1}"
    #         current_box = Box(box_id, name, box_w, box_h, 0, 0, 0, 0, False, [], orientation)
    #         Box_n += 1
            
    #         if db.add_part(current_box):
    #             print(f"Box {box_id} added successfully.")
    #         else:
    #             print(f"Failed to add Box {box_id}.")

if __name__ == "__main__":
    main()
