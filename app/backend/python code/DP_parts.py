import sqlite3
import numpy as np
from faker import Faker
import random

class PartsDatabase:
    def __init__(self, db_name="parts.db"):
        """
        Initialize the PartsDatabase.

        Args:
            db_name (str): The name of the database file.
        """
        self.db_name = db_name
        self.conn = None
        self.cursor = None

    def connect(self):
        """
        Connect to the SQLite database.
        """
        self.conn = sqlite3.connect(self.db_name)
        self.cursor = self.conn.cursor()

    def disconnect(self):
        """
        Disconnect from the SQLite database.
        """
        if self.conn:
            self.conn.close()

    def create_parts_table(self):
        """
        Create the Parts table if it doesn't exist.
        """
        self.connect()
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS Parts (
                PartID INTEGER PRIMARY KEY AUTOINCREMENT,
                PartName TEXT NOT NULL,
                NumberOfParts INTEGER NOT NULL,
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
        """
        Add a new part to the database.

        Args:
            part_data (dict): A dictionary containing part information.
        """
        self.connect()
        orientation = part_data['Orientation']
        self.cursor.execute("""
            INSERT INTO Parts (PartName, NumberOfParts, LocationX, LocationY, LocationZ,
            OrientationX, OrientationY, OrientationZ,
            FullWeight, HalfWeight, EmptyWeight, CurrentWeight, InService)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            part_data['PartName'],
            part_data['NumberOfParts'],
            part_data['LocationX'],
            part_data['LocationY'],
            part_data['LocationZ'],
            orientation[0],
            orientation[1],
            orientation[2],
            part_data['FullWeight'],
            part_data['HalfWeight'],
            part_data['EmptyWeight'],
            part_data.get('CurrentWeight', 0),  # default to 0 if not provided
            part_data.get('InService', 1),  # default to 1 (True) if not provided
        ))
        self.conn.commit()
        self.disconnect()
        print("Part added successfully.")

    def get_part_by_name(self, part_name):
        """
        Retrieve a part by its name.

        Args:
            part_name (str): The name of the part to retrieve.

        Returns:
            tuple: A tuple containing the part's information.
        """
        self.connect()
        self.cursor.execute("SELECT * FROM Parts WHERE PartName = ?", (part_name,))
        part = self.cursor.fetchone()
        self.disconnect()
        return part

    def print_part_info(self, part):
        """
        Print information about a part.

        Args:
            part (tuple): A tuple containing the part's information.
        """
        if part:
            print("Part found:")
            print("PartID:", part[0])
            print("PartName:", part[1])
            print("NumberOfParts:", part[2])
            print("LocationX:", part[3])
            print("LocationY:", part[4])
            print("LocationZ:", part[5])
            orientation = np.array([part[6], part[7], part[8]])
            print("Orientation:", orientation)
            print("FullWeight:", part[9])
            print("HalfWeight:", part[10])
            print("EmptyWeight:", part[11])
            print("CurrentWeight:", part[12])  # Adding CurrentWeight
            print("InService:", "Yes" if part[13] else "No")  # Adding InService
        else:
            print("Part not found")

    def generate_random_names(self, num_names):
        """
        Generate random part names using the Faker library.

        Args:
            num_names (int): The number of random names to generate.

        Returns:
            list: A list of random part names.
        """
        fake = Faker()
        random_names = [fake.word() for _ in range(num_names)]
        return random_names

def demo_add_parts(parts_db):
    # Input: List of new part data
    parts_to_add = [
        {
            'PartName': 'Part1',
            'NumberOfParts': 5,
            'LocationX': 0.4,
            'LocationY': 0,
            'LocationZ':  0.4,
            'Orientation': [1.0, 0.0, 0.0],
            'FullWeight': 80.0,
            'HalfWeight': 40.0,
            'EmptyWeight': 8.0,
            'CurrentWeight': 75.0,
            'InService': 1,
        },
        {
            'PartName': 'Part2',
            'NumberOfParts': 3,
            'LocationX': 0,
            'LocationY':  0.4,
            'LocationZ':  0.4,
            'Orientation': [0.0, 1.0, 0.0],
            'FullWeight': 50.0,
            'HalfWeight': 25.0,
            'EmptyWeight': 5.0,
            'CurrentWeight': 45.0,
            'InService': 1,
        },
        {
            'PartName': 'Part3',
            'NumberOfParts': 2,
            'LocationX':  0.4,
            'LocationY':  0.4,
            'LocationZ':  0.4,
            'Orientation': [0.5, 0.5, 0.5],
            'FullWeight': 60.0,
            'HalfWeight': 30.0,
            'EmptyWeight': 6.0,
            'CurrentWeight': 55.0,
            'InService': 0,
        },
        {
            'PartName': 'Part4',
            'NumberOfParts': 4,
            'LocationX':  0.2,
            'LocationY': 0,
            'LocationZ':  0.2,
            'Orientation': [0.0, 0.0, 1.0],
            'FullWeight': 70.0,
            'HalfWeight': 35.0,
            'EmptyWeight': 7.0,
            'CurrentWeight': 60.0,
            'InService': 1,
        },
        {
            'PartName': 'Part5',
            'NumberOfParts': 1,
            'LocationX': 0,
            'LocationY':  0.2,
            'LocationZ':  0.2,
            'Orientation': [1.0, 1.0, 1.0],
            'FullWeight': 90.0,
            'HalfWeight': 45.0,
            'EmptyWeight': 9.0,
            'CurrentWeight': 80.0,
            'InService': 0,
        },
        {
            'PartName': 'Part6',
            'NumberOfParts': 1,
            'LocationX':  0.2,
            'LocationY':  0.4,
            'LocationZ':  0.2,
            'Orientation': [1.0, 1.0, 1.0],
            'FullWeight': 90.0,
            'HalfWeight': 45.0,
            'EmptyWeight': 9.0,
            'CurrentWeight': 30.0,
            'InService': 1,
        },
    ]

    # Add the list of parts to the database
    for part_data in parts_to_add:
        parts_db.add_part(part_data)

def demo_add_random_parts(parts_db, num_parts):
    # Generate random part names
    random_part_names = parts_db.generate_random_names(num_parts)

    # Input: List of random part data for testing
    random_parts_to_add = []

    for i, part_name in enumerate(random_part_names):
        part_data = {
            'PartName': part_name,
            'NumberOfParts': i + 1,
            'LocationX': i * 0.1,
            'LocationY': i * 0.2,
            'LocationZ': i * 0.3,
            'Orientation': [0.1 * i, 0.2 * i, 0.3 * i],
            'FullWeight': 10.0 * i,
            'HalfWeight': 5.0 * i,
            'EmptyWeight': random.uniform(0, i),
            'CurrentWeight': 20*random.uniform(0, i),
            'InService': i % 2,  # Alternate between 0 and 1
        }
        random_parts_to_add.append(part_data)

    # Add the list of random parts to the database
    for part_data in random_parts_to_add:
        parts_db.add_part(part_data)

def main():
    parts_db = PartsDatabase()

    # Create the Parts table
    parts_db.create_parts_table()

    # Input: Number of random parts to generate
    num_random_parts = 100
    demo_add_random_parts(parts_db, num_random_parts)


if __name__ == "__main__":
    main()
    