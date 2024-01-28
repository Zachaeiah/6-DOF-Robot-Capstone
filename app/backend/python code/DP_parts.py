import sqlite3
import numpy as np
from faker import Faker
import random

class PartsDatabase:
    def __init__(self, db_name="parts.db"):
        self.db_name = db_name
        self.conn = None
        self.cursor = None
        self.fake = Faker()

    def connect(self):
        try:
            self.conn = sqlite3.connect(self.db_name)  # Replace with your actual database name
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
        return part

    def print_part_info(self, part):
        if part:
            print("Part found:")
            for i, column_name in enumerate(['PartID', 'PartName', 'NumberOfParts', 'LocationX', 'LocationY', 'LocationZ', 'OrientationX', 'OrientationY', 'OrientationZ', 'FullWeight', 'HalfWeight', 'EmptyWeight', 'CurrentWeight', 'InService']):
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
            'PartID': 0,
            'PartName': 'Part0',
            'NumberOfParts': 5,
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
            'PartID': 1,
            'PartName': 'Part1',
            'NumberOfParts': 3,
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
            'PartID': 2,
            'PartName': 'Part2',
            'NumberOfParts': 2,
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
            'PartID': 3,
            'PartName': 'Part3',
            'NumberOfParts': 4,
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
            'PartID': 4,
            'PartName': 'Part4',
            'NumberOfParts': 5,
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
            'PartID': 5,
            'PartName': 'Part5',
            'NumberOfParts': 3,
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
            'PartID': 6,
            'PartName': 'Part6',
            'NumberOfParts': 2,
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
            'PartID': 7,
            'PartName': 'Part7',
            'NumberOfParts': 4,
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

def demo_add_random_parts(parts_db, num_parts):

    # Input: List of random part data for testing
    random_parts_to_add = []

    for i in range(6, num_parts, 1):
        part_data = {
            'PartID': i,
            'PartName': f"Part: {i}",
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

    demo_add_parts(parts_db)


if __name__ == "__main__":
    main()
    