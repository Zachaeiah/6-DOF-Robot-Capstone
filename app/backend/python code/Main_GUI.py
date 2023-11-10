# Define a custom Tkinter application class
import numpy as np
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import colorchooser
from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *

LARGE_FONT = ("Verdana", 12)

class RobotCart(tk.Tk):
    def __init__(self, *args, **kwargs):
        """
        Initialize the main application.

        Args:
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.
        """
        # Initialize the Tkinter application
        tk.Tk.__init__(self, *args, **kwargs)
        
        # Create a container frame to hold the pages
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)

        # Configure the container's grid layout
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        # Dictionary to store different pages
        self.frames = {}

        # Create and add pages to the application
        for F in (MainUserPage, InventoryPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        # Show the MainUserPage by default
        self.show_frame(MainUserPage)

        # Create a PartsDatabase instance and create the Parts table
        parts_db = PartsDatabase()
        parts_db.create_parts_table()

    def show_frame(self, cont):
        """
        Show the specified frame.

        Args:
            cont: The frame to be displayed.
        """
        frame = self.frames[cont]
        frame.tkraise()

class PageBase(tk.Frame):
    """Base class for pages in the application.

    Args:
        tk (type): Tkinter module.
    """
    def __init__(self, parent, controller, title):
        """Initialize the PageBase.

        Args:
            parent (type): The parent widget.
            controller (type): The main application controller.
            title (type): The title of the page.
        """
        tk.Frame.__init__(self, parent)
        
        # Title label
        label = tk.Label(self, text=title, font=LARGE_FONT)
        label.grid(row=0, column=0, pady=10, padx=10)

        # Content frame
        self.content_frame = tk.Frame(self)
        self.content_frame.grid(row=1, column=0, sticky="nsew")

        # Configure row and column weights to allow expansion
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)

class MainUserPage(PageBase):
    """Page class representing the main user interface for part selection.

    Args:
        PageBase (type): Base class for pages.
    """
    def __init__(self, parent, controller):
        """Initialize the MainUserPage.

        Args:
            parent (type): The parent widget.
            controller (type): The main application controller.
        """
        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, "Part Selection Page")

        # Create a PartsDatabase instance for handling parts data
        self.parts_db = PartsDatabase()

        # List to store selected parts in the cart
        self.cart = []

        # Create a frame to hold the main treeview for available parts
        self.parts_treeview_frame = tk.Frame(self.content_frame)
        self.parts_treeview_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Configure the main treeview for available parts
        self.configure_parts_db_treeview()

        # Create buttons for interaction with available parts
        self.create_buttons()

        # Create a frame to hold the cart treeview
        self.cart_tree_frame = tk.Frame(self.content_frame)
        self.cart_tree_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Create a second treeview for the selected parts (cart)
        self.configure_cart_treeview()

    def configure_parts_db_treeview(self):
        """Configure the Treeview widget for displaying available parts.

        This method sets up the style, structure, and data loading for the Treeview.

        """
        # Set up the style for the Treeview
        style = ttk.Style()
        style.theme_use('default')
        style.configure("treeview", background="#D3D3D3", foreground="black", rowheight=25, fieldbackground="#D3D3D3")
        style.map("treeview", background=[('selected', "#347083")])

        # Create a frame to hold the Treeview
        tree_frame = tk.Frame(self.content_frame)
        tree_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create a vertical scrollbar for the Treeview
        tree_scroll = tk.Scrollbar(tree_frame)
        tree_scroll.pack(side='right', fill='y')

        # Create the Treeview widget with vertical scrollbar
        self.parts_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended")
        self.parts_treeview.pack()

        # Configure the scrollbar to control the Treeview's vertical movement
        tree_scroll.config(command=self.parts_treeview.yview)

        # Define columns and headings for the Treeview
        columns = ["PartName", "Number of Parts", "InService", "CurrentWeight"]
        headings = ["PartName", "Number of Parts", "InService", "CurrentWeight"]

        # Set the columns for the Treeview
        self.parts_treeview['columns'] = columns
        self.parts_treeview.column("#0", width=0, stretch=0)

        # Configure column widths and anchor points
        for col in columns:
            if col in ("PartName", "Number of Parts"):
                self.parts_treeview.column(col, anchor="w", width=140)
            else:
                self.parts_treeview.column(col, anchor="center", width=140)

        # Set column headings and anchor points
        self.parts_treeview.heading("#0", text='', anchor="w")
        for col, heading in zip(columns, headings):
            self.parts_treeview.heading(col, text=heading, anchor="center")

        # Load data into the Treeview
        self.load_data()

    def load_data(self):
        """Load data into the Treeview widget.

        This method retrieves data from the Parts database, configures tags for styling,
        sorts the data alphabetically, and inserts it into the Treeview.

        """
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select all records from the Parts table and order by the second column (index 1)
            self.parts_db.cursor.execute("SELECT * FROM Parts ORDER BY PartName")  # Replace 'column_name' with the actual column name you want to sort by

            # Fetch all the data from the executed query
            data = self.parts_db.cursor.fetchall()

            # Configure tags for alternate row colors and low weight indication
            self.parts_treeview.tag_configure('oddrow', background='white')
            self.parts_treeview.tag_configure('evenrow', background='lightblue')
            self.parts_treeview.tag_configure('lowweight', background='lightcoral')  # Add a tag for low weight

            # Iterate through the retrieved data and insert it into the Treeview
            for idx, record in enumerate(data):
                # Determine the tag for alternate row coloring
                tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
                # Determine the service status for display
                service = "In Service" if record[13] == 1 else " "

                # Check if CurrentWeight is below 5 and set a different tag for low weight
                if record[12] < record[10]:
                    tag = 'lowweight'

                # Insert the data into the Treeview with specified tags
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[2], service, record[12]), tags=(tag,))
        
        # Handle exceptions, print the error, and ensure database disconnection
        except Exception as e:
            print(e)
        
        # Ensure the database is disconnected in the 'finally' block
        finally:
            self.parts_db.disconnect()

    def create_buttons(self):
        """Create buttons for user interaction.

        This method creates buttons for refreshing data, adding parts to the cart, placing an order, and searching data.

        """
        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.content_frame)  # Change 'self' to 'self.content_frame'
        button_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky="nsew")  # Use 'grid' instead of 'pack'

        # Create a button to refresh the data
        refresh_button = tk.Button(button_frame, text="Refresh Data", command=self.load_data, bg='#4CAF50')
        refresh_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a button to search the data
        search_button = tk.Button(button_frame, text="Search Data", command=self.serch_data, bg='#2196F3')
        search_button.grid(row=0, column=1, padx=10, pady=10)

        # Create a button to add selected parts to the cart
        add_to_cart_button = tk.Button(button_frame, text="Add to Cart", command=self.select_part, bg='#FFEB3B')
        add_to_cart_button.grid(row=0, column=2, padx=10, pady=10)

        # Create a button to place an order with selected parts
        place_order_button = tk.Button(button_frame, text="Place Order", command=self.place_order, bg='#FF5722')  # Different color for Place Order
        place_order_button.grid(row=0, column=3, padx=10, pady=10)



    def select_part(self):
        # Method to add selected part to the cart
        selected = self.parts_treeview.focus()
        part_name = self.parts_treeview.item(selected, "values")[0]
        self.cart.append(part_name)

        # Update the cart treeview
        self.update_cart_tree()

    def update_cart_tree(self):
        """Update the cart treeview with the current content of the cart.

        This method clears the existing content in the cart treeview and inserts the current
        parts in the cart for display.

        """
        # Method to update the cart treeview
        self.cart_tree.delete(*self.cart_tree.get_children())

        # Iterate through the parts in the cart and insert them into the cart treeview
        for idx, part in enumerate(self.cart):
            # Determine the tag for alternate row coloring
            tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
            # Insert the part into the cart treeview with specified tag
            self.cart_tree.insert(parent='', index='end', iid=idx, text='', values=(part,), tags=(tag,))

    def place_order(self):
        # Method to handle placing an order
        # Implement the logic to process the order using the selected parts in self.cart
        print("Placing order with selected parts:", self.cart)
        # Clear the cart after placing the order
        self.cart = []
        # Update the cart treeview
        self.update_cart_tree()

    def configure_cart_treeview(self):
        """Configure the Treeview widget for displaying selected parts in the cart.

        This method sets up the style, structure, and appearance of the cart Treeview.

        """
        # Configure treeview style for the cart
        style = ttk.Style()
        style.theme_use('default')
        style.configure("cart.Treeview", background="#D3D3D3", foreground="black", rowheight=25, fieldbackground="#D3D3D3")
        style.map("cart.Treeview", background=[('selected', "#347083")])

        # Configure cart tree scroll
        cart_tree_scroll = tk.Scrollbar(self.cart_tree_frame)
        cart_tree_scroll.pack(side='right', fill='y')

        # Create cart treeview widget
        self.cart_tree = ttk.Treeview(self.cart_tree_frame, yscrollcommand=cart_tree_scroll.set, selectmode="extended", style="cart.Treeview")
        self.cart_tree.pack(fill='both', expand=True)

        # Configure scrollbar to control cart treeview's vertical movement
        cart_tree_scroll.config(command=self.cart_tree.yview)

        # Columns and headings for the cart treeview
        cart_columns = ["Selected Part"]
        cart_headings = ["Selected Part"]

        # Set columns for the cart treeview
        self.cart_tree['columns'] = cart_columns

        # Configure column widths and anchor points
        for col in cart_columns:
            self.cart_tree.column(col, anchor="w", width=140)

        # Set column headings and anchor points
        for col, heading in zip(cart_columns, cart_headings):
            self.cart_tree.heading(col, text=heading, anchor="center")

    def serch_data(self):
        serch = tk.Toplevel(self)
        serch.title("Lookup parts")
        serch.geometry("400x200")

        # Create a reference to the search window to use in the closing event
        self.search_window = serch

        serch_frame = tk.LabelFrame(serch, text="Part Name")
        serch_frame.pack(padx=10, pady=10)

        Serch_entry = tk.Entry(serch_frame)
        Serch_entry.pack(padx=20, pady=20)

        # Pass a lambda function to the command parameter
        Serch_button = tk.Button(serch, text="Search Part", command=lambda: self.search_and_update_treeview(Serch_entry.get()))
        Serch_button.pack(padx=20, pady=20)

        # Bind the closing event of the search window to the restore_treeview method
        serch.protocol("WM_DELETE_WINDOW", self.restore_treeview)

    def search_and_update_treeview(self, part_name):
        """Search for parts and update the parts_treeview based on the search."""
        self.serch_db_parts(part_name)

    def restore_treeview(self):
        """Restore the parts_treeview to show all parts."""
        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())
        
        # Load all parts into the parts_treeview
        self.load_data()

        # Destroy the search window
        self.search_window.destroy()

    def serch_db_parts(self, partName):
        """Search for parts in the database with the given name and update the parts_treeview.

        Args:
            partName (str): The name of the part to search for.
        """
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select records from the Parts table with a matching part name
            query = "SELECT * FROM Parts WHERE PartName LIKE ?"
            self.parts_db.cursor.execute(query, ('%' + partName + '%',))

            # Fetch all the data from the executed query
            data = self.parts_db.cursor.fetchall()

            # Clear existing content in the parts_treeview
            self.parts_treeview.delete(*self.parts_treeview.get_children())

            # Configure tags for alternate row colors and low weight indication
            self.parts_treeview.tag_configure('oddrow', background='white')
            self.parts_treeview.tag_configure('evenrow', background='lightblue')
            self.parts_treeview.tag_configure('lowweight', background='lightcoral')  # Add a tag for low weight

            # Iterate through the retrieved data and insert it into the parts_treeview
            for idx, record in enumerate(data):
                # Determine the tag for alternate row coloring
                tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
                # Determine the service status for display
                service = "In Service" if record[13] == 1 else " "

                # Check if CurrentWeight is below 5 and set a different tag for low weight
                if record[12] < record[10]:
                    tag = 'lowweight'

                # Insert the data into the parts_treeview with specified tags
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[2], service, record[12]), tags=(tag,))

        # Handle exceptions, print the error, and ensure database disconnection
        except Exception as e:
            print(e)

        # Ensure the database is disconnected in the 'finally' block
        finally:
            self.parts_db.disconnect()

# Page One class
class InventoryPage(PageBase):
    def __init__(self, parent, controller):

        """
        Initialize Page One.

        Args:
            parent (tk.Frame): The parent frame.
            controller (tk.Tk): The main application controller.
        """
        super().__init__(parent, controller, "InventoryPage")

if __name__ == "__main__":
    """Entry point of the script."""

    app = RobotCart()
    app.mainloop()