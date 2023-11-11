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
        for F in (MainUserPage, MotorSetUpPage):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        # Show the working page by default
        self.show_frame(MotorSetUpPage)

        # Create a PartsDatabase instance and create the Parts table
        parts_db = PartsDatabase()
        parts_db.create_parts_table()

        menuBar = tk.Menu(container)
        settingMenu = tk.Menu(menuBar, tearoff=0)
        settingMenu.add_command(label='Motor Set Up ', command=lambda: self.show_frame(MotorSetUpPage))
        settingMenu.add_separator()
        settingMenu.add_command(label='Primary Color', command=self.primary_color)
        settingMenu.add_command(label='Secondary Color', command=self.secondary_color)
        settingMenu.add_command(label='Highlight Color', command=self.highlight_color)
        settingMenu.add_command(label='Low Weight', command = self.low_weight_color)
        menuBar.add_cascade(label="Settings", menu=settingMenu)


        tk.Tk.config(self, menu = menuBar)

    def show_frame(self, cont):
        """
        Show the specified frame.

        Args:
            cont: The frame to be displayed.
        """
        frame = self.frames[cont]
        frame.tkraise()

    def popupmsg(self, msg):
        popup = tk.Tk()

        def leavemini():
            popup.destroy()

        popup.wm_title("!")
        lable = ttk.Label(popup, text = msg)
        lable.pack(side = "top", fill="x", pady=10)
        B1 = ttk.Button(popup, text = "Okay", command = leavemini)
        B1.pack(side = "bottom", fill="x", pady=10)

    def primary_color(self):
        primary_color = colorchooser.askcolor()[1]

        if primary_color:
            # Access the current frame and update the tag configuration
            current_frame = self.frames[MainUserPage]
            current_frame.parts_treeview.tag_configure('oddrow', background=primary_color)

    def secondary_color(self):
        secondary_color = colorchooser.askcolor()[1]

        if secondary_color:
            # Access the current frame and update the tag configuration
            current_frame = self.frames[MainUserPage]
            current_frame.parts_treeview.tag_configure('evenrow', background=secondary_color)

    def low_weight_color(self):
        low_weight_color = colorchooser.askcolor()[1]

        if low_weight_color:
            # Access the current frame and update the tag configuration
            current_frame = self.frames[MainUserPage]
            current_frame.parts_treeview.tag_configure('lowweight', background=low_weight_color)

    def highlight_color(self):
        highlight_color = colorchooser.askcolor()[1]

        if highlight_color:
            # Access the current frame and update the tag configuration
            current_frame = self.frames[MainUserPage]

            # Access the style of the current frame's parts_treeview
            style = ttk.Style()

            # Update the map for the 'Treeview' style
            style.map("Treeview", background=[('selected', highlight_color)])



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
        self.create_buttons(controller)

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
        columns = ["Part Name", "Number of Parts", "In Service", "Current Weight"]
        headings = ["PartName", "Number of Parts", "In Service", "Current Weight"]

        # Set the columns for the Treeview
        self.parts_treeview['columns'] = columns
        self.parts_treeview.column("#0", width=0, stretch=0)

        # Configure column widths and anchor points
        for col in columns:
            if col in ("Part Name", "Number of Parts"):
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

    def create_buttons(self, controller):
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
        place_order_button = tk.Button(button_frame, text="Place Order", command=lambda: self.place_order(controller), bg='#FF5722')  # Different color for Place Order
        place_order_button.grid(row=0, column=3, padx=10, pady=10)

    def select_part(self):
        print("Selecting part...")  # Add this line for debugging

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
        print("Updating cart tree...")
        
        # Clear existing content in the cart treeview
        self.cart_tree.delete(*self.cart_tree.get_children())

        # Iterate through the parts in the cart and insert them into the cart treeview
        for idx, part in enumerate(self.cart):
            print(f"Inserting part {part} into cart tree...")
            # Determine the tag for alternate row coloring
            tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
            # Insert the part into the cart treeview with specified tag
            self.cart_tree.insert(parent='', index='end', iid=idx, text='', values=(part,), tags=(tag,))

    def place_order(self, controller):
        # Method to handle placing an order
        # Implement the logic to process the order using the selected parts in self.cart
        print("Placing order with selected parts:", self.cart)
        # Clear the cart after placing the order
        self.cart = []
        # Update the cart treeview
        self.update_cart_tree()
        controller.show_frame(MotorSetUpPage)

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

        # Set columns for the cart treeview
        self.cart_tree['columns'] = ("#0",)

        # Configure column widths and anchor points (optional if you decide to keep it)
        self.cart_tree.column("#0", anchor="w", width=140)

        # Set column headings and anchor points (optional if you decide to keep it)
        self.cart_tree.heading("#0", text="Selected Part", anchor="center")  # Change "#0" to the actual column ID if necessary

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
        Serch_entry.bind('<KeyRelease>', lambda event, entry=Serch_entry: self.auto_fill_suggestions(entry))

        # Pass a lambda function to the command parameter
        Serch_button = tk.Button(serch, text="Search Part", command=lambda: self.search_and_update_treeview(Serch_entry.get()))
        Serch_button.pack(padx=20, pady=20)

        # Bind the closing event of the search window to the restore_treeview method
        serch.protocol("WM_DELETE_WINDOW", self.restore_treeview)

    def auto_fill_suggestions(self, entry):
        """Auto-fill suggestions based on the current content of the entry."""
        # Get the current content of the entry
        current_text = entry.get()

        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())

        # Load suggestions based on the current_text
        self.load_suggestions(current_text)

    def load_suggestions(self, prefix):
        """Load suggestions into the parts_treeview based on the given prefix."""
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select records from the Parts table with a matching part name
            query = "SELECT * FROM Parts WHERE PartName LIKE ?"
            self.parts_db.cursor.execute(query, ('%' + prefix + '%',))

            # Fetch all the data from the executed query
            data = self.parts_db.cursor.fetchall()

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

# Set up page class
class MotorSetUpPage(PageBase):
    def __init__(self, parent, controller):

        """
        Initialize Page One.

        Args:
            parent (tk.Frame): The parent frame.
            controller (tk.Tk): The main application controller.
        """
        super().__init__(parent, controller, "Motor Set Up Page ")

        self.manager = motorManager({})

        # Configure the main treeview for available parts
        self.configure_Motor_treeview()

        self.create_buttons(controller)

    def configure_Motor_treeview(self):
        """Configure the Treeview widget for displaying Motor Data.

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
        self.motor_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended")
        self.motor_treeview.pack()

        # Configure the scrollbar to control the Treeview's vertical movement
        tree_scroll.config(command=self.motor_treeview.yview)

        # Define columns and headings for the Treeview
        columns = ["Motor Name", "index", "max_speed", "max_acceleration", "max_torqu", "is_activate", "type", "steps_per_revolution"]
        headings = ["Motor Name", "index", "max_speed", "max_acceleration", "max_torqu", "is_activate", "type", "steps_per_revolution"]

        # Set the columns for the Treeview
        self.motor_treeview['columns'] = columns
        self.motor_treeview.column("#0", width=0, stretch=0)

        # Configure column widths and anchor points
        for col in columns:
            if col in ("Motor Name", "index"):
                self.motor_treeview.column(col, anchor="w", width=140)
            else:
                self.motor_treeview.column(col, anchor="center", width=140)

        # Set column headings and anchor points
        self.motor_treeview.heading("#0", text='', anchor="w")
        for col, heading in zip(columns, headings):
            self.motor_treeview.heading(col, text=heading, anchor="center")

        # Load data into the Treeview
        self.load_data()

    def load_data(self):
        # Read motor configurations from the JSON file
        self.manager.read_motor_config("motors_config.json")

        # Clear existing items in the Treeview
        for item in self.motor_treeview.get_children():
            self.motor_treeview.delete(item)

        # Iterate over motors and insert them into the Treeview
        for motor_name, motor in self.manager.motors.items():
            values = (
                motor.name,
                motor.index,
                motor.max_speed,
                motor.max_acceleration,
                motor.max_torqu,
                motor.is_activate,
                motor.__class__.__name__,  # Assuming the motor class name is desired for 'type'
                motor.steps_per_revolution
            )
            self.motor_treeview.insert("", "end", text="", values=values)

    def create_buttons(self, controller):
        """Create buttons for user interaction.

        This method creates buttons for refreshing data, adding parts to the cart, placing an order, and searching data.

        """
        # Create a frame to hold the buttons
        button_frame = tk.LabelFrame(self.content_frame, text="Commands")  # Change 'self' to 'self.content_frame'
        button_frame.grid(row=1, column=0, columnspan=2, pady=10, sticky="nsew")  # Use 'grid' instead of 'pack'

        select_motor_button = tk.Button(button_frame, text="Select motor")  
        select_motor_button.grid(row=0, column=1, padx=10, pady=10)

        edit_motor_button = tk.Button(button_frame, text="Select motor")  
        edit_motor_button.grid(row=0, column=1, padx=10, pady=10)

        activate_button = tk.Button(button_frame, text="activate Motor")
        activate_button.grid(row=0, column=2, padx=10, pady=10)

        deactivate_button = tk.Button(button_frame, text="deactivate Motor")
        deactivate_button.grid(row=0, column=3, padx=10, pady=10)

if __name__ == "__main__":
    """Entry point of the script."""

    app = RobotCart()
    app.mainloop()