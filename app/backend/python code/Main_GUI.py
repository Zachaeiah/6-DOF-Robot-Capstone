# Define a custom Tkinter application class
import numpy as np
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import colorchooser
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from DP_parts import *
from ik_solver import *
from intrerpolation import *
from MotorManager import *
from Motors import *
from VelocityPFP import *

LARGE_FONT = ("Verdana", 12)

class RobotCart(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        self.logged_in = True
        self.admin_page = 0

        # Create a container frame to hold the pages
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)

        # Configure the container's grid layout
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        # Dictionary to store different pages
        self.frames = {}

        # Create and add pages to the application
        for F in (MainUserPage, SecurityPage, MotorSetUpPage, MoshionPlanningPage, DataBacePannle):
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        # Show the working page by default
        self.show_frame(MainUserPage)

        # Create a PartsDatabase instance and create the Parts table
        parts_db = PartsDatabase()
        parts_db.create_parts_table()

        menuBar = tk.Menu(container)
        settingMenu = tk.Menu(menuBar, tearoff=0)
        settingMenu.add_command(label='Motor Set Up', command = lambda: self.admin_controll(0))
        settingMenu.add_command(label='Data Bace Config', command = lambda: self.admin_controll(1))
        settingMenu.add_separator()
        settingMenu.add_command(label='Primary Color', command=self.primary_color)
        settingMenu.add_command(label='Secondary Color', command=self.secondary_color)
        settingMenu.add_command(label='Highlight Color', command=self.highlight_color)
        settingMenu.add_command(label='Low Weight', command=self.low_weight_color)
        menuBar.add_cascade(label="Menu", menu=settingMenu)

        tk.Tk.config(self, menu=menuBar)

        # Configure the style for Treeview widgets
        style = ttk.Style()
        style.theme_use('default')

        # Specify your tag configurations here
        style.configure("OddRow.TTreeview", background='white')
        style.configure("EvenRow.TTreeview", background='lightblue')
        style.configure("LowWeight.TTreeview", background='lightcoral')
        # Add more tag configurations as needed

        tk.Tk.config(self, menu = menuBar)

    def show_frame(self, cont: tk.Frame, cart_data=None) -> None:
        """
        Show the specified frame.

        Args:
            cont: The frame to be displayed.
            cart_data: Additional data to pass to the frame.
        """
        frame = self.frames[cont]

        # Pass the cart_data to the frame
        if hasattr(frame, 'set_cart_data'):
            frame.set_cart_data(cart_data)

        frame.tkraise()
    
    def admin_controll(self, pageNum) -> None:
        self.admin_page = pageNum

        # Check if the user is logged in
        if not self.logged_in:
            # If not logged in, show the SecurityPage
            self.show_frame(SecurityPage)
        else:
            if self.admin_page == 0:
                self.show_frame(MotorSetUpPage)

            elif self.admin_page == 1:
                self.show_frame(DataBacePannle)

    def popupmsg(self, msg: str) -> None:
        popup = tk.Tk()

        def leavemini():
            popup.destroy()

        popup.wm_title("!")
        lable = ttk.Label(popup, text = msg)
        lable.pack(side = "top", fill="x", pady=10)
        B1 = ttk.Button(popup, text = "Okay", command = leavemini)
        B1.pack(side = "bottom", fill="x", pady=10)

    def primary_color(self) -> None:
        primary_color = colorchooser.askcolor()[1]
        if primary_color:
            current_frame = self.frames[MainUserPage]
            current_frame.parts_treeview.tag_configure('OddRow.TTreeview', background=primary_color)

    def secondary_color(self) -> None:
        secondary_color = colorchooser.askcolor()[1]
        if secondary_color:
            current_frame = self.frames[MainUserPage]
            current_frame.parts_treeview.tag_configure('EvenRow.TTreeview', background=secondary_color)

    def low_weight_color(self) -> None:
        low_weight_color = colorchooser.askcolor()[1]
        if low_weight_color:
            current_frame = self.frames[MainUserPage]
            current_frame.parts_treeview.tag_configure('LowWeight.TTreeview', background=low_weight_color)

    def highlight_color(self) -> None:
        highlight_color = colorchooser.askcolor()[1]
        if highlight_color:
            current_frame = self.frames[MainUserPage]
            style = ttk.Style()
            style.map("OddRow.TTreeview", background=[('selected', highlight_color)])
            style.map("EvenRow.TTreeview", background=[('selected', highlight_color)])

class PageBase(tk.Frame):
    def __init__(self, parent: tk.Widget, controller: RobotCart, title: str):
        super().__init__(parent)

        # Title label
        label = tk.Label(self, text=title, font=LARGE_FONT)
        label.grid(row=0, column=0, pady=10, padx=10)

        # Content frame
        self.content_frame = tk.Frame(self)
        self.content_frame.grid(row=1, column=0, sticky="nsew")

        # Configure row and column weights to allow expansion
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)

class SecurityPage(PageBase):
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        super().__init__(parent, controller, "Security")

        self.username_label = tk.Label(self.content_frame, text="Username")
        self.username_label.grid(row=0, column=0, padx=10, pady=10)
        self.username_entry = tk.Entry(self.content_frame)
        self.username_entry.grid(row=0, column=1, padx=10, pady=10)

        self.password_label = tk.Label(self.content_frame, text="Password")
        self.password_label.grid(row=1, column=0, padx=10, pady=10)
        self.password_entry = tk.Entry(self.content_frame, show="*")  # Use show="*" to hide the entered characters
        self.password_entry.grid(row=1, column=1, padx=10, pady=10)
        
        self.login_button = tk.Button(self.content_frame, text="Login", command=lambda: self.check_credentials(controller))
        self.login_button.grid(row=2, column=0, columnspan=2, pady=10)

    def check_credentials(self, controller):
        # For simplicity, using hardcoded username and password
        correct_username = "admin"
        correct_password = "admin123"

        entered_username = self.username_entry.get()
        entered_password = self.password_entry.get()

        if entered_username == correct_username and entered_password == correct_password:
            # If the credentials are correct, set the login status to True
            controller.logged_in = True

            if controller.admin_page == 0:
                controller.show_frame(MotorSetUpPage)

            elif controller.admin_page == 1:
                controller.show_frame(DataBacePannle)
        else:
            # If the credentials are incorrect, show a message
            messagebox.showerror("Login Failed", "Incorrect username or password")

class MainUserPage(PageBase):
    def __init__(self, parent: tk.Widget, controller: RobotCart):
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
        self.cart: list[str] = []

        # Create a frame to hold the main treeview for available parts
        self.parts_treeview_frame = tk.Frame(self.content_frame)
        self.parts_treeview_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create a frame to hold the cart treeview
        self.cart_tree_frame = tk.Frame(self.content_frame)
        self.cart_tree_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Configure the main treeview for available parts
        self.configure_parts_db_treeview()

        # Create buttons for interaction with available parts
        self.create_buttons(controller)

        # Create a second treeview for the selected parts (cart)
        self.configure_cart_treeview()

    def configure_parts_db_treeview(self) -> None:
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
        self.parts_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended", height=20)
        self.parts_treeview.pack()

        # Configure the scrollbar to control the Treeview's vertical movement
        tree_scroll.config(command=self.parts_treeview.yview)

        # Define columns and headings for the Treeview
        columns = ["Part Name", "Part ID", "In Service", "Current Weight"]
        headings = ["PartName", "Part ID", "In Service", "Current Weight"]

        # Set the columns for the Treeview
        self.parts_treeview['columns'] = columns
        self.parts_treeview.column("#0", width=0, stretch=0)

        # Configure column widths and anchor points
        for col in columns:
            if col in ("Part Name", "Part ID"):
                self.parts_treeview.column(col, anchor="w", width=140)
            else:
                self.parts_treeview.column(col, anchor="center", width=140)

        # Set column headings and anchor points
        self.parts_treeview.heading("#0", text='', anchor="w")
        for col, heading in zip(columns, headings):
            self.parts_treeview.heading(col, text=heading, anchor="center")

        # Load data into the Treeview
        self.load_data()

    def load_data(self) -> None:
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
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[0], service, record[12]), tags=(tag,))
        
        # Handle exceptions, print the error, and ensure database disconnection
        except Exception as e:
            print(e)
        
        # Ensure the database is disconnected in the 'finally' block
        finally:
            self.parts_db.disconnect()

    def create_buttons(self, controller: RobotCart) -> None:
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

    def select_part(self) -> None:
        print("Selecting part...")  # Add this line for debugging

        # Method to add selected part to the cart
        selected = self.parts_treeview.focus()
        part_name = self.parts_treeview.item(selected, "values")[0]
        self.cart.append(part_name)

        # Update the cart treeview
        self.update_cart_tree()

    def update_cart_tree(self)-> None:
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

    def place_order(self, controller: RobotCart) -> None:
        # Implement the logic to process the order using the selected parts in self.cart
        print("Placing order with selected parts:", self.cart)
        controller.show_frame(MoshionPlanningPage, cart_data=self.cart)
        # Clear the cart after placing the order
        self.cart = []
        # Update the cart treeview
        self.update_cart_tree()

    def configure_cart_treeview(self) -> None:
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

    def serch_data(self) -> None:
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

        # # Pass a lambda function to the command parameter
        # Serch_button = tk.Button(serch, text="Search Part", command=lambda: self.search_and_update_treeview(Serch_entry.get()))
        # Serch_button.pack(padx=20, pady=20)

        # Bind the closing event of the search window to the restore_treeview method
        serch.protocol("WM_DELETE_WINDOW", self.restore_treeview)

    def auto_fill_suggestions(self, entry: tk.Widget) -> None:
        """Auto-fill suggestions based on the current content of the entry."""
        # Get the current content of the entry
        current_text = entry.get()

        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())

        # Load suggestions based on the current_text
        self.load_suggestions(current_text)

    def load_suggestions(self, prefix: str) -> None:
        """Load suggestions into the parts_treeview based on the given prefix."""
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select records from the Parts table with a matching part name or part ID
            query = "SELECT * FROM Parts WHERE PartName LIKE ? OR PartID LIKE ?"
            self.parts_db.cursor.execute(query, ('%' + prefix + '%', '%' + prefix + '%'))

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

    def search_and_update_treeview(self, part_name: str) -> None:
        """Search for parts and update the parts_treeview based on the search."""
        self.serch_db_parts(part_name)

    def restore_treeview(self) -> None:
        """Restore the parts_treeview to show all parts."""
        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())
        
        # Load all parts into the parts_treeview
        self.load_data()

        # Destroy the search window
        self.search_window.destroy()

    def serch_db_parts(self, partName: str) -> None:
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

class MotorSetUpPage(PageBase):
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        super().__init__(parent, controller, "Motor Set Up Page ")

        self.manager = motorManager({})

        self.configure_Motor_treeview()
        self.motor_treeview.bind("<ButtonRelease-1>", lambda event: self.secect_motor())

        # Create Entry boxes after configuring the treeview
        self.motor_entry_boxes()

        self.create_buttons(controller)

    def configure_Motor_treeview(self) -> None:
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
        tree_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # Create a vertical scrollbar for the Treeview
        tree_scroll = tk.Scrollbar(tree_frame)
        tree_scroll.pack(side='right', fill='y')

        # Create the Treeview widget with vertical scrollbar
        self.motor_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended", height=6)
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

    def load_data(self) -> None:
        """_summary_
        """

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

    def motor_entry_boxes(self) -> None:
        """Create entry boxes for motor data."""
        # Create a frame to hold the entry boxes
        self.entry_frame = tk.LabelFrame(self.content_frame, text="Motor Data")
        self.entry_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky="nsew")

        # Define the labels and corresponding entry variables
        labels = ["Name", "Speed", "Acceleration", "Torque", "Is Activate", "Steps per Revolution"]
        entry_vars = [tk.StringVar() for _ in range(len(labels))]

        # Loop to create labels and entry boxes
        for i, label in enumerate(labels):
            tk.Label(self.entry_frame, text=label).grid(row=0, column=i, padx=10, pady=10)
            tk.Entry(self.entry_frame, textvariable=entry_vars[i]).grid(row=1, column=i, padx=10, pady=10)

        # Assign entry variables to class attributes for later access
        self.mn_var, self.ms_var, self.ma_var, self.mt_var, self.mact_var, self.mspr_var = entry_vars

    def create_buttons(self, controller: RobotCart) -> None:
        """Create buttons for MotorSetUpPage.

        Args:
            controller (RobotCart): The main controller for the application.
        """
        # Button to save and edit motor data
        edit_motor_button = tk.Button(self.entry_frame, text="Save Edit", command=self.update_motor_data)
        edit_motor_button.grid(row=1, column=6, padx=10, pady=10)

        # Button to return to Part Selection
        return_button = tk.Button(self.content_frame, text="Part Selection", command=lambda: controller.show_frame(MainUserPage))
        return_button.grid(row=0, column=0, padx=10, pady=10, sticky="nw")

    def secect_motor(self) -> None:
        # Method to add selected part to the cart
        selected = self.motor_treeview.focus()
        motor_data = self.motor_treeview.item(selected, "values")

        # Use set method to update StringVar values
        self.mn_var.set(motor_data[0])
        self.ms_var.set(motor_data[2])
        self.ma_var.set(motor_data[3])
        self.mt_var.set(motor_data[4])
        self.mact_var.set(motor_data[5])
        self.mspr_var.set(motor_data[7])
    
    def update_motor_data(self) -> None:
        confirmation = messagebox.askokcancel("Confirmation", "Are you sure you want to update the motor data?")

        if confirmation:
            try:
                # Get the values from the StringVar variables
                motor_name = self.mn_var.get()
                max_speed = float(self.ms_var.get())
                max_acceleration = float(self.ma_var.get())
                max_torqu = float(self.mt_var.get())
                is_activate = bool(self.mact_var.get())  # Assuming 'is_activate' is a boolean
                steps_per_revolution = int(self.mspr_var.get())
                print("got motor data, now editing")

                # Validate input values
                if max_speed < 0 or max_acceleration < 0 or max_torqu < 0 or steps_per_revolution <= 0:
                    raise ValueError("Numeric values must be non-negative, and steps per revolution must be positive.")

                # Update the selected motor with the new values
                if self.manager.edit_motor(
                    motor_name=motor_name,
                    max_speed=max_speed,
                    max_acceleration=max_acceleration,
                    max_torqu=max_torqu,
                    is_activate=is_activate,
                    steps_per_revolution=steps_per_revolution):

                    # Provide feedback to the user
                    messagebox.showinfo("Success", "Motor data updated successfully.")

                    print("loading new data")
                    # Reload data into the Treeview to reflect the changes
                    self.load_data()

            except ValueError as e:
                # Handle invalid input (e.g., non-numeric values in numeric fields)
                messagebox.showerror("Error", f"Invalid input: {str(e)}. Please enter valid numeric values.")
            except Exception as e:
                # Handle other unexpected errors
                messagebox.showerror("Error", f"An unexpected error occurred: {str(e)}")

class DataBacePannle(PageBase):
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the DataBacePannle.

        Args:
            parent (type): The parent widget.
            controller (type): The main application controller.
        """
        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, "Part Selection Page")

        # Create a PartsDatabase instance for handling parts data
        self.parts_db = PartsDatabase()


        # Create a frame to hold the main treeview for available parts
        self.parts_treeview_frame = tk.Frame(self.content_frame)
        self.parts_treeview_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")


        # Configure the main treeview for available parts
        self.configure_parts_db_treeview()

        self.parts_treeview.bind("<ButtonRelease-1>", lambda event: self.select_part())

        # Create buttons for interaction with available parts
        self.create_buttons(controller)

        self.parts_entry_boxes()

    def configure_parts_db_treeview(self) -> None:
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
        self.parts_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended", height=20)
        self.parts_treeview.pack()

        # Configure the scrollbar to control the Treeview's vertical movement
        tree_scroll.config(command=self.parts_treeview.yview)

        # Define columns and headings for the Treeview
        columns = ["Part Name", "Part ID", "In Service", "Current Weight"]
        headings = ["PartName", "Part ID", "In Service", "Current Weight"]

        # Set the columns for the Treeview
        self.parts_treeview['columns'] = columns
        self.parts_treeview.column("#0", width=0, stretch=0)

        # Configure column widths and anchor points
        for col in columns:
            if col in ("Part Name", "Part ID"):
                self.parts_treeview.column(col, anchor="w", width=140)
            else:
                self.parts_treeview.column(col, anchor="center", width=140)

        # Set column headings and anchor points
        self.parts_treeview.heading("#0", text='', anchor="w")
        for col, heading in zip(columns, headings):
            self.parts_treeview.heading(col, text=heading, anchor="center")

        # Load data into the Treeview
        self.load_data()

    def load_data(self) -> None:
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
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[0], service, record[12]), tags=(tag,))
        
        # Handle exceptions, print the error, and ensure database disconnection
        except Exception as e:
            print(e)
        
        # Ensure the database is disconnected in the 'finally' block
        finally:
            self.parts_db.disconnect()

    def create_buttons(self, controller: RobotCart) -> None:
        """Create buttons for user interaction.

        This method creates buttons for refreshing data, adding parts to the cart, placing an order, and searching data.

        """
        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.content_frame)  # Change 'self' to 'self.content_frame'
        button_frame.grid(row=3, column=0, columnspan=2, pady=10, sticky="nsew")  # Use 'grid' instead of 'pack'

        # Create a button to refresh the data
        refresh_button = tk.Button(button_frame, text="Refresh Data", command=self.load_data, bg='#4CAF50')
        refresh_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a button to search the data
        search_button = tk.Button(button_frame, text="Search Data", command=self.serch_data, bg='#2196F3')
        search_button.grid(row=0, column=1, padx=10, pady=10)

        # Button to save and edit motor data
        edit_motor_button = tk.Button(button_frame, text="Save Edit", command = self.update_part_data)
        edit_motor_button.grid(row=0, column=2, padx=10, pady=10)

    def parts_entry_boxes(self) -> None:
        """Create entry boxes for paty data editing."""
        # Create a frame to hold the entry boxes
        self.entry_frame = tk.LabelFrame(self.content_frame, text="Part Data")
        self.entry_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky="nsew")

        # Define the labels and corresponding entry variables

        labels = ["Part ID",          "Part Name",      "Number Of Parts", 
                  "Location: X",      "Location: Y",    "Location: Z", 
                  "Orientation: X",   "Orientation: Y", "Orientation: Z", 
                  "Full Weight",      "Half Weight",    "Empty Weight",  
                  "CurrentWeight",    "In Service"]
        
        entry_vars = [tk.StringVar() for _ in range(len(labels))]

        # Loop to create labels and entry boxes
        row, col = (0,0)
        for i, label in enumerate(labels):
            
            if ((i % 7) == 0):
                row += 2
                col = 0
            else:
                col += 1

            tk.Label(self.entry_frame, text=label).grid(row = row, column=col, padx=10, pady=10)
            tk.Entry(self.entry_frame, textvariable=entry_vars[i]).grid(row= row + 1, column=col, padx=10, pady=10)

        # Assign entry variables to class attributes for later access
        self.PartID, self.PartName, self.NumberOfParts, self.LocationX, self.LocationY, self.LocationZ, self.OrientationX, self.OrientationY, self.OrientationZ, self.FullWeight, self.HalfWeight, self.EmptyWeight, self.CurrentWeight, self.InService = entry_vars
    
    def select_part(self) -> None:
        print("Selecting part...")  # Add this line for debugging

        # Method to add selected part to the cart
        selected = self.parts_treeview.focus()
        part_name = self.parts_treeview.item(selected, "values")[0]
        part_data = self.parts_db.get_part_by_name(part_name)
        print(part_data)

        # Use set method to update StringVar values
        self.PartID.set(part_data[0])
        self.PartName.set(part_data[1])
        self.NumberOfParts.set(part_data[2])
        self.LocationX.set(part_data[3])
        self.LocationY.set(part_data[4])
        self.LocationZ.set(part_data[5])
        self.OrientationX.set(part_data[6])
        self.OrientationY.set(part_data[7])
        self.OrientationZ.set(part_data[8])
        self.FullWeight.set(part_data[9])
        self.HalfWeight.set(part_data[10])
        self.EmptyWeight.set(part_data[1])
        self.CurrentWeight.set(part_data[12])
        self.InService.set(part_data[13])       

    def update_part_data(self) -> None:
        confirmation = messagebox.askokcancel("Confirmation", "Are you sure you want to update the Part data?")
        if confirmation:
            try:
                new_part_data = {
                'PartID': self.PartID.get(),
                'PartName': self.PartName.get(),
                'NumberOfParts':self.NumberOfParts.get(),
                'LocationX': self.LocationX.get(),
                'LocationY': self.LocationY.get(),
                'LocationZ': self.LocationZ.get(),
                'Orientation': [self.OrientationX.get(), self.OrientationY.get(), self.OrientationZ.get()],
                'FullWeight': self.FullWeight.get(),
                'HalfWeight': self.HalfWeight.get(),
                'EmptyWeight': self.EmptyWeight.get(),
                'CurrentWeight': self.CurrentWeight.get(),
                'InService': self.InService.get(),  
                }

                # Establish a connection to the Parts database
                self.parts_db.connect()
                
                self.parts_db.edit_part(self.PartName.get(), new_part_data)

                # Provide feedback to the user
                messagebox.showinfo("Success", "Part data updated successfully.")

                # Clear existing content in the parts_treeview
                self.parts_treeview.delete(*self.parts_treeview.get_children())
                
                # Load all parts into the parts_treeview
                self.load_data()

            except ValueError:
                # Handle invalid input (e.g., non-numeric values in numeric fields)
                messagebox.showerror("Error", "Invalid input. Please enter valid numeric values.")

            except Exception as e:
                print(e)

            finally:
                # Disconnect from the database only when you are done with all operations
                self.parts_db.disconnect()

    def serch_data(self) -> None:
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

        # Bind the closing event of the search window to the restore_treeview method
        serch.protocol("WM_DELETE_WINDOW", self.restore_treeview)

    def auto_fill_suggestions(self, entry: tk.Widget) -> None:
        """Auto-fill suggestions based on the current content of the entry."""
        # Get the current content of the entry
        current_text = entry.get()

        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())

        # Load suggestions based on the current_text
        self.load_suggestions(current_text)

    def load_suggestions(self, prefix: str) -> None:
        """Load suggestions into the parts_treeview based on the given prefix."""
        try:
            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select records from the Parts table with a matching part name or part ID
            query = "SELECT * FROM Parts WHERE PartName LIKE ? OR PartID LIKE ?"
            self.parts_db.cursor.execute(query, ('%' + prefix + '%', '%' + prefix + '%'))

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

    def search_and_update_treeview(self, part_name: str) -> None:
        """Search for parts and update the parts_treeview based on the search."""
        self.serch_db_parts(part_name)

    def restore_treeview(self) -> None:
        """Restore the parts_treeview to show all parts."""
        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())
        
        # Load all parts into the parts_treeview
        self.load_data()

        # Destroy the search window
        self.search_window.destroy()

    def serch_db_parts(self, partName: str) -> None:
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

class MoshionPlanningPage(PageBase):
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the MoshionPlanningPage.

        Args:
            parent (type): The parent widget.
            controller (type): The main application controller.
        """
        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, "Motshion verification")

        self.DROP_OFF_ZONE = (-0.50, -0.100, 0.0)
        self.PICK_UP_ZONE = (0.50, -0.100, 0.50)

        # Create a canvas for displaying 3D paths
        self.canvas_frame = ttk.Frame(self.content_frame)
        self.canvas_frame.grid(row=0, column=0, columnspan=2, pady=10, sticky="nsew")

        self.figsizes = (8, 6)

        # Additional attribute to store cart data
        self.cart_data = {}

        # Dictionary to store part information
        self.part_info_dict = {}  

        # Dictionary to store locations of each part
        self.locations = {}  

        self.part_names_to_fetch = []

        # Create a PartsDatabase instance for handling parts data
        self.parts_db = PartsDatabase()

        # Create buttons for interaction with available parts
        self.create_buttons(controller)

    def create_buttons(self, controller: RobotCart) -> None:

        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.content_frame)  # Change 'self' to 'self.content_frame'
        button_frame.grid(row=1, column=0, columnspan=2, pady=10, sticky="nsew")  # Use 'grid' instead of 'pack'

        # Create a button to refresh the data
        start_sim_button = tk.Button(button_frame, text="Start Sim", command = self.get_select_part)
        start_sim_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a button to refresh the data
        conferm_path_button = tk.Button(button_frame, text="Conferm Path", command = self.show_animation)
        conferm_path_button.grid(row=1, column=0, padx=10, pady=10)

        # Button to return to Part Selection
        return_button = tk.Button(button_frame, text="Part Selection", command = lambda: controller.show_frame(MainUserPage))
        return_button.grid(row=2, column=0, padx=10, pady=10, sticky="nw")

    def set_cart_data(self, cart_data):
        # Setter method to update cart data
        self.part_names_to_fetch = cart_data
        print(self.part_names_to_fetch)

    def get_select_part(self):
        # Retrieve and store information for specified parts
        for part_name_to_find in self.part_names_to_fetch:
            part = self.parts_db.get_part_by_name(part_name_to_find)
            if part:
                self.part_info_dict[part_name_to_find] = {
                    'PartName': part[1],
                    'NumberOfParts': part[2],
                    'LocationX': part[3],
                    'LocationY': part[4],
                    'LocationZ': part[5],
                    'Orientation': [part[6], part[7], part[8]],
                    'FullWeight': part[9],
                    'HalfWeight': part[10],
                    'EmptyWeight': part[11],
                    'InService': part[12]
                }
        self.get_part_locations()

    def get_part_locations(self):

        for part_name, part_info in self.part_info_dict.items():
            location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
            self.locations[part_name] = location

        self.generate_robot_path()

    def generate_robot_path(self):
        """Generate paths and velocity profiles for each part."""

        planner = PathPlanner(20, 2)
        planner.setVelocityPFP(1)

        self.direction_vector = []
        self.paths = []
        for part_name, location in self.locations.items():
            self.direction_vector.append((np.array(location) / np.linalg.norm(location), 0, 0))  # Calculate direction vector from the origin to the current location
            self.paths.append(planner.generate_path(location, self.DROP_OFF_ZONE, linear=False))

        self.show_path()

    def show_path(self):
        """Display the generated paths."""

        # Create a new figure and a single 3D plot for the existing paths
        fig_path = plt.figure(figsize = self.figsizes)
        ax_path = fig_path.add_subplot(111, projection='3d')

        # Initialize variables to store min and max values for each axis
        x_min, x_max = float('inf'), float('-inf')
        y_min, y_max = float('inf'), float('-inf')
        z_min, z_max = float('inf'), float('-inf')

        for path in self.paths:
            x_coords, y_coords, z_coords = path.T

            # Customize marker size and transparency
            ax_path.scatter(x_coords, y_coords, z_coords, s=20, marker='o', alpha=0.5)

            # Update min and max values for each axis
            x_min = min(x_min, np.min(x_coords))
            x_max = max(x_max, np.max(x_coords))
            y_min = min(y_min, np.min(y_coords))
            y_max = max(y_max, np.max(y_coords))
            z_min = min(z_min, np.min(z_coords))
            z_max = max(z_max, np.max(z_coords))

        ax_path.scatter([0], [0], color="red")

        # Customize azimuth (horizontal viewing angle) and elevation (vertical viewing angle)
        ax_path.view_init(azim=-45, elev=20)

        # Add grid lines
        ax_path.grid(True)

        ax_path.set_xlabel('X')
        ax_path.set_ylabel('Y')
        ax_path.set_zlabel('Z')

        # Embed the matplotlib figure in the Tkinter window on the left side
        canvas_path = FigureCanvasTkAgg(fig_path, master=self.canvas_frame)
        canvas_path.draw()
        canvas_path.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

        # Show the plot in the Tkinter window
        canvas_path.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=1)

    def show_animation(self):
        """Perform inverse kinematics for the generated paths and visualize the motion using RobotArm."""

        # Initialize the RobotArm with the URDF file path
        urdf_file_path = "app\\backend\\python code\\urdf_tes1.urdf"  # Replace with the actual file path
        robot = RobotArm(urdf_file_path)

        target_positions = []
        target_orientations = []

        for vector, path in enumerate(self.paths):
            for point in path:
                target_positions.append(point)
                target_orientations.append([0, 0, np.pi/4])

        print("Animating")

        # Create a new figure and a single 3D plot for the robot arm animation
        fig_robot = plt.figure(figsize = self.figsizes)
        ax_robot = fig_robot.add_subplot(111, projection='3d')

        def update(frame, self, target_positions, target_orientations, ax):
            """Update the animation frame.

            Args:
                frame (int): The frame number.
                self: The current object instance.
                target_positions (list): List of target positions.
                target_orientations (list): List of target orientations.
                ax: The subplot for the 3D plot.
            """
            ax.clear()

            target_position = target_positions[frame]
            target_orientation = target_orientations[frame]
            ik_solution = list(robot.calculate_ik([target_position], [target_orientation], precision=2, batch_size=1))[0]

            robot.my_chain.plot(np.radians(ik_solution), ax, target=np.array(target_position, dtype=np.float32))
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_zlim(-1, 1)

            # Set the view for the plot
            # ax.view_init(elev=30, azim=45)

        anim = animation.FuncAnimation(fig_robot, update, frames=len(target_positions), fargs=(self, target_positions, target_orientations, ax_robot), interval=1, repeat=False)

        # Embed the Matplotlib figure in the Tkinter window on the right side
        canvas_robot = FigureCanvasTkAgg(fig_robot, master=self.canvas_frame)
        canvas_robot.draw()
        canvas_robot.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

if __name__ == "__main__":
    """Entry point of the script."""

    app = RobotCart()
    app.mainloop()