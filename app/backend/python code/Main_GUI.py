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

def error_handling_wrapper(func):
    """A decorator that wraps a function with error handling.

    Args:
        func (function): The function to be wrapped.

    Returns:
        function: The wrapped function with error handling.
    """
    def wrapper(self, *args, **kwargs):
        """Wrapper function that adds error handling to the original function.

        Returns:
            Any: The result of the original function.

        Raises:
            tk.TclError: If a TclError occurs (typically for Tkinter-related issues).
            matplotlib.MatplotlibError: If a MatplotlibError occurs.
            Exception: If any other unexpected error occurs.
        """
        try:
            # Call the original function
            return func(self, *args, **kwargs)
        
        except tk.TclError as tcl_error:
            # Handle TclError, which might occur for Tkinter-related issues
            error_msg = f"TclError in {func.__name__}: {str(tcl_error)}"
            print(error_msg)
            self.popup_error(error_msg)

        except Exception as e:
            # Handle other unexpected errors 
            error_msg = f"Error in {func.__name__}: {str(e)}"
            print(error_msg)
            self.popup_error(error_msg)

    # Copy the original function's metadata to the wrapper
    wrapper.__doc__ = func.__doc__
    wrapper.__name__ = func.__name__
    wrapper.__module__ = func.__module__

    return wrapper

def rotate_x(matrix, angle_x):
    """_summary_

    Args:
        matrix (_type_): _description_
        angle_x (_type_): _description_

    Returns:
        _type_: _description_
    """
    rotation_matrix_x = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x), -np.sin(angle_x)],
        [0, np.sin(angle_x), np.cos(angle_x)]
    ])
    rotated_matrix = np.dot(rotation_matrix_x, matrix)
    return rotated_matrix

def rotate_y(matrix, angle_y):
    """_summary_

    Args:
        matrix (_type_): _description_
        angle_y (_type_): _description_

    Returns:
        _type_: _description_
    """
    rotation_matrix_y = np.array([
        [np.cos(angle_y), 0, np.sin(angle_y)],
        [0, 1, 0],
        [-np.sin(angle_y), 0, np.cos(angle_y)]
    ])
    rotated_matrix = np.dot(rotation_matrix_y, matrix)
    return rotated_matrix

def rotate_z(matrix, angle_z):
    """_summary_

    Args:
        matrix (_type_): _description_
        angle_z (_type_): _description_

    Returns:
        _type_: _description_
    """
    rotation_matrix_z = np.array([
        [np.cos(angle_z), -np.sin(angle_z), 0],
        [np.sin(angle_z), np.cos(angle_z), 0],
        [0, 0, 1]
    ])
    rotated_matrix = np.dot(rotation_matrix_z, matrix)
    return rotated_matrix

def translate_point_along_z(point, orientation_matrix, translation_distance):
    """_summary_

    Args:
        point (_type_): _description_
        orientation_matrix (_type_): _description_
        translation_distance (_type_): _description_

    Returns:
        _type_: _description_
    """
    final_coordinates = np.dot(orientation_matrix, [0, 0, translation_distance]) + point

    return final_coordinates

def XY_angle(vector1, vector2):
    # Calculate the dot product
    dot_product = np.dot(vector1[:-1], vector2[:-1])

    # Calculate the magnitudes of the vectors
    magnitude_vector1 = np.linalg.norm(vector1[:-1])
    magnitude_vector2 = np.linalg.norm(vector2[:-1])

    # Calculate the angle in radians
    angle_radians = np.arccos(dot_product / (magnitude_vector1 * magnitude_vector2))

    # Calculate the cross product to determine orientation
    cross_product = np.cross(vector1[:-1], vector2[:-1])

    # Check the z-component of the cross product
    if cross_product > 0:
        # Vector2 is counterclockwise (CCW) with respect to vector1
        return angle_radians
    else:
        # Vector2 is clockwise (CW) with respect to vector1
        return -angle_radians

class RobotCart(tk.Tk):
    """Main application class representing a robot control interface.

    Args:
        tk (module): The Tkinter module.
    """
    @error_handling_wrapper
    def __init__(self, *args, **kwargs):
        """Initialize the RobotCart application.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        tk.Tk.__init__(self, *args, **kwargs)

        # Initialize attributes
        self.logged_in = False
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
        try:
            parts_db = PartsDatabase()
            parts_db.create_parts_table()
        except Exception as e:
            error_msg = f"Error connecting to the database: {str(e)}"
            self.popup_error(error_msg)

        # Create and configure the menu bar
        menuBar = tk.Menu(container)
        settingMenu = tk.Menu(menuBar, tearoff=0)
        settingMenu.add_command(label='Motor Set Up', command=lambda: self.admin_control(0))
        settingMenu.add_command(label='Database Config', command=lambda: self.admin_control(1))
        settingMenu.add_separator()
        settingMenu.add_command(label='Primary Color', command=self.primary_color)
        settingMenu.add_command(label='Secondary Color', command=self.secondary_color)
        settingMenu.add_command(label='Highlight Color', command=self.highlight_color)
        settingMenu.add_command(label='Low Weight', command=self.low_weight_color)
        menuBar.add_cascade(label="Menu", menu=settingMenu)

        try:
            tk.Tk.config(self, menu=menuBar)
        except Exception as e:
            self.popup_error(f"Error configuring menu: {str(e)}")

        # Configure the style for Treeview widgets
        style = ttk.Style()
        style.theme_use('default')

        # Specify your tag configurations here
        style.configure("OddRow.TTreeview", background='white')
        style.configure("EvenRow.TTreeview", background='lightblue')
        style.configure("LowWeight.TTreeview", background='lightcoral')
        # Add more tag configurations as needed

        try:
            tk.Tk.config(self, menu=menuBar)
        except Exception as e:
            self.popup_error(f"Error configuring menu: {str(e)}")

    def show_frame(self, cont: tk.Frame, cart_data=None) -> None:
        """Show the specified frame.

        Args:
            cont (tk.Frame): The frame to be shown.
            cart_data (Any, optional): Additional data for the frame. Defaults to None.
        """
        frame = self.frames.get(cont)

        if frame:
            if hasattr(frame, 'set_cart_data'):
                frame.set_cart_data(cart_data)

            frame.tkraise()
        else:
            self.popup_error(f"Error: Frame {cont} not found.")

    @error_handling_wrapper
    def admin_control(self, page_num) -> None:
        """Control the admin page.

        Args:
            page_num (int): The admin page number.
        """
        self.admin_page = page_num

        if not self.logged_in:
            self.show_frame(SecurityPage)
        else:
            if self.admin_page == 0:
                self.show_frame(MotorSetUpPage)
            elif self.admin_page == 1:
                self.show_frame(DataBacePannle)

    @error_handling_wrapper
    def popup_error(self, msg: str) -> None:
        """Show an error popup with the specified message.

        Args:
            msg (str): The error message.
        """
        error_popup = tk.Toplevel(self)
        error_popup.title("Error")

        # Create a frame to hold the error message and button
        frame = ttk.Frame(error_popup)
        frame.pack(padx=20, pady=20)

        # Error message label
        label = ttk.Label(frame, text=msg, font=("Helvetica", 12), wraplength=300)
        label.grid(row=0, column=0, padx=10, pady=10)

        # Okay button
        okay_button = ttk.Button(frame, text="Okay", command=error_popup.destroy)
        okay_button.grid(row=1, column=0, pady=10)

        # Center the error popup on the screen
        error_popup.geometry(f"+{self.winfo_screenwidth() // 2 - 150}+{self.winfo_screenheight() // 2 - 100}")

        # Make the error popup a transient window, preventing interaction with the main window until closed
        error_popup.transient(self)
        error_popup.grab_set()

        # Wait for the error popup to be closed before returning
        self.wait_window(error_popup)

    @error_handling_wrapper
    def primary_color(self) -> None:
        """Choose and set the primary color for the interface."""
        primary_color = colorchooser.askcolor()[1]
        if primary_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                current_frame.parts_treeview.tag_configure('OddRow.TTreeview', background=primary_color)
            else:
                self.popup_error("Error: MainUserPage frame not found.")

    @error_handling_wrapper
    def secondary_color(self) -> None:
        """Choose and set the secondary color for the interface."""
        secondary_color = colorchooser.askcolor()[1]
        if secondary_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                current_frame.parts_treeview.tag_configure('EvenRow.TTreeview', background=secondary_color)
            else:
                self.popup_error("Error: MainUserPage frame not found.")
        
    @error_handling_wrapper
    def low_weight_color(self) -> None:
        """Choose and set the color for low-weight items in the interface."""
        low_weight_color = colorchooser.askcolor()[1]
        if low_weight_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                current_frame.parts_treeview.tag_configure('LowWeight.TTreeview', background=low_weight_color)
            else:
                self.popup_error("Error: MainUserPage frame not found.")

    @error_handling_wrapper
    def highlight_color(self) -> None:
        """Choose and set the highlight color for the interface."""
        highlight_color = colorchooser.askcolor()[1]
        if highlight_color:
            current_frame = self.frames.get(MainUserPage)
            if current_frame:
                style = ttk.Style()
                style.map("OddRow.TTreeview", background=[('selected', highlight_color)])
                style.map("EvenRow.TTreeview", background=[('selected', highlight_color)])
            else:
                self.popup_error("Error: MainUserPage frame not found.")

class PageBase(tk.Frame):
    """Base class for pages in the RobotCart application.

    Args:
        tk (module): The Tkinter module.
    """
    @error_handling_wrapper
    def __init__(self, parent: tk.Widget, controller: RobotCart, title: str):
        """Initialize the PageBase.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
            title (str): The title of the page.
        """
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

    @error_handling_wrapper
    def popup_error(self, msg: str) -> None:
        """Display an error popup with the specified message.

        Args:
            msg (str): The error message.
        """
        error_popup = tk.Toplevel(self)
        error_popup.title("Error")

        # Create a frame to hold the error message and button
        frame = ttk.Frame(error_popup)
        frame.pack(padx=20, pady=20)

        # Error message label
        label = ttk.Label(frame, text=msg, font=("Helvetica", 12), wraplength=300)
        label.grid(row=0, column=0, padx=10, pady=10)

        # Okay button
        okay_button = ttk.Button(frame, text="Okay", command=error_popup.destroy)
        okay_button.grid(row=1, column=0, pady=10)

        # Center the error popup on the screen
        error_popup.geometry(f"+{self.winfo_screenwidth() // 2 - 150}+{self.winfo_screenheight() // 2 - 100}")

        # Make the error popup a transient window, preventing interaction with the main window until closed
        error_popup.transient(self)
        error_popup.grab_set()

        # Wait for the error popup to be closed before returning
        self.wait_window(error_popup)

class SecurityPage(PageBase):
    """Page for handling user authentication.

    Args:
        PageBase (class): The base class for pages.

    Raises:
        ValueError: Raised when username or password is empty.
    """
    @error_handling_wrapper
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the SecurityPage.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
        """
        super().__init__(parent, controller, "Security")

        # Username entry
        self.username_label = tk.Label(self.content_frame, text="Username")
        self.username_label.grid(row=0, column=0, padx=10, pady=10)
        self.username_var = tk.StringVar()
        self.username_entry = tk.Entry(self.content_frame, textvariable=self.username_var)
        self.username_entry.grid(row=0, column=1, padx=10, pady=10)

        # Password entry
        self.password_label = tk.Label(self.content_frame, text="Password")
        self.password_label.grid(row=1, column=0, padx=10, pady=10)
        self.password_var = tk.StringVar()
        self.password_entry = tk.Entry(self.content_frame, textvariable=self.password_var, show="*")
        self.password_entry.grid(row=1, column=1, padx=10, pady=10)

        # Login button
        self.login_button = tk.Button(self.content_frame, text="Login", command=lambda: self.check_credentials(controller))
        self.login_button.grid(row=2, column=0, columnspan=2, pady=10)

    @error_handling_wrapper
    def check_credentials(self, controller):
        """Check the entered credentials and perform login.

        Args:
            controller (RobotCart): The main application controller.

        Raises:
            ValueError: Raised when username or password is empty.
        """
        # Input validation: Check if username and password are not empty
        entered_username = self.username_var.get().strip()
        entered_password = self.password_var.get().strip()

        if not entered_username or not entered_password:
            raise ValueError("Username and password cannot be empty")

        # For simplicity, using hardcoded username and password
        correct_username = "admin"
        correct_password = "admin123"

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
    """Page for selecting and managing parts.

    Args:
        PageBase (class): The base class for pages.
    """
    @error_handling_wrapper
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

            # Clear existing content in the parts_treeview
            self.parts_treeview.delete(*self.parts_treeview.get_children())

            # Iterate through the retrieved data and insert it into the parts_treeview
            for idx, record in enumerate(data):
                # Determine the tag for alternate row coloring
                tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
                # Determine the service status for display
                service = "In Service" if record[13] == 1 else " "

                # Check if CurrentWeight is below 5 and set a different tag for low weight
                if record[12] < 5:  # Replace 5 with the actual threshold for low weight
                    tag = 'lowweight'

                # Insert the data into the parts_treeview with specified tags
                self.parts_treeview.insert(parent='', index='end', iid=idx, text='', values=(record[1], record[0], service, record[12]), tags=(tag,))

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

    @error_handling_wrapper
    def create_buttons(self, controller: RobotCart) -> None:
        """Create buttons for user interaction.

        This method creates buttons for refreshing data, adding parts to the cart,
        placing an order, and searching data.

        """
        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.content_frame)
        button_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky="nsew")

        # Create a button to refresh the data
        refresh_button = tk.Button(button_frame, text="Refresh Data", command=self.load_data, bg='#4CAF50')
        refresh_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a button to search the data
        search_button = tk.Button(button_frame, text="Search Data", command=self.search_data, bg='#2196F3')
        search_button.grid(row=0, column=1, padx=10, pady=10)

        # Create a button to add selected parts to the cart
        add_to_cart_button = tk.Button(button_frame, text="Add to Cart", command=self.select_part, bg='#FFEB3B')
        add_to_cart_button.grid(row=0, column=2, padx=10, pady=10)

        # Create a button to place an order with selected parts
        place_order_button = tk.Button(button_frame, text="Place Order", command=lambda: self.place_order(controller), bg='#FF5722')
        place_order_button.grid(row=0, column=3, padx=10, pady=10)

    @error_handling_wrapper
    def select_part(self) -> None:
        """Select a part and add it to the cart."""
        
        # Method to add a selected part to the cart
        selected = self.parts_treeview.focus()

        # Check if a part is selected before attempting to retrieve its values
        if selected:
            # Retrieve part_name and in_service status from the selected part
            part_name = self.parts_treeview.item(selected, "values")[0]
            in_service = self.parts_treeview.item(selected, "values")[2]

            if in_service == "In Service":
                # If the part is "In Service," display a message to the user
                error_msg = "This part is currently in service and cannot be added to the cart."
                self.popup_error(error_msg)
            else:
                # If the part is not "In Service," add it to the cart
                self.cart.append(part_name)
                # Update the cart treeview
                self.update_cart_tree()

        else:
            # If no part is selected, show an error message or handle it as needed
            error_msg = "No part selected. Please select a part before adding it to the cart."
            self.popup_error(error_msg)


    @error_handling_wrapper
    def update_cart_tree(self)-> None:
        """Update the cart treeview with the current content of the cart.

        This method clears the existing content in the cart treeview and inserts the current
        parts in the cart for display.

        """

        # Clear existing content in the cart treeview
        self.cart_tree.delete(*self.cart_tree.get_children())

        # Iterate through the parts in the cart and insert them into the cart treeview
        for idx, part in enumerate(self.cart):
            # Determine the tag for alternate row coloring
            tag = 'evenrow' if idx % 2 == 0 else 'oddrow'
            # Insert the part into the cart treeview with specified tag
            self.cart_tree.insert(parent='', index='end', iid=idx, text='', values=(part,), tags=(tag,))

    @error_handling_wrapper
    def place_order(self, controller: RobotCart) -> None:
        """Place an order for the selected parts in the cart.

        Args:
            controller (RobotCart): The main application controller.
        """
    
        # Check if the cart is empty before proceeding with the order
        if not self.cart:
            error_msg = "Cannot place an order with an empty cart. Please add parts to the cart first."
            self.popup_error(error_msg)
            return

        # Assuming you have logic here to process the order using the selected parts in self.cart

        # Show the Motion Planning Page with cart data
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

    @error_handling_wrapper     
    def search_data(self) -> None:
        """Open a search window to look up parts."""
        
        # Create a new Toplevel window for the search functionality
        search_window = tk.Toplevel(self)
        search_window.title("Lookup Parts")
        search_window.geometry("400x200")

        # Create a reference to the search window to use in the closing event
        self.search_window = search_window

        # Create a labeled frame for the part name search
        search_frame = tk.LabelFrame(search_window, text="Part Name")
        search_frame.pack(padx=10, pady=10)

        # Entry widget for entering the part name and bind KeyRelease event for auto-fill suggestions
        search_entry = tk.Entry(search_frame)
        search_entry.pack(padx=20, pady=20)
        search_entry.bind('<KeyRelease>', lambda event, entry=search_entry: self.auto_fill_suggestions(entry))

        # Bind the closing event of the search window to the restore_treeview method
        search_window.protocol("WM_DELETE_WINDOW", self.restore_treeview)


    @error_handling_wrapper
    def auto_fill_suggestions(self, entry: tk.Widget) -> None:
    
        """Auto-fill suggestions based on the current content of the entry."""
        # Get the current content of the entry
        current_text = entry.get()

        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())

        # Load suggestions based on the current_text
        self.load_suggestions(current_text)

    @error_handling_wrapper
    def load_suggestions(self, prefix: str) -> None:
        """Load suggestions based on the provided part name or part ID prefix.

        Args:
            prefix (str): The prefix used for searching part names or part IDs.
        """
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

            # Clear existing content in the parts_treeview
            self.parts_treeview.delete(*self.parts_treeview.get_children())

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

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)


    @error_handling_wrapper
    def search_and_update_treeview(self, part_name: str) -> None:
        """Search for parts and update the parts_treeview based on the search."""
        self.serch_db_parts(part_name)

        
    @error_handling_wrapper
    def restore_treeview(self) -> None:
        """Restore the parts_treeview to show all parts."""
        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())
        
        # Load all parts into the parts_treeview
        self.load_data()

        # Destroy the search window
        if hasattr(self, 'search_window') and self.search_window:
            self.search_window.destroy()

    @error_handling_wrapper
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

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

class MotorSetUpPage(PageBase):
    """Page for setting up and managing motors.

    Args:
        PageBase (class): The base class for pages.

    Raises:
        ValueError: Raised for specific errors during motor setup.
    """
    @error_handling_wrapper
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the MotorSetUpPage.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
        """
        super().__init__(parent, controller, "Motor Set Up Page")

        # Create an instance of the motorManager class for managing motors
        self.manager = motorManager({})

        # Configure the Motor treeview and bind a click event to select a motor
        self.configure_motor_treeview()
        self.motor_treeview.bind("<ButtonRelease-1>", lambda event: self.select_motor())

        # Create Entry boxes after configuring the treeview
        self.create_motor_entry_boxes()

        # Create buttons for interacting with motors
        self.create_buttons(controller)

        
    @error_handling_wrapper
    def configure_motor_treeview(self) -> None:
        """Configure the Treeview widget for displaying Motor Data.

        This method sets up the style, structure, and data loading for the Treeview.

        """
       
        # Existing code for configuring the Motor treeview
        style = ttk.Style()
        style.theme_use('default')
        style.configure("treeview", background="#D3D3D3", foreground="black", rowheight=25, fieldbackground="#D3D3D3")
        style.map("treeview", background=[('selected', "#347083")])

        tree_frame = tk.Frame(self.content_frame)
        tree_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        tree_scroll = tk.Scrollbar(tree_frame)
        tree_scroll.pack(side='right', fill='y')

        self.motor_treeview = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="extended", height=6)
        self.motor_treeview.pack()

        tree_scroll.config(command=self.motor_treeview.yview)

        columns = ["Motor Name", "index", "max_speed", "max_acceleration", "max_torqu", "is_activate", "type", "steps_per_revolution"]
        headings = ["Motor Name", "index", "max_speed", "max_acceleration", "max_torqu", "is_activate", "type", "steps_per_revolution"]

        self.motor_treeview['columns'] = columns
        self.motor_treeview.column("#0", width=0, stretch=0)

        for col in columns:
            if col in ("Motor Name", "index"):
                self.motor_treeview.column(col, anchor="w", width=140)
            else:
                self.motor_treeview.column(col, anchor="center", width=140)

        self.motor_treeview.heading("#0", text='', anchor="w")
        for col, heading in zip(columns, headings):
            self.motor_treeview.heading(col, text=heading, anchor="center")

        self.load_data()

    @error_handling_wrapper
    def load_data(self) -> None:
        """Load motor configuration data into the Treeview."""
        
        # Read motor configurations from the JSON file
        self.manager.read_motor_config("motors_config.json")

        # Clear existing items in the Treeview
        for item in self.motor_treeview.get_children():
            self.motor_treeview.delete(item)

        # Iterate over motors and insert them into the Treeview
        for motor_name, motor in self.manager.motors.items():
            # Define values for each motor to be displayed in the Treeview
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
            # Insert motor data into the Treeview
            self.motor_treeview.insert("", "end", text="", values=values)

    @error_handling_wrapper
    def create_motor_entry_boxes(self) -> None:
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

    @error_handling_wrapper
    def create_buttons(self, controller: RobotCart) -> None:
        """Create buttons for MotorSetUpPage.

        Args:
            controller (RobotCart): The main controller for the application.
        """
        try:
            # Button to save and edit motor data
            edit_motor_button = tk.Button(self.entry_frame, text="Save Edit", command=self.update_motor_data)
            edit_motor_button.grid(row=1, column=6, padx=10, pady=10)

            # Button to return to Part Selection
            return_button = tk.Button(self.content_frame, text="Part Selection", command=lambda: controller.show_frame(MainUserPage))
            return_button.grid(row=0, column=0, padx=10, pady=10, sticky="nw")

        except tk.TclError as tcl_error:
            error_msg = f"TclError while creating buttions: {str(tcl_error)}"
            self.popup_error(error_msg)
        except Exception as e:
            error_msg = f"Error while creating buttions: {str(e)}"
            self.popup_error(error_msg)

    @error_handling_wrapper
    def select_motor(self) -> None:
        """Select a motor from the Motor treeview and update Entry boxes with its data."""
        
        # Get the selected motor's data from the treeview
        selected = self.motor_treeview.focus()
        motor_data = self.motor_treeview.item(selected, "values")

        # Use set method to update StringVar values for Entry boxes
        self.mn_var.set(motor_data[0])   # Motor Name
        self.ms_var.set(motor_data[2])   # Max Speed
        self.ma_var.set(motor_data[3])   # Max Acceleration
        self.mt_var.set(motor_data[4])   # Max Torque
        self.mact_var.set(motor_data[5])  # Motor Activation
        self.mspr_var.set(motor_data[7])  # Steps per Revolution

        

    @error_handling_wrapper
    def update_motor_data(self) -> None:
        """Update motor data based on user input.

        Raises:
            ValueError: Raised for invalid or negative numeric input values.
        """
        # Ask for user confirmation before updating motor data
        confirmation = messagebox.askokcancel("Confirmation", "Are you sure you want to update the motor data?")

        if confirmation:
            # Get the values from the StringVar variables
            motor_name = self.mn_var.get()
            max_speed = float(self.ms_var.get())
            max_acceleration = float(self.ma_var.get())
            max_torqu = float(self.mt_var.get())
            is_activate = bool(self.mact_var.get())  # Assuming 'is_activated' is a boolean
            steps_per_revolution = int(self.mspr_var.get())

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

                # Reload data into the Treeview to reflect the changes
                self.load_data()

class DataBacePannle(PageBase):
    """Page for managing parts database.

    Args:
        PageBase (class): The base class for pages.
    """
    @error_handling_wrapper
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the DataBacePannle.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
        """
        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, "Database Panel")

        # Create a PartsDatabase instance for handling parts data
        self.parts_db = PartsDatabase()

        # Create a frame to hold the main treeview for available parts
        self.parts_treeview_frame = tk.Frame(self.content_frame)
        self.parts_treeview_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Configure the main treeview for available parts
        self.configure_parts_db_treeview()

        # Bind a click event to select a part
        self.parts_treeview.bind("<ButtonRelease-1>", lambda event: self.select_part())

        # Create buttons for interaction with available parts
        self.create_buttons(controller)

        # Create Entry boxes for parts data
        self.parts_entry_boxes()

    @error_handling_wrapper
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

    @error_handling_wrapper
    def load_data(self) -> None:
        """Load data into the Treeview widget.

        This method retrieves data from the Parts database, configures tags for styling,
        sorts the data alphabetically, and inserts it into the Treeview.

        """
        try:
            # Clear existing content in the parts_treeview
            self.parts_treeview.delete(*self.parts_treeview.get_children())

            # Establish a connection to the Parts database
            self.parts_db.connect()

            # Execute a SQL query to select all records from the Parts table and order by the second column (index 1)
            self.parts_db.cursor.execute("SELECT * FROM Parts ORDER BY PartName")

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

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

    @error_handling_wrapper
    def create_buttons(self, controller: RobotCart) -> None:
        """Create buttons for user interaction.

        This method creates buttons for refreshing data, adding parts to the cart, placing an order, and searching data.

        """
        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.content_frame)  # Change 'self' to 'self.content_frame'
        button_frame.grid(row=3, column=0, columnspan=2, pady=10, sticky="nsew")  # Use 'grid' instead of 'pack'

        # Button to return to Part Selection
        return_button = tk.Button(button_frame, text="Part Selection", command=lambda: controller.show_frame(MainUserPage))
        return_button.grid(row=0, column=0, padx=10, pady=10, sticky="nw")

        # Create a button to refresh the data
        refresh_button = tk.Button(button_frame, text="Refresh Data", command=self.load_data, bg='#4CAF50')
        refresh_button.grid(row=0, column=1, padx=10, pady=10)

        # Create a button to search the data
        search_button = tk.Button(button_frame, text="Search Data", command=self.search_data, bg='#2196F3')
        search_button.grid(row=0, column=2, padx=10, pady=10)

        # Button to save and edit motor data
        edit_motor_button = tk.Button(button_frame, text="Save Edit", command = self.update_part_data)
        edit_motor_button.grid(row=0, column=3, padx=10, pady=10)

    @error_handling_wrapper
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

    @error_handling_wrapper
    def select_part(self) -> None:
        """Select a part from the parts treeview and update Entry boxes with its data."""
        
        # Get the selected part's name from the treeview
        selected = self.parts_treeview.focus()
        part_name = self.parts_treeview.item(selected, "values")[0]

        # Retrieve part data from the database based on the selected part's name
        part_data = self.parts_db.get_part_by_name(part_name)
        print(part_data)  # Display part data in the console (for debugging)

        # Use set method to update StringVar values for Entry boxes
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
        self.EmptyWeight.set(part_data[11]) 
        self.CurrentWeight.set(part_data[12])
        self.InService.set(part_data[13]) 

    @error_handling_wrapper
    def update_part_data(self) -> None:
        """Update part data based on user input.

        Raises:
            ValueError: Raised for invalid or non-numeric input values.
        """
        # Ask for user confirmation before updating part data
        confirmation = messagebox.askokcancel("Confirmation", "Are you sure you want to update the Part data?")

        if confirmation:
            try:
                # Prepare a dictionary with the new part data
                new_part_data = {
                    'PartID': self.PartID.get(),
                    'PartName': self.PartName.get(),
                    'NumberOfParts': self.NumberOfParts.get(),
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

                # Update the part data in the database
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

            finally:
                try:
                    # Ensure the database is disconnected in the 'finally' block
                    self.parts_db.disconnect()
                except Exception as disconnect_error:
                    # Handle any potential error during disconnection
                    error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                    self.popup_error(error_msg)


    @error_handling_wrapper
    def search_data(self) -> None:
        """Open a search window to look up parts."""
        
        # Create a new top-level window for the search
        search_window = tk.Toplevel(self)
        search_window.title("Lookup parts")
        search_window.geometry("400x200")

        # Create a reference to the search window for later use
        self.search_window = search_window

        # Create a labeled frame within the search window
        search_frame = tk.LabelFrame(search_window, text="Part Name")
        search_frame.pack(padx=10, pady=10)

        # Create an entry widget for user input in the search frame
        search_entry = tk.Entry(search_frame)
        search_entry.pack(padx=20, pady=20)

        # Bind the key release event to trigger auto-fill suggestions
        search_entry.bind('<KeyRelease>', lambda event, entry=search_entry: self.auto_fill_suggestions(entry))

        # Bind the closing event of the search window to the restore_treeview method
        search_window.protocol("WM_DELETE_WINDOW", self.restore_treeview)

    @error_handling_wrapper
    def auto_fill_suggestions(self, entry: tk.Widget) -> None:
        """Auto-fill suggestions based on the current content of the entry."""
        # Get the current content of the entry
        current_text = entry.get()

        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())

        # Load suggestions based on the current_text
        self.load_suggestions(current_text)

    @error_handling_wrapper
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

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

    @error_handling_wrapper
    def search_and_update_treeview(self, part_name: str) -> None:
        """Search for parts and update the parts_treeview based on the search."""
        self.serch_db_parts(part_name)

    @error_handling_wrapper
    def restore_treeview(self) -> None:
        """Restore the parts_treeview to show all parts."""
        # Clear existing content in the parts_treeview
        self.parts_treeview.delete(*self.parts_treeview.get_children())
        
        # Load all parts into the parts_treeview
        self.load_data()

        # Destroy the search window
        self.search_window.destroy()

    @error_handling_wrapper
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

        finally:
            try:
                # Ensure the database is disconnected in the 'finally' block
                self.parts_db.disconnect()
            except Exception as disconnect_error:
                # Handle any potential error during disconnection
                error_msg = f"Error disconnecting from the database: {str(disconnect_error)}"
                self.popup_error(error_msg)

class MoshionPlanningPage(PageBase):
    """Page for Motion Planning and Verification.

    Args:
        PageBase (type): The base class for pages.
    """
    @error_handling_wrapper
    def __init__(self, parent: tk.Widget, controller: RobotCart):
        """Initialize the MoshionPlanningPage.

        Args:
            parent (tk.Widget): The parent widget.
            controller (RobotCart): The main application controller.
        """
        # Call the constructor of the base class (PageBase)
        super().__init__(parent, controller, "Motion Verification")

        # Define drop-off and pick-up zones in 3D space
        self.DROP_OFF_ZONE = [-0.51, -0.50, 0.0]
        self.PICK_UP_ZONE = [0.50, -0.50, 0.0]

        self.DROP_OFF_ORINT = rotate_x(np.eye(3), np.pi/2)
        self.IDLE_POSITION = (0.0, -0.43209168, -1.21891881, 0.0, -1.92214302,  1.138704, 0.0, -1.57057404,  0.0)
        self.GRAB_DISTANCE_Z = [0, 0, 0.1]
        self.NORTH_WALL = (0.0, 1.0, 0.0)
        self.EAST_WALL = (1.0, 0.0, 0.0)
        self.WEST_WALL = (-1.0, 0.0, 0.0)
        self.SOUNTH_WALL = (0.0, -1.0, 0.0)
        max_acc = 50
        max_vel = 50

        # Set the default size for figures in the canvas
        self.figsizes = (8, 6)

        # Additional attribute to store cart data
        self.cart_data = {}

        # Dictionary to store part information
        self.part_info_dict = {}

        # Dictionary to store locations of each part
        self.locations = {}  

        # Dictionary to store orientations of each part
        self.orientations = {}  

        # List to store part names to fetch
        self.part_names_to_fetch = []

        # list to stor the path of the robot
        self.travle_paths = []

        # list of the orientations alone the
        self.travle_orientation = []

        # what axix to fix alone the path
        self.travle_alinements = []

        # Create a PartsDatabase instance for handling parts data
        self.parts_db = PartsDatabase()

        # Create a pth panner for moshion planning
        self.planner = PathPlanner(max_acc, max_vel)

        # Initialize the RobotArm with the URDF file path
        urdf_file_path = "C:\\Users\\zachl\\Capstone2024\\app\\backend\\python code\\urdf_tes1.urdf"
        #urdf_file_path = "E:\\Capstone\\app\\backend\\python code\\urdf_tes1.urdf"
        self.robot = RobotArm(urdf_file_path, self.IDLE_POSITION)

        # Create buttons for interaction with available parts
        self.create_buttons(controller)


    @error_handling_wrapper
    def create_buttons(self, controller: RobotCart) -> None:
        """Create buttons for motion planning.

        Args:
            controller (RobotCart): The main application controller.
        """

        # Create a frame to hold the buttons
        button_frame = tk.Frame(self.content_frame)  # Use 'self.content_frame' instead of 'self'
        button_frame.grid(row=1, column=0, columnspan=2, pady=10, sticky="nsew")  # Use 'grid' instead of 'pack'

        # Create a button to start the simulation
        start_sim_button = tk.Button(button_frame, text="Start Sim", command=self.get_select_part)
        start_sim_button.grid(row=0, column=0, padx=10, pady=10)

        # Create a button to confirm the path
        confirm_path_button = tk.Button(button_frame, text="Confirm Path", command=self.show_animation)
        confirm_path_button.grid(row=1, column=0, padx=10, pady=10)

        # Button to return to Part Selection
        return_button = tk.Button(button_frame, text="Part Selection", command=lambda: controller.show_frame(MainUserPage))
        return_button.grid(row=2, column=0, padx=10, pady=10, sticky="nw")

    @error_handling_wrapper
    def set_cart_data(self, cart_data):
        """Set the cart data for motion planning.

        Args:
            cart_data (list): A list of part names to be fetched for the motion planning.
        """
        # Setter method to update cart data
        self.part_names_to_fetch = cart_data
        print(self.part_names_to_fetch)


    @error_handling_wrapper
    def get_select_part(self):
        """Retrieve and store information for specified parts.

        Fetch information for each part in the cart from the PartsDatabase and store it in the part_info_dict.
        """
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

    @error_handling_wrapper
    def get_part_locations(self):
        """Retrieve part locations from part_info_dict.

        Iterate through part_info_dict and extract the location information for each part, storing it in the locations dictionary.
        """

        for part_name, part_info in self.part_info_dict.items():
            location = (part_info['LocationX'], part_info['LocationY'], part_info['LocationZ'])
            orientation = (part_info['Orientation'][0], part_info['Orientation'][1], part_info['Orientation'][2])
            self.locations[part_name] = location
            self.orientations[part_name] = orientation

        # Proceed to generate the robot path using the collected part locations
        self.generate_robot_path()

    @error_handling_wrapper
    def generate_robot_path(self):
        """Generate paths and velocity profiles for each part."""

        for location, orientation in zip(self.locations.values(), self.orientations.values()):

            print(location, orientation)
        
            T1 = self.planner.generate_path(self.DROP_OFF_ZONE, location, linear=False)

            if (orientation == self.EAST_WALL):
                if (XY_angle(self.DROP_OFF_ZONE, location) >= 0):
                    delta_angles = np.linspace(0, np.pi/2, len(T1))
                else:
                    delta_angles = np.linspace(0, -np.pi/2, len(T1))
            
            elif (orientation == self.NORTH_WALL):
                if (XY_angle(self.DROP_OFF_ZONE, location) >= 0):
                    delta_angles = np.linspace(0, np.pi, len(T1))
                else:
                    delta_angles = np.linspace(0, -np.pi, len(T1))

            elif (orientation == self.WEST_WALL):
                if (XY_angle(self.DROP_OFF_ZONE, location) >= 0):
                    delta_angles = np.linspace(0, np.pi/2, len(T1))
                else:
                    delta_angles = np.linspace(0, -np.pi/2, len(T1))

            elif (orientation == self.SOUNTH_WALL):
                if (XY_angle(self.DROP_OFF_ZONE, location) <= np.pi):
                    delta_angles = np.linspace(0, 0, len(T1))
                else:
                    delta_angles = np.linspace(0, 0, len(T1))
            else:
                print("No angle")
                delta_angles = np.linspace(0, 0, len(T1))

            T1_alinements = ["all" for _ in range(len(T1))]
            T1_orientation = [rotate_z(self.DROP_OFF_ORINT, rad) for rad in delta_angles]
            
            translated_point = np.dot(T1_orientation[-1], self.GRAB_DISTANCE_Z) + location

            T2 = self.planner.generate_path(T1[-1], translated_point, linear=True)
            T2_alinements = ["all" for _ in range(len(T2))]
            T2_orientation = [T1_orientation[-1] for _ in range(len(T2))]

            T3 = self.planner.generate_path(T2[-1], location, linear=True)
            T3_alinements = ["all" for _ in range(len(T3))]
            T3_orientation = [T2_orientation[-1] for _ in range(len(T3))]
            
            T4 = self.planner.generate_path(location, self.DROP_OFF_ZONE, linear=False)
            T4_alinements = ["all" for _ in range(len(T4))]
            T4_orientation = [rotate_z(T3_orientation[-1], -rad) for rad in delta_angles]

            self.travle_paths.extend(T1)
            self.travle_paths.extend(T2)
            self.travle_paths.extend(T3)
            self.travle_paths.extend(T4)

            self.travle_orientation.extend(T1_orientation)
            self.travle_orientation.extend(T2_orientation)
            self.travle_orientation.extend(T3_orientation)
            self.travle_orientation.extend(T4_orientation)


            self.travle_alinements.extend(T1_alinements)
            self.travle_alinements.extend(T2_alinements)
            self.travle_alinements.extend(T3_alinements)
            self.travle_alinements.extend(T4_alinements)

        # Plot the 3D paths
        self.show_path()

    @error_handling_wrapper
    def show_path(self):
        """Display the generated paths."""
        self.planner.plot_3d_path()
        

    @error_handling_wrapper
    def show_animation(self):
        """Perform inverse kinematics for the generated paths and visualize the motion using RobotArm."""

        print("Animating")

        self.robot.animate_ik(self.travle_paths, self.travle_orientation, self.travle_alinements)


if __name__ == "__main__":
    """Entry point of the script."""

    app = RobotCart()
    app.mainloop()