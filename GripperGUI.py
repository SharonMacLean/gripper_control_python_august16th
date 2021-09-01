from tkinter import *
import tkinter as tk
from tkinter.ttk import *
from tkinter import messagebox
from PIL import ImageTk, Image
import atexit

class GripperGUI:


    def __init__(self, master, gripper):
        # Declare background color for the GUI
        background_colour = "white"

        # Inherit tkinter attributes/methods
        self.master = master
        master.title("Automated Gripper Control")

        # Determine the size of the current screen and size GUI window accordingly.
        # Double check that this works correctly on a multi-screen setup
        # height = master.winfo_screenheight()
        # width = master.winfo_screenwidth()
        # master.geometry('%sx%s' % (int(width*0.6), int(height*0.4))
        master.geometry('650x600')  # Possibly remove this and opt for an autosize at end
        master.configure(bg=background_colour)

        # Altering the closing behaviour
        # master.wm_protocol("WM_DELETE_WINDOW", self.on_closing())
        #atexit.register(self.on_closing)

        # Inherit Gripper attributes/methods
        self.kukaGripper1 = gripper

        # Frame to hold the select gripper finger options
        frame_gripperfingers1 = tk.Frame(master, background=background_colour)
        frame_gripperfingers1.grid(row=0, column=0, rowspan=1, columnspan=7, sticky=W + E + N + S)

        # Insert label to display the currently selected gripper fingers
        self.image_folder_name = "Gripper_finger_images/"  # Name of the folder that contains the images
        # Dictionary of gripper finger images
        self.finger_image_names = dict([('None', "GripperBase_w_nofingers.PNG"),
                                        ('Rigid', "GripperBase_w_rigidfingers.PNG"),
                                        ('Thin Convex', "GripperBase_w_convexfingers.PNG"),
                                        ('Thin Concave', "GripperBase_w_concavefingers.PNG"),
                                        ('Thick Concave', "GripperBase_w_concavefingers.PNG"),
                                        ('Festo Flexible', "GripperBase_w_festofingers.PNG")])
        self.gripper_image = None  # Variable for storing the currently displayed image
        # Label to display the gripper image upon
        self.label_image = Label(frame_gripperfingers1, background=background_colour)
        self.label_image.grid(column=1, row=0)
        self.update_gripper_finger_image()  # Display the image


        # Frame to hold gripper image
        frame_gripperfingers2 = tk.Frame(frame_gripperfingers1, background=background_colour)
        frame_gripperfingers2.grid(row=0, column=0, rowspan=2, columnspan=1, sticky=W + E + N + S)

        # Insert Combobox to get user to input current fingers being used
        self.fingerlabel = Label(frame_gripperfingers2, text="Select Gripping Fingers", font=("Times New Roman", 16), background=
                                 background_colour)
        self.fingerlabel.grid(column=0, row=0, padx=(0, 70), pady=(0, 10))

        self.fingercombo = Combobox(frame_gripperfingers2, state="readonly")  # Readonly so the user cannot type into the combobox
        self.fingercombo['values'] = ("Rigid", "Thin Convex", "Thin Concave", "Thick Concave", "Festo Flexible")
        self.fingercombo.set("")
        self.fingercombo.grid(column=0, row=1)
        self.fingercombo.bind("<<ComboboxSelected>>", self.update_finger_type)
        # The above bind line makes it so the finger type variable is automatically updated as soon as the user
        # makes a different selection in the combobox (is triggered by the selection event).

        # Frame to hold control type options
        frame_controltype1 = tk.Frame(master, background=background_colour)
        frame_controltype1.grid(row=1, column=0, rowspan=1, columnspan=1, sticky=W + E + N + S)

        # Insert Radio button to prompt user for type of setpoint entry
        self.radiolbl = Label(frame_controltype1, text="Choose Control Type", font=("Times New Roman", 16), background=
                              background_colour, justify=LEFT)
        self.radiolbl.grid(column=0, row=0, pady=(16, 4))

        self.radiovalue = IntVar()

        self.radio1 = tk.Radiobutton(frame_controltype1, text="Force Setpoint", font=("Times New Roman", 10),
                                     variable=self.radiovalue, value=1,bg=background_colour,
                                     activebackground=background_colour, highlightthickness=0)
        self.radio1.grid(column=0, row=1)

        self.radio2 = tk.Radiobutton(frame_controltype1, text="Mass Setpoint", font=("Times New Roman", 10),
                                     variable=self.radiovalue, value=2, bg=background_colour,
                                     activebackground=background_colour, highlightthickness=0)
        self.radio2.grid(column=0, row=2)

        self.radio3 = tk.Radiobutton(frame_controltype1, text="Unknown Mass", font=("Times New Roman", 10),
                                     variable=self.radiovalue, value=3, bg=background_colour,
                                     activebackground=background_colour, highlightthickness=0)
        self.radio3.grid(column=0, row=3)

        # Use text entry to get desired force set point
        self.forcelabel = Label(frame_controltype1, text="Set Gripping Force [N]", font=("Times New Roman", 10), background=
                                background_colour)
        self.forcelabel.grid(column=1, row=1)

        self.forceinput = Entry(frame_controltype1, width=10)
        self.forceinput.grid(column=2, row=1)

        # Use text entry to get desired mass set point
        self.masslabel = Label(frame_controltype1, text="Input Object Mass [kg]", font=("Times New Roman", 10), background=
                               background_colour)
        self.masslabel.grid(column=1, row=2)

        self.massinput = Entry(frame_controltype1, width=10)
        self.massinput.grid(column=2, row=2)

        # Text Entry Box
        self.objectpositionlabel = Label(frame_controltype1, text="Distance to Object [cm]", font=("Times New Roman", 10),
                                         background=background_colour)
        self.objectpositionlabel.grid(column=1, row=3)

        self.objectpositioninput = Entry(frame_controltype1, width=10)
        self.objectpositioninput.grid(column=2, row=3)

        # Frame to hold the Gripping buttons
        frame_grippingbuttons = tk.Frame(frame_controltype1, background=background_colour)
        frame_grippingbuttons.grid(row=0, column=3, rowspan=4, columnspan=1, sticky=W + E + N + S)

        self.stopbutton = Button(frame_grippingbuttons, text="Stop Gripping", command=self.stop_button_clicked)
        self.stopbutton.pack(side=BOTTOM)

        self.closebutton = Button(frame_grippingbuttons, text="Start Gripping", command=self.close_button_clicked)
        self.closebutton.pack(side=BOTTOM)

        self.rebootButton = Button(frame_grippingbuttons, text="Reset Motor", command=self.reset_button_clicked)
        self.rebootButton.pack(side=BOTTOM)

        # Frame to hold the opening gripper widgets
        frame_opengripper = tk.Frame(master, background=background_colour)
        frame_opengripper.grid(row=2, column=0, rowspan=1, columnspan=1, sticky=W + E + N + S)

        self.openlabel = Label(frame_opengripper, text="Open Gripper", font=("Times New Roman", 16),
                               background=background_colour, justify=LEFT)
        self.openlabel.grid(column=0, row=0, pady=(16, 4))

        self.openpositionlabel = Label(frame_opengripper, text="To an object size of [mm]:", font=("Times New Roman", 10),
                                       background=background_colour)
        self.openpositionlabel.grid(column=0, row=1, padx=20)

        self.openpositionvalue = Entry(frame_opengripper, width=10)
        self.openpositionvalue.grid(column=1, row=1)

        self.chk_state = BooleanVar()
        self.chk_state.set(False)
        self.chk = tk.Checkbutton(frame_opengripper, text='Fully Open', var=self.chk_state, background=
                                  background_colour, activebackground=background_colour, highlightthickness=0)
        self.chk.grid(column=2, row=1)

        # Define open, close, and stop buttons on GUI with respective object methods
        self.openbutton = Button(frame_opengripper, text="Open Gripper", command=self.open_button_clicked)
        self.openbutton.grid(column=4, row=1)

        # Frame to hold the measurement values widgets
        frame_measurements = tk.Frame(master, background=background_colour)
        frame_measurements.grid(row=3, column=0, rowspan=1, columnspan=1, sticky=W + E + N + S)
        frame_measurements.grid(column=0, padx=15)

        measurementslabel = Label(frame_measurements, text="Measurements", font=("Times New Roman", 16), background=
                                  background_colour, justify=LEFT)
        measurementslabel.grid(column=0, row=0, pady=(16, 4))

        # Display current gripper size opening - Allows updating
        self.cursizelabel = Label(frame_measurements, text="Current gripper opening:", font=("Times New Roman", 10),
                                  background=background_colour)
        self.cursizelabel.grid(column=0, row=1)

        self.cursizevar = StringVar()
        self.cursizevar.set("X mm")
        self.cursize = Label(frame_measurements, textvariable=self.cursizevar, font=("Times New Roman", 10), background=
                             background_colour)
        self.cursize.grid(column=1, row=1)

        # Display current distance between grippers fingers w/o deflection - Allows updating
        self.curfingersizelabel = Label(frame_measurements, text="Current distance between gripper fingers \n(w/o "
                                                                 "estimated finger deflection): ", font=("Times New "
                                                                 "Roman", 10), background=background_colour, justify=
                                                                 CENTER)
        self.curfingersizelabel.grid(column=0, row=2)

        self.curfingersizevar = StringVar()
        self.curfingersizevar.set("X mm")
        self.curfingersize = Label(frame_measurements, textvariable=self.curfingersizevar, font=("Times New Roman", 10),
                                   background=background_colour)
        self.curfingersize.grid(column=1, row=2)

        # Display current distance between gripper fingers with deflection - Allows updating
        self.curfingersize_deflect_label = Label(frame_measurements, text="Current distance between gripper fingers \n"
                                                                          "(w/ estimated finger deflection): ",
                                                                          font=("Times New Roman", 10),background=
                                                                          background_colour, justify=CENTER)
        self.curfingersize_deflect_label.grid(column=0, row=3)

        self.curfingersize_deflect_var = StringVar()
        self.curfingersize_deflect_var.set("X mm")
        self.curfingersize_deflect = Label(frame_measurements, textvariable=self.curfingersize_deflect_var, font=
                                                            ("Times New Roman", 10), background=background_colour)
        self.curfingersize_deflect.grid(column=1, row=3)

        # Display current gripping force - Allows updating
        self.curforcelabel = Label(frame_measurements, text="Current gripping force:", font=("Times New Roman", 10), background=
                                   background_colour)
        self.curforcelabel.grid(column=2, row=1)

        self.curforcevar = StringVar()
        self.curforcevar.set("X N")
        self.curforce = Label(frame_measurements, textvariable=self.curforcevar, font=("Times New Roman", 10), background=
                              background_colour)
        self.curforce.grid(column=3, row=1)

        # Display current gripper state - Allows updating
        self.curstatelabel = Label(frame_measurements, text="Current Gripper State:", font=("Times New Roman", 10), background=
                                   background_colour)
        self.curstatelabel.grid(column=2, row=2)

        self.curstatevar = StringVar()
        self.curstatevar.set("Unknown")
        self.curstate = Label(frame_measurements, textvariable=self.curstatevar, font=("Times New Roman", 10), background=
                              background_colour)
        self.curstate.grid(column=3, row=2)

        # Display current position uncertainty - Allows updating
        self.posuncertaintylabel = Label(frame_measurements, text="Current position uncertainty:", font=("Times New Roman", 10),
                                         background=background_colour)
        self.posuncertaintylabel.grid(column=0, row=4)

        self.posuncertaintyvar = StringVar()
        self.posuncertaintyvar.set("+/-X mm")
        self.posuncertainty = Label(frame_measurements, textvariable=self.posuncertaintyvar, font=("Times New Roman", 10),
                                    background=background_colour)
        self.posuncertainty.grid(column=1, row=4)

        # Frame to hold the sensor status
        frame_sensorstatus = tk.Frame(master, background=background_colour)
        frame_sensorstatus.grid(row=4, column=0, rowspan=1, columnspan=1, sticky=W + E + N + S)

        self.sensorstatuslabel = Label(frame_sensorstatus, text="Sensor Status:",
                                     font=("Times New Roman", 16),
                                     background=background_colour)
        self.sensorstatuslabel.grid(column=0, row=0, pady=(16, 4))

        # Sensor not connected labels
        self.label_servomotor = Label(frame_sensorstatus, text="Smart servo motor error code: ",
                                         font=("Times New Roman", 10),
                                         background=background_colour)
        self.label_loadcell_left = Label(frame_sensorstatus, text="Left load cell: ",
                                     font=("Times New Roman", 10),
                                     background=background_colour)
        self.label_loadcell_right = Label(frame_sensorstatus, text="Right load cell: ",
                                     font=("Times New Roman", 10),
                                     background=background_colour)
        self.label_flexsensor_left = Label(frame_sensorstatus, text="Left flex sensor: ",
                                     font=("Times New Roman", 10),
                                     background=background_colour)
        self.label_flexsensor_right = Label(frame_sensorstatus, text="Right flex sensor: ",
                                     font=("Times New Roman", 10),
                                     background=background_colour)

        self.label_servomotor.grid(column=0, row=1)
        self.label_loadcell_left.grid(column=0, row=2)
        self.label_loadcell_right.grid(column=0, row=3)
        self.label_flexsensor_left.grid(column=0, row=4)
        self.label_flexsensor_right.grid(column=0, row=5)

        # Sensor not connected labels
        self.servomotor_error_var = StringVar()
        self.servomotor_error_var.set("")
        self.label_servomotor = Label(frame_sensorstatus, textvariable=self.servomotor_error_var,
                                     font=("Times New Roman", 10),
                                     background=background_colour)
        self.label_loadcell_left_value = Label(frame_sensorstatus, text="Connected",
                                     font=("Times New Roman", 10),
                                     background=background_colour)
        self.label_loadcell_right_value = Label(frame_sensorstatus, text="Connected",
                                     font=("Times New Roman", 10),
                                     background=background_colour)
        self.label_flexsensor_left_value = Label(frame_sensorstatus, text="Connected",
                                     font=("Times New Roman", 10),
                                     background=background_colour)
        self.label_flexsensor_right_value = Label(frame_sensorstatus, text="Connected",
                                     font=("Times New Roman", 10),
                                     background=background_colour)

        self.label_servomotor.grid(column=1, row=1)
        self.label_loadcell_left_value.grid(column=1, row=2)
        self.label_loadcell_right_value.grid(column=1, row=3)
        self.label_flexsensor_left_value.grid(column=1, row=4)
        self.label_flexsensor_right_value.grid(column=1, row=5)

    # Read gripping control set point from user input, convert to a force set point if needed and pass to gripper method
    def close_button_clicked(self):

        # If no finger type was selected, remind the user and do not close the gripper
        if self.kukaGripper1.fingertype is None:
            messagebox.showerror("No Finger Type Selected", "Please select a finger type from the drop down menu.")
            return

        # Determine input type selection from radio button
        # Force Setpoint selected
        if self.radiovalue.get() == 1:
            controlunits = 'N'  # not needed for actual implementation

            # Retrieve the force input by the user
            forceinput = self.forceinput.get()

            # Validate the user input force
            inputresult = self.validate_user_input(input=forceinput, fieldname="Set Gripping Force",
                                                   minvalue=self.kukaGripper1.minimumforce,
                                                   maxvalue=self.kukaGripper1.maximumforce,
                                                   valuetype="force", valueunit="N")

            # If the input was valid, update the control setpoint
            if inputresult is not None:
                controlsetpoint = float(self.forceinput.get())
            else:
                return


        # Mass Setpoint selected
        elif self.radiovalue.get() == 2:
            controlunits = 'kg'

            # Retrieve the mass input by the user
            massinput = self.massinput.get()

            inputresult = self.validate_user_input(input=massinput, fieldname="Input Object Mass",
                                                   minvalue=0,
                                                   maxvalue=self.kukaGripper1.maximummass,
                                                   valuetype="mass", valueunit="kg")

            # If the input was valid, update the control setpoint
            if inputresult is not None:
                # Convert Mass setpoint to a Force Set point
                # Fgrip = m*(accel_max + g)/(2*mu)
                controlsetpoint = inputresult * 24.375

                # Ensure setpoint is not below minimum measurable value
                if controlsetpoint < self.kukaGripper1.minimumforce:
                    controlsetpoint = self.kukaGripper1.minimumforce
            else:
                return


        # Unknown Mass selection
        elif self.radiovalue.get() == 3:
            controlsetpoint = self.kukaGripper1.maximumforce
            controlunits = 'N'

        # No input given
        else:
            messagebox.showerror("No Control Option Selected", "Please select a control type using the Radio Buttons")
            raise ValueError

        # Ensure object position input is within acceptable range
        # (Object position is measured from base of Kuka wrist)

        # Retrieve the object position input by the user
        objectpositioninput = self.objectpositioninput.get()

        # Validate the user input
        inputresult = self.validate_user_input(input=objectpositioninput, fieldname="Distance to Object",
                                               minvalue=self.kukaGripper1.minimumobjectdist,
                                               maxvalue=self.kukaGripper1.maximumobjectdist,
                                               valuetype="distance", valueunit="cm")

        # If the input was valid, close the gripper
        if inputresult is not None:
            self.kukaGripper1.close_gripper(objectdistance=inputresult, forcesetpoint=controlsetpoint,
                                            gui=self)

    # Pass input to open gripper to inputted value using Gripper.open_gripper
    def open_button_clicked(self):

        fingertype = self.kukaGripper1.fingertype

        # If no finger type was selected, remind the user and do not open the gripper
        if fingertype is None:
            messagebox.showerror("No Finger Type Selected", "Please select a finger type from the drop down menu.")
            return

        # Check if "fully open" has been selected
        if self.chk_state.get():
            # position = self.kukaGripper1.maximumsize
            self.kukaGripper1.zero_gripper()
            return

        # If here, "fully open" was not selected. Retrieve the object position input by the user
        openpositionvalue = self.openpositionvalue.get()

        # Determine the min valid size
        finger_offset = self.kukaGripper1.finger_position_offsets[fingertype]
        minsize = 0
        if finger_offset < 0:
            minsize = -2*finger_offset

        # Determine the max valid size
        kuka_maxsize = self.kukaGripper1.maximumsize
        maxsize = kuka_maxsize
        if finger_offset != 0:
            maxsize = maxsize - 2*finger_offset

        # Validate the user input
        inputresult = self.validate_user_input(input=openpositionvalue, fieldname="To an object of size",
                                               minvalue=minsize,
                                               maxvalue=maxsize,
                                               valuetype="opening size", valueunit="mm")

        # If the input was valid, open the gripper
        if inputresult is not None:
            # Convert entered object size into distance along lead screw
            qctp_position = inputresult + 2*finger_offset
            leadscrew_position = (kuka_maxsize - qctp_position) / 2
            self.kukaGripper1.open_gripper(position=leadscrew_position)

    # Switch gripper to 0-torque mode on user request using Gripper.stop_gripper_loose
    def stop_button_clicked(self):

        # Use stop_gripper_loose method
        self.kukaGripper1.stop_gripper_loose()

    def reset_button_clicked(self):

        self.kukaGripper1.reset_motor()

    # Handles when user hits the x button in the corner of the GUI
    # def on_closing(self):
    #   if messagebox.askokcancel("Quit", "Do you want to quit?"):
    #      self.master.destroy()

    # Handles when user hits the x button in the corner of the GUI.
    # See if I need to put a timeout thing here to make sure it doesn't get stuck before ending the program.
    #def on_closing(self):
        #print("Stop")
        #self.stop_button_clicked()

    # This method updates the self.fingertype variable when the user selects a new fingertype from the combobox.
    def update_finger_type(self, event):
        self.kukaGripper1.set_finger_type(self.fingercombo.get())
        self.update_gripper_finger_image()

    def change_finger_combobox_state(self, gripper_status):
        # Check the current state of the finger combobox
        # Readonly: user can interact w/ the combobox but cannot type a custom fingertype into it
        # Disabled: user cannot interact w/ the combobox (cannot make a new selection)
        currentstate = str(self.fingercombo['state'])

        # Finger combobox should be deactivated if the status is Opening, Closing or Holding
        deactivate_menu = gripper_status in (self.kukaGripper1.status_types.Opening,
                                        self.kukaGripper1.status_types.Closing, self.kukaGripper1.status_types.Holding)

        # Change the current state
        if currentstate == 'readonly' and deactivate_menu:
            self.fingercombo['state'] = 'disabled'
        elif currentstate == 'disabled' and not deactivate_menu:
            self.fingercombo['state'] = 'readonly'

    def update_gripper_finger_image(self):
        # Try to open the image from the given file path
        try:
            self.gripper_image = Image.open(self.image_folder_name +
                                            self.finger_image_names[str(self.kukaGripper1.fingertype)])
        # If the image cannot be opened, display that on the GUI using the image label.
        except FileNotFoundError:
            print("Image file not found from given file path.")
            self.label_image.configure(image="")
            self.label_image.configure(text="Image of gripper fingers could not be found.")
            return

        # Scale image to a fixed height to ensure the GUI elements do not shift dramatically each time the image changes
        # Original image size
        gripper_image_height = self.gripper_image.height
        gripper_image_width = self.gripper_image.width

        # Scale factor
        height_desired = 120
        size_factor = height_desired / gripper_image_height

        # Scaled image size
        width_scaled = round(gripper_image_width * size_factor)
        height_scaled = round(gripper_image_height * size_factor)
        self.gripper_image = self.gripper_image.resize((width_scaled, height_scaled), Image.ANTIALIAS)
        self.gripper_image = ImageTk.PhotoImage(self.gripper_image)
        self.label_image.configure(image=self.gripper_image)


    # Validates numeric user input from the text fields
    def validate_user_input(self, input, fieldname, minvalue, maxvalue, valuetype, valueunit):

        # If no value was entered, display an error.
        if input == '':
            messagebox.showerror("No Value Entered", ("Please enter a number into the '" + fieldname + "' field."))
            raise ValueError("No value entered into the '" + fieldname + "' field.")
        else:
            # Try to cast the entered value to a float
            try:
                inputfloat = float(input)
            # If the entered value cannot be cast to a float (aka is not a number), display an error
            except ValueError:
                messagebox.showerror("Non-numeric Input", ("The value in the '" + fieldname + "' field is not a "
                                                           "numeric value. Please enter a number."))
            # If the value was successfully cast to a float
            else:
                # If the value is in the valid range, close the gripper
                if minvalue < inputfloat <= maxvalue:
                    return inputfloat
                else:  # If the value is invalid show an error
                    messagebox.showerror("Invalid Range", ("Please enter a " + valuetype + " between " +
                                         str(minvalue) + " and " +
                                         str(maxvalue) + " " + valueunit))