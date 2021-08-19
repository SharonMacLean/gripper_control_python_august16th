from tkinter import *
from tkinter.ttk import *
from tkinter import messagebox
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
        master.geometry('650x350')  # Possibly remove this and opt for an autosize at end
        master.configure(bg=background_colour)

        # Altering the closing behaviour
        # master.wm_protocol("WM_DELETE_WINDOW", self.on_closing())
        #atexit.register(self.on_closing)

        # Inherit Gripper attributes/methods
        self.kukaGripper1 = gripper

        # Insert Radio button to prompt user for type of setpoint entry
        self.radiolbl = Label(master, text="Choose Control Type", font=("Times New Roman", 16), background=
                              background_colour)
        self.radiolbl.grid(column=0, row=0)

        self.radiovalue = IntVar()

        self.radio1 = Radiobutton(master, text="Force Setpoint", variable=self.radiovalue, value=1)
        self.radio1.grid(column=0, row=1)

        self.radio2 = Radiobutton(master, text="Mass Setpoint", variable=self.radiovalue, value=2)
        self.radio2.grid(column=0, row=2)

        self.radio3 = Radiobutton(master, text="Unknown Mass", variable=self.radiovalue, value=3)
        self.radio3.grid(column=0, row=3)

        # Use text entry to get desired force set point
        self.forcelabel = Label(master, text="Set Gripping Force (N)", font=("Times New Roman", 10), background=
                                background_colour)
        self.forcelabel.grid(column=1, row=1)

        self.forceinput = Entry(master, width=10)
        self.forceinput.grid(column=2, row=1)

        # Use text entry to get desired mass set point
        self.masslabel = Label(master, text="Input Object Mass (kg)", font=("Times New Roman", 10), background=
                               background_colour)
        self.masslabel.grid(column=1, row=2)

        self.massinput = Entry(master, width=10)
        self.massinput.grid(column=2, row=2)

        # Insert Combobox to get user to input current fingers being used
        self.fingerlabel = Label(master, text="Select Gripping Fingers", font=("Times New Roman", 16), background=
                                 background_colour)
        self.fingerlabel.grid(column=0, row=5)

        self.fingercombo = Combobox(master, state="readonly")  # Readonly so the user cannot type into the combobox
        self.fingercombo['values'] = ("Rigid", "Thin Convex", "Thin Concave", "Thick Concave")
        self.fingercombo.current(0)
        self.fingercombo.grid(column=0, row=6)
        self.fingercombo.bind("<<ComboboxSelected>>", self.update_finger_type)
        # The above bind line makes it so the finger type variable is automatically updated as soon as the user
        # makes a different selection in the combobox (is triggered by the selection event).

        # Text Entry Box
        self.objectpositionlabel = Label(master, text="Distance to Object? [cm]", font=("Times New Roman", 10),
                                         background=background_colour)
        self.objectpositionlabel.grid(column=1, row=6)

        self.objectpositioninput = Entry(master, width=10)
        self.objectpositioninput.grid(column=2, row=6)

        spacelabel1 = Label(master, text=" ", font=("Times New Roman", 7), background=background_colour)
        spacelabel1.grid(column=0, row=7)

        self.openlabel = Label(master, text="Open Gripper", font=("Times New Roman", 16), background=background_colour)
        self.openlabel.grid(column=0, row=8)

        self.openpositionlabel = Label(master, text="To an object size of [mm]:", font=("Times New Roman", 10),
                                       background=background_colour)
        self.openpositionlabel.grid(column=0, row=9)

        self.openpositionvalue = Entry(master, width=10)
        self.openpositionvalue.grid(column=1, row=9)

        self.chk_state = BooleanVar()
        self.chk_state.set(False)
        self.chk = Checkbutton(master, text='Fully Open', var=self.chk_state)
        self.chk.grid(column=2, row=9)

        # Define open, close, and stop buttons on GUI with respective object methods
        self.openbutton = Button(master, text="Open Gripper", command=self.open_button_clicked)
        self.openbutton.grid(column=4, row=9)

        self.closebutton = Button(master, text="Start Gripping", command=self.close_button_clicked)
        self.closebutton.grid(column=4, row=5)

        self.stopbutton = Button(master, text="Stop Gripping", command=self.stop_button_clicked)
        self.stopbutton.grid(column=4, row=6)

        self.rebootButton = Button(master, text="Reset Motor", command=self.reset_button_clicked)
        self.rebootButton.grid(column=4, row=2)

        # Display measurement values

        spacelabel2 = Label(master, text=" ", font=("Times New Roman", 7), background=background_colour)
        spacelabel2.grid(column=0, row=13)

        measurementslabel = Label(master, text="Measurements", font=("Times New Roman", 16), background=
                                  background_colour)
        measurementslabel.grid(column=0, row=14)

        # Display current gripper size opening - Allows updating
        self.cursizelabel = Label(master, text="Current gripper opening:", font=("Times New Roman", 10), background=
                                  background_colour)
        self.cursizelabel.grid(column=0, row=15)

        self.cursizevar = StringVar()
        self.cursizevar.set("X mm")
        self.cursize = Label(master, textvariable=self.cursizevar, font=("Times New Roman", 10), background=
                             background_colour)
        self.cursize.grid(column=1, row=15)

        # Display current distance between grippers fingers w/o deflection - Allows updating
        self.curfingersizelabel = Label(master, text="Current distance between gripper fingers \n(w/o estimated finger "
                                                     "deflection): ", font=("Times New Roman", 10), background=
                                                     background_colour, justify=CENTER)
        self.curfingersizelabel.grid(column=0, row=16)

        self.curfingersizevar = StringVar()
        self.curfingersizevar.set("X mm")
        self.curfingersize = Label(master, textvariable=self.curfingersizevar, font=("Times New Roman", 10), background=
                                   background_colour)
        self.curfingersize.grid(column=1, row=16)

        # Display current distance between gripper fingers with deflection - Allows updating
        self.curfingersize_deflect_label = Label(master, text="Current distance between gripper fingers \n(w/ estimated"
                                                              " finger deflection): ", font=("Times New Roman", 10),
                                                              background=background_colour, justify=CENTER)
        self.curfingersize_deflect_label.grid(column=0, row=17)

        self.curfingersize_deflect_var = StringVar()
        self.curfingersize_deflect_var.set("X mm")
        self.curfingersize_deflect = Label(master, textvariable=self.curfingersize_deflect_var, font=("Times New Roman",
                                                                10), background=background_colour)
        self.curfingersize_deflect.grid(column=1, row=17)

        # Display current gripping force - Allows updating
        self.curforcelabel = Label(master, text="Current gripping force:", font=("Times New Roman", 10), background=
                                   background_colour)
        self.curforcelabel.grid(column=2, row=15)

        self.curforcevar = StringVar()
        self.curforcevar.set("X N")
        self.curforce = Label(master, textvariable=self.curforcevar, font=("Times New Roman", 10), background=
                              background_colour)
        self.curforce.grid(column=2, row=16)

        # Display current gripper state - Allows updating
        self.curstatelabel = Label(master, text="Current Gripper State:", font=("Times New Roman", 10), background=
                                   background_colour)
        self.curstatelabel.grid(column=2, row=17)

        self.curstatevar = StringVar()
        self.curstatevar.set("Unknown")
        self.curstate = Label(master, textvariable=self.curstatevar, font=("Times New Roman", 10), background=
                              background_colour)
        self.curstate.grid(column=2, row=18)

        # Display current position uncertainty - Allows updating
        self.posuncertaintylabel = Label(master, text="Current position uncertainty:", font=("Times New Roman", 10),
                                         background=background_colour)
        self.posuncertaintylabel.grid(column=0, row=18)

        self.posuncertaintyvar = StringVar()
        self.posuncertaintyvar.set("+/-X mm")
        self.posuncertainty = Label(master, textvariable=self.posuncertaintyvar, font=("Times New Roman", 10),
                                    background=background_colour)
        self.posuncertainty.grid(column=1, row=18)

    # Read gripping control set point from user input, convert to a force set point if needed and pass to gripper method
    def close_button_clicked(self):
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

        # Check if "fully open" has been selected
        if self.chk_state.get():
            # position = self.kukaGripper1.maximumsize
            self.kukaGripper1.zero_gripper()
            return

        # If here, "fully open" was not selected. Retrieve the object position input by the user
        openpositionvalue = self.openpositionvalue.get()

        # Validate the user input
        inputresult = self.validate_user_input(input=openpositionvalue, fieldname="To an object of size",
                                               minvalue=0,
                                               maxvalue=self.kukaGripper1.maximumsize,
                                               valuetype="opening size", valueunit="mm")

        # If the input was valid, open the gripper
        if inputresult is not None:
            fingertype = self.fingercombo.get()
            # Convert entered object size into distance along lead screw
            leadscrew_position = (self.kukaGripper1.maximumsize - inputresult +
                                  2 * self.kukaGripper1.finger_position_offsets[fingertype]) / 2
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
        self.kukaGripper1.fingertype = self.fingercombo.get()

    def change_finger_combobox_state(self):
        print("Finger combo state: " + str(self.fingercombo['state']))

        # Check the current state of the finger combobox
        # Readonly: user can interact w/ the combobox but cannot type a custom fingertype into it
        # Disabled: user cannot interact w/ the combobox (cannot make a new selection)
        currentstate = self.fingercombo['state']

        # Change the current state
        if currentstate == 'readonly':
            self.fingercombo['state'] = "disabled"
        elif currentstate == 'disabled':
            self.fingercombo['state'] = 'readonly'

        print("Finger combo state: " + str(self.fingercombo['state']))

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