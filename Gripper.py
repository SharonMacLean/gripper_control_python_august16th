import serial  # Check that this runs properly
import time
import struct
from enum import Enum
import json

class Gripper:

    # Initialize the gripper object
    # *Attributes can be updated at anytime, do not necessarily need to be set on initialization
    def __init__(self, controller):
        self.controller = controller
        self.fingertype = None
        self.serial_communication = None
        self.gripperGui = None
        self.motorError = None

        self.status_types = Enum('status_types', 'Idle Holding Opening Closing Error')
        self.status = self.status_types.Idle

        self.positionsetpoint = None
        self.forcesetpoint = None

        # Current distance between the closest surfaces of the flexible finger bases. The flexible finger bases are the
        # metal dovetails that slide onto the QCTP)
        self.currentpositiontrue = None                # mm

        # Current distance between the gripper fingers (accounting for finger offset and
        # finger deflection
        self.currentpositionfinger = None              # mm
        self.currentpositionfinger_w_deflection = None # mm
        self.currentpositionuncertainty = None         # mm
        self.current_sensor_force = None               # N, current force measured by the load cell
        self.current_state_data = None
        self.current_flex_percent = None
        self.currentforce = None                       # N, currently applied gripping force (calculated from the load
                                                       # cell measurement)
        self.objectdistance = None                     # Object position is measured from base of Kuka wrist (this was
                                                       # stated in the close_button_clicked() method)
        self.currentservoangle = None
        self.finger_deflection = 0
        self.finger_deflection_uncertainty = 0
        self.force_uncertainty = 1.135                 # N
        self.control_input = None
        # The distance in mm that one lead screw has made it's QCTP travel (mm). Equivalent to x1 in the diagram
        self.lead_screw_position = 0
        self.force_sensor_preload = 0
        self.sensor_to_gripping_force = 1     # Updates with each new object distance

        self.first_loop = True
        self.contact_made = False
        self.counter = 0
        self.start_time = 0

        # mm. Distance between the closest surfaces of the flexible finger bases when fully opened (maximum distance).
        self.maximumsize = 150        # mm
        self.maximumforce = 73.125    # N
        self.minimumforce = 5  # N - Due to Op-Amp issues outputting near rail voltage (0V - 12V)
        self.maximummass = 3  # kg
        self.maximumobjectdist = 20  # cm TODO: update this value
        self.minimumobjectdist = 11.5  # cm TODO: update this value
        self.minimum_stable_time = 0.1  # TODO: update this value (in seconds)
        self.positionthreshold = 0.5  # mm TODO: this value needs to be decreased. The current position tolerance is only 2 mm (cause 1 mm per lead screw)
        self.hinge_distance = 11.5  # Distance from KUKA wrist to QCTP hinge (cm)
        self.hinge_to_FS = 4  # Distance from Hinge to Force Sensor (cm)

        # Finger Stiffness values from testing in the order (Linear Stiffness, Quadratic 'a' value, Quadratic 'b' value,
        #                                                    Position Uncertainty)
        # TODO: replace the Festo Flexible values with real stiffness values (just put zeros as a fake filler for now)
        self.finger_stiffness_values = dict([('Rigid', (15313140, 0, 0, 0)),
                                             ('Thin Convex', (11764.71, -0.0013, 0.1205, 0.2)),
                                             ('Thin Concave', (9345.79, -0.002, 0.1671, 0.2)),
                                             ('Thick Concave', (23640.7, -0.0001, 0.0486, 0.1)),
                                             ('Festo Adaptive', (0, 0, 0, 0))])

        # Finger offset distance (mm) from the inner surface of the flexible finger base (dwg. # ENGG4000-GR13015)
        # Positive if offset towards the center of the gripper, negative if offset in the opposite direction.
        # Verified all of the values (except for the Festo Flexible) on August 2021
        # TODO: replace the 'Festo Flexible' offset with the real value once it is successfully installed on the adapter
        # TODO: add the finger offset value for the 'Thick Concave' finger type (if it is still a valid type).
        self.finger_position_offsets = dict([('Rigid', 0), ('Thin Convex', 9), ('Thin Concave', -9),
                                             ('Festo Adaptive', 5)])

        # Any functions which Teensy should respond to must be added here, and to Teensy code
        # Bytes must line up exactly as listed here, and in the Teensy code
        self.functions_dictionary = dict([('FullyOpen', b'\x01'), ('Open', b'\x02'), ('Hold', b'\x03'),
                                          ('Loose', b'\x04'), ('GetInfo', b'\x05'),
                                          ('UpdateControl', b'\x06'), ('Close', b'\x07'),
                                          ('ResendData', b'\x08'), ('ContactMade', b'\x09'),
                                          ('SetPosition', b'\x0A'), ('SetVelocity', b'\x0B'),
                                          ('RebootMotor', b'\x0C')])

        # Any additional parameters desired must be added here and to the Teensy code
        self.parameter_dictionary = dict([('Voltage', b'\x01'), ('Current', b'\x02'),
                                          ('DutyCycle', b'\x03'), ('ThetaTarget', b'\x04'),
                                          ('ThetaActual', b'\x05'), ('OmegaTarget', b'\x06'),
                                          ('OmegaActual', b'\x07'), ('MotorError', b'\x08'),
                                          ('LoadCellOpAmpOutVoltage', b'\x09'), ('Force', b'\x0A'),
                                          ('Position', b'\x0B'), ('Deflection', b'\x0C'),
                                          ('FlexSensorOutVoltage', b'\x0D')])

        # From the A1-16 smart servo datasheet
        # These are the definitions of the motor error codes (status_error)
        self.motor_error_dictionary = dict([(b'\x00', 'No Error (White LED on)'),
                                          (b'\x01', 'Exceed Potentiometer Range Error (Blue LED on)'),
                                          (b'\x02', 'Over Voltage Limits Error (Red LED on/White LED off)'),
                                          (b'\x04', 'Over Temperature Error (Red LED on/White LED off)'),
                                          (b'\x08', 'Overload/Over-current Error (Red LED on/White LED off)'),
                                          (b'\x10', 'Reserved'),
                                          (b'\x20', 'Requested Packet Checksum Error (Green LED on)'),
                                          (b'\x40', 'Requested Packet Data Error (Green LED on)'),
                                          (b'\x80', 'Requested Packet RX FIFO Error (Green LED on)')])

        self.start_byte = b'\xff'

        # System Constants
        self.ls_lead = 15 / 360  # mm per degree
        self.ls_constant = 0.01064  # m (T/F)

        # System uncertainties
        self.servo_position_uncertainty = 0.363  # +/- Degrees
        self.gear_backlash = 0.915  # +/- Degrees
        self.lead_screw_uncertainty = 0.15 / 300  # +/- mm per mm
        self.kuka_arm_uncertainty = 0.03  # +/- mm
        self.force_sensor_uncertainty = (0.01 * 25) * 4.54  # +/- N

    def initialize_gripper(self, serial_port, baud, serial_timeout=0.01, gui=None):
        # Initialize serial communication, store in attribute
        teensy_serial_communication = serial.Serial(port=serial_port, baudrate=baud, timeout=serial_timeout)
        time.sleep(1)  # Pause for 1 second to allow serial port to stabilize
        self.serial_communication = teensy_serial_communication
        self.gripperGui = gui
        # Set any required parameters on micro ?
        # self.send_teensy_serial(function)

        # Fully open gripper to establish position
        # self.zero_gripper()

        # print line only for testing
        print("Initialized")

    def zero_gripper(self):
        func = self.functions_dictionary['FullyOpen']
        self.send_teensy_serial(func)
        self.update_status(self.status_types.Opening)
        self.positionsetpoint = 0


    def close_communications(self):
        # Close the serial port
        self.serial_communication.close()

    def send_teensy_serial(self, function, data=None):

        msg = []
        # Write header to indicate start of command, then write function byte
        msg.append(self.start_byte)
        msg.append(function)

        # Check if data bytes are being sent as well, if so, write the number of bytes then the data
        if data is not None:
            # Convert number of bytes from an int to a byte to transmit
            data_size = bytes([len(data)])
            msg.append(data_size)
            for i in data:
                if type(i) is not bytes:
                    msg.append(bytes([i]))
                else:
                    msg.append(i)
        else:
            # Number of data bytes being transmitted is zero
            msg.append(b'\x00')

        tempMsg = bytes([])
        tempMsg = tempMsg.join(msg)
        #print(tempMsg)
        self.serial_communication.write(tempMsg)

        # Just for testing
        #print("Data Sent")

    # Used to retrieve data from Teensy in Serial port, returns None if no data is available
    # Retrieves data as a list [function_byte, data_received] where data_received may be None
    # If no serial data is waiting in buffer, function will return None
    def get_teensy_serial(self):
        # Check if serial data is available, if so, read the data
        if self.serial_communication.in_waiting > 0:
            # Initialize flags

            start_byte_received = False
            function_received = False
            # Initialize value in case only a start byte is transmitted
            function_byte = None
            data_received = None

            # Loop through available bytes
            for c in range(self.serial_communication.in_waiting):
                nextByte = self.serial_communication.read(1)
                #print(nextByte)
                # If the start byte and function bytes have already been read, the following byte must be the number
                # of data bytes transmitted
                if function_received:
                    # Need to unpack num_data from byte to a 8-bit integer (char)
                    num_data_byte = nextByte
                    num_data = struct.unpack('B', num_data_byte)

                    # Read the number of bytes transmitted
                    if num_data == 0:
                        data_received = None
                        break
                    else:
                        data_received = self.serial_communication.read(num_data[0])
                        break

                # If start byte has been received, following byte will be the function byte
                if start_byte_received:
                    function_byte = nextByte
                    function_received = True

                # Check for start byte
                if nextByte == self.start_byte:
                    start_byte_received = True

            if function_byte is None:
                print("Error in message transmission")

            message_received = [function_byte, data_received]
        else:
            message_received = None

        return message_received

    # Returns requested values as floats
    def get_info(self, parameters):
        # Initialize list
        parameter_bytes = []

        # Ensure all requested parameters are valid requests
        for param in parameters:
            if param not in self.parameter_dictionary:
                print("Value requested which is not a valid parameter: " + param)
                exit()
            else:
                parameter_bytes.append(self.parameter_dictionary[param])
                #print(self.parameter_dictionary[param])
                #print(parameter_bytes)

        # If all items are valid, prompt Teensy to get requested values
        self.send_teensy_serial(self.functions_dictionary['GetInfo'], parameter_bytes)

        # Initialize values
        waiting_for_data = True
        attempts = 0
        message_recieved = None

        # Wait for Teensy to return system parameters
        while waiting_for_data:
            #time.sleep(0.01)
            #print(self.serial_communication.in_waiting)
            message_recieved = self.get_teensy_serial()

            if message_recieved is not None:
                waiting_for_data = False

            # Function to break infinite loop if microcontroller gets disconnected
            #attempts = attempts + 1
            #if attempts > 10:
            #    temp = input("Microcontroller not responding, Exit(Y/N)")
            #    if temp == "y":
            #        exit()
        #print("after while")
        # Get values from list, values are hex
        returned_values_bytes = message_recieved[1]
        #print(returned_values_bytes)
        #print("Before unpack in get_info")
        # Use method to convert bytes to floats and integers
        returned_values = self.unpack_serial_data(returned_values_bytes, parameters)
        #print("After unpack")
        #print("Returned values: " + str(returned_values))
        #if parameters == 'LoadCellOpAmpOutVoltage':
           #print("Returned values: " + str(returned_values))

        return returned_values

    def stop_gripper_loose(self):
        self.send_teensy_serial(self.functions_dictionary['Loose'])
        self.update_status(self.status_types.Idle)

    def stop_gripper_hold(self):
        self.send_teensy_serial(self.functions_dictionary['Hold'])
        self.update_status(self.status_types.Holding)
        func = self.parameter_dictionary['Hold']
        self.send_teensy_serial(func)

    def open_gripper(self, position):
            position_bytes = struct.pack('f', position)

            # Update status and prompt Teensy to open
            self.update_status(self.status_types.Opening)
            self.send_teensy_serial(self.functions_dictionary['Open'], position_bytes)
            self.positionsetpoint = position

    def close_gripper(self, objectdistance, forcesetpoint, gui=None):
        self.objectdistance = objectdistance
        self.forcesetpoint = forcesetpoint

        # Update effect of moment arm on force measurement
        self.sensor_to_gripping_force = self.hinge_to_FS / (self.objectdistance - self.hinge_distance)

        # Update Current sensor preload value
        self.force_sensor_preload = self.current_sensor_force * self.sensor_to_gripping_force

        # Update setpoint for controller
        self.controller.setpoint = self.forcesetpoint

        # Pass parameters and prompt Teensy to start gripping
        self.send_teensy_serial(self.functions_dictionary['Close'])
        self.update_status(self.status_types.Closing)

        # Update gui status
        if gui is not None:
            gui.curstatevar.set(self.status.name)

        # Set flags prior to loop
        self.first_loop = True
        self.contact_made = False
        self.counter = 0
        self.start_time = 0

        # Continue updating control input to Teensy until response within acceptable range
        # UPDATE if teensy determines end of control loop instead of python

    # This method updates the position variables and calculates their corresponding uncertainty values.
    def update_pos(self):
        tempPos = self.get_info(["Position"])
        self.lead_screw_position = tempPos[0][0]

        # Get parameters for finger deflections depending on which finger set is being used
        # Estimate finger deflection from quadratic finger stiffness curve-fitting
        self.finger_deflection = self.finger_stiffness_values[self.fingertype][1] * (self.currentforce ** 2) + \
            self.finger_stiffness_values[self.fingertype][2] * self.currentforce

        # Finger Deflection uncertainty in mm (From force sensor error and curve-fitting error)
        self.finger_deflection_uncertainty = \
            2 * self.finger_stiffness_values[self.fingertype][1] * self.currentforce * self.force_uncertainty + \
            self.finger_stiffness_values[self.fingertype][1] * (self.force_uncertainty ** 2) + \
            self.finger_stiffness_values[self.fingertype][2] * self.force_uncertainty + \
            self.finger_stiffness_values[self.fingertype][3]

        #print(self.fingertype)
        #print(self.finger_position_offsets[self.fingertype])

        # Update the current position of the QCTP, and the gripping fingers
        self.currentpositiontrue = self.maximumsize - 2 * self.lead_screw_position
        self.currentpositionfinger = self.currentpositiontrue - 2*self.finger_position_offsets[self.fingertype]
        self.currentpositionfinger_w_deflection = self.currentpositionfinger + 2 * self.finger_deflection

        # Calculate uncertainty over current position estimate
        self.currentpositionuncertainty = 2 * (self.ls_lead * (self.servo_position_uncertainty + self.gear_backlash) +
                                               self.lead_screw_position * self.lead_screw_uncertainty +
                                               self.finger_deflection_uncertainty)

    #@staticmethod
    def unpack_serial_data(self, data_received, parameters):
        # Initialize values
        char_pos = 0
        unpacked_data = []
        num_bytes = 4

        for n in range(len(parameters)):
            if parameters[n] == 'MotorError':
                unpacked_data.append(data_received[char_pos])
                char_pos += 1
            else:
                # Splice off data, 4 bytes at a time
                value_bytes = data_received[char_pos:(char_pos + num_bytes)]
                #print("Value_bytes: ")
                #print(value_bytes)
                #print("Before unpack")
                value = None

                # Convert back to a float
                value = struct.unpack('f', value_bytes)
                #print("After unpack")
                # Add converted value to list of values to return
                unpacked_data.append(value)

                # Update character position for next loop
                char_pos = char_pos + num_bytes

        return unpacked_data

    def reset_motor(self):
        self.send_teensy_serial(self.functions_dictionary['RebootMotor'])

    def update_status(self, new_status):
        self.status = new_status
        self.gripperGui.change_finger_combobox_state(gripper_status=new_status)

    def set_finger_type(self, fingertype_new):
        # Update the finger type variable
        self.fingertype = fingertype_new
        # Update controller with current gripping fingers to get new gains
        self.controller.update_finger_gains(fingertype=self.fingertype)

    def gripper_loop(self):
        if self.status == self.status_types.Opening:
            # Update current position
            self.update_pos()

            # If the current position of the gripper is within the position tolerance, loosen the gripper.
            if abs(self.lead_screw_position-self.positionsetpoint) < self.positionthreshold:
                self.update_status(self.status_types.Idle)
                func = self.functions_dictionary['Loose']
                self.send_teensy_serial(func)

        if self.status == self.status_types.Holding:
            func = ['Force']
            force = self.get_info(func)

            # TODO: Remove this and just let it update normally in the main loop of Main.py?
            # TODO: Why isn't this value multiplied by the constants the other one later in the loop is?
            # TODO: Since it is not multiplied, it is equal to just the force measured at the load cell, not the actual
            # gripper finger.
            # TODO: This doesn't update the self.current_force variable
            # self.gripperGui.curforcevar.set(str(force)+' N')
            # formatted the print better for now
            self.gripperGui.curforcevar.set(("{:." + str(3) + "f}").format(force) + " N")

        if self.status == self.status_types.Closing:
            # Wait until gripper has contacted object to begin supplying control inputs
            if not self.contact_made:
                teensy_update = self.get_teensy_serial()

                # Should we include some sort of timeout situation here ( what criteria? if t>t_fullclose: exit ? )
                if teensy_update is not None:
                    if teensy_update[0] == self.functions_dictionary['ContactMade']:
                        self.contact_made = True
                    return

            # Update current sensor force and position
            getparameters = ['Force', 'Position']
            [self.current_sensor_force, self.lead_screw_position] = self.get_info(parameters=getparameters)
            self.currentforce = self.current_sensor_force[0] * self.sensor_to_gripping_force - self.force_sensor_preload

            # Update current object size, finger deflection and position uncertainty
            self.update_pos()

            # Reset controller previous time step values if first loop of new control loop
            if self.first_loop:
                self.controller.new_control_loop()
                self.first_loop = False

            # Update control action
            self.control_input = self.controller.pi_control(currentvalue=self.currentforce)

            # Need to convert control action (Torque setpoint) to a byte to send to Teensy
            control_bytes = struct.pack('f', self.control_input)

            # Send update to Teensy
            self.send_teensy_serial(self.functions_dictionary['UpdateControl'], control_bytes)

            # Check if control response has settled within an acceptable band
            if (-self.force_uncertainty) <= (self.forcesetpoint - self.currentforce) <= self.force_uncertainty:
                self.counter = self.counter + 1
                if self.counter == 1:
                    self.start_time = time.perf_counter()
                else:
                    cur_time = time.perf_counter()

                    # Check if control response has been stable for a sufficient time
                    if (cur_time - self.start_time) >= self.minimum_stable_time:

                        # Update teensy to keep current position setpoint
                        self.send_teensy_serial(self.functions_dictionary['Hold'])
                        self.update_status(self.status_types.Holding)

            else:
                self.counter = 0
        if self.status == self.status_types.Idle:
            pass

        # Update the force and motor error variables
        self.current_state_data = self.get_info(['Force', 'MotorError'])
        self.current_sensor_force = self.current_state_data[0][0]
        #print("Raw force read in: " + str(self.current_sensor_force))
        self.currentforce = self.current_sensor_force * self.sensor_to_gripping_force - self.force_sensor_preload
        self.motorError = self.current_state_data[1]

        self.current_flex_percent = self.get_info(['Deflection'])[0][0]
        print("Flex amount: " + str(self.current_flex_percent))

        current_flex_voltage = self.get_info(['FlexSensorOutVoltage'])
        print("Flex sensor voltage: " + str(current_flex_voltage[0][0]))

        #Temp output the measured op amp voltage:
        #print("Measured op amp voltage: " + str(temp_data))
        #print("Lead screw pos: " + str(self.lead_screw_position))

        # If a motor error is reported, change the gripper status to 'Error'
        if self.motorError is not 0:
            self.update_status(self.status_types.Error)
        # If no motor error is reported but the gripper status is Error, change it back to Idle
        # TODO: This could be problematic if an error was reported for a quick sec while closing or holding
        # TODO: This might need to be changed
        #else:
         #   if self.status == self.status_types.Error:
         #       self.update_status("Idle")
