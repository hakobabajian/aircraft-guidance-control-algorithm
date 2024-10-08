import krpc
# from krpc import VesselSituation
import time

connection = krpc.connect()


# vessel = connection.space_center.active_vessel
# while True:
#     ref = connection.space_center.ReferenceFrame.create_hybrid(
#         position=vessel.orbit.body.reference_frame,
#         rotation=vessel.surface_reference_frame)
#     # print(vessel.flight(ref).latitude, vessel.flight(ref).longitude)
#     # print(vessel.recoverable, vessel.situation, vessel.crew_count)
#     # print(connection.space_center.active_vessel)
#     print(vessel.situation)
#     if vessel.situation.name == "splashed":
#         print("SPLASHED")
#     if vessel.crew_count == 0:
#         connection.space_center.revert_to_launch()
#         vessel.control.activate_next_stage()
#         # connection.space_center.launch_vessel()
#     time.sleep(1)

class Vessel:
    def __init__(self, conn, cruise_altitude=100, cruise_speed=100, cruise_acceleration=10, time_step=0.01,
                 control_step=0.01):
        self.vessel = conn.space_center.active_vessel
        self.ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
            position=self.vessel.orbit.body.reference_frame,
            rotation=self.vessel.surface_reference_frame)
        self.cruise_altitude = cruise_altitude
        self.cruise_speed = cruise_speed
        self.cruise_acceleration = cruise_acceleration
        self.t = time_step
        self.control_step = control_step
        self.altitude_offset = 0
        self.max_target_speed = 100
        self.max_take_off_vertical_speed = 15
        self.lift_off_speed = 40
        self.runway_center = -0.0493255
        self.runway_end = -74.5
        self.runway_heading = 270
        self.correction_angle = 40
        self.max_correction_roll = 15
        self.min_correction_roll = 5
        self.max_latitude_difference = 0.15
        self.approach_altitude = self.cruise_altitude
        self.approach_speed = 80
        self.approach_offset = 1
        self.landing_altitude = 100
        self.landing_offset = .25
        self.landing_pitch = 5

    """ 
    Maneuver methods
    ----------------
    Each maneuver method includes a while-loop which iterates until the maneuver's end condition (i.e. altitude > target). 
    The while-loop serves as a numeric simulation, containing helper methods which calculate time derivatives over the 
    time step 'self.t' and make their respective changes to the vessel's controls (i.e. pitch-up or pitch-down)
    """
    def start_engine(self):
        # Brings vessel up to take-off speed on the runway
        self.vessel.control.activate_next_stage()
        self.vessel.control.brakes = False
        initial_speed = self.vessel.flight(self.ref_frame).speed
        while self.vessel.flight(self.ref_frame).speed < self.lift_off_speed:
            initial_speed = self.control_quantity("speed", initial_speed, self.cruise_speed, self.cruise_acceleration,
                                                  "throttle")
            time.sleep(self.t)

    def take_off(self, altitude_quantity="mean_altitude"):
        # Guides vessel pitch until reaching cruise altitude
        self.start_engine()
        initial_speed = self.vessel.flight(self.ref_frame).speed
        altitude_derivatives = self.get_initial_derivatives(altitude_quantity, 4)
        while getattr(self.vessel.flight(self.ref_frame), altitude_quantity) < self.cruise_altitude:
            initial_speed = self.control_quantity("speed", initial_speed, self.cruise_speed, self.cruise_acceleration,
                                                  "throttle")
            target_climb_speed = self.max_take_off_vertical_speed
            altitude_derivatives = self.angular_control_from_position(
                altitude_quantity, altitude_derivatives, target_climb_speed, "pitch",
                self.control_step, sensitivity=1)
            time.sleep(self.t)
        print("Take-off Complete")

    def cruise(self, altitude_quantity="mean_altitude", longitude_bound=-74.3, direction=1):
        # Guides vessel with constant speed, altitude, and roll angle of zero (straight, level flight)
        self.vessel.control.gear = False
        initial_speed = self.vessel.flight(self.ref_frame).speed
        altitude_derivatives = self.get_initial_derivatives(altitude_quantity, 4)
        roll_derivatives = self.get_initial_derivatives("roll", 4)
        error = []
        while self.vessel.flight(self.ref_frame).longitude < longitude_bound:
            initial_speed = self.control_quantity("speed", initial_speed, self.cruise_speed, self.cruise_acceleration,
                                                  "throttle")
            target_climb_speed = self.get_quadratic_target_quantity_velocity(altitude_quantity, self.cruise_altitude,
                                                                             self.max_target_speed)
            target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", 0,
                                                                                      self.max_target_speed / 4,
                                                                                      anti_sensitivity=0.1)
            # print(self.vessel.flight(self.ref_frame).longitude)
            altitude_derivatives = self.angular_control_from_position(
                altitude_quantity, altitude_derivatives, target_climb_speed, "pitch", self.control_step, sensitivity=2)
            error.append(abs(altitude_derivatives[1] - self.vessel.flight(self.ref_frame).velocity[0]))
            print(sum(error) / len(error))
            roll_derivatives = self.angular_control_from_position(
                'roll', roll_derivatives, target_roll_speed, "roll", self.control_step / 10, sensitivity=0.5)
            time.sleep(self.t)

    def runway_alignment_correction(self, altitude_quantity="mean_altitude", direction=1):
        """
        This method is fairly involved. It guides the vessel from a trajectory parallel to the runway to one directly on
        top of it by making an S-shaped maneuver, then goes for the landing. The aggressiveness by which it corrects
        its heading, roll, altitude, and speed is proportionate to how close it is the runway in the direction of the
        landing, however there may be more appropriate quantities to make these corrections proportionate to.
        The S-shaped maneuver is repeated until the landing is complete.
        """
        self.vessel.control.gear = False
        initial_speed = self.vessel.flight(self.ref_frame).speed
        altitude_derivatives = self.get_initial_derivatives(altitude_quantity, 4)
        roll_derivatives = self.get_initial_derivatives("roll", 4)
        initial_latitude = self.vessel.flight(self.ref_frame).latitude
        correction_roll = 200 * abs(self.runway_center - initial_latitude)
        if self.vessel.flight(self.ref_frame).latitude < self.runway_end:
            self.min_correction_roll = 1.5
        if correction_roll > self.max_correction_roll:
            correction_roll = self.max_correction_roll
        elif correction_roll < self.min_correction_roll:
            correction_roll = self.min_correction_roll
        half_turn_di = 0.25
        speed = self.cruise_speed
        altitude = self.cruise_altitude
        a_s = 3.0
        s = 2.0
        print(f"Correction Roll: {correction_roll}")
        if self.vessel.flight(self.ref_frame).latitude > self.runway_center:
            while self.vessel.flight(self.ref_frame).latitude > self.runway_center:
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end + self.approach_offset:
                    speed = self.approach_speed
                    altitude = self.approach_altitude
                    self.correction_angle = 10
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end + self.landing_offset:
                    altitude = self.landing_altitude
                    self.approach_speed = 40
                    s = 0.75
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end and \
                        self.vessel.flight(self.ref_frame).speed > self.approach_speed:
                    altitude_quantity = "surface_altitude"
                    altitude = 35
                    speed = 20
                    a_s = 1.5
                    s = 0.5
                    self.vessel.control.gear = True
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end and \
                        self.vessel.flight(self.ref_frame).speed < self.approach_speed:
                    print("LANDING")
                    a_s = 1
                    self.vessel.control.gear = True
                    speed = 27
                    s = 0.15
                    altitude = 25
                if self.vessel.flight(self.ref_frame).surface_altitude < 2:
                    altitude = 1
                    self.vessel.control.brakes = True
                    speed = 0

                target_climb_speed = self.get_quadratic_target_quantity_velocity(altitude_quantity, altitude,
                                                                                 self.max_target_speed)
                altitude_derivatives = self.angular_control_from_position(
                    altitude_quantity, altitude_derivatives, target_climb_speed, "pitch", self.control_step,
                    sensitivity=a_s)
                initial_speed = self.control_quantity("speed", initial_speed, speed, self.cruise_acceleration,
                                                      "throttle")

                if self.vessel.flight(self.ref_frame).latitude > initial_latitude:
                    initial_latitude = self.vessel.flight(self.ref_frame).latitude
                di = (self.runway_center - self.vessel.flight(self.ref_frame).latitude) / \
                     (self.runway_center - initial_latitude)
                if self.vessel.flight(self.ref_frame).heading > self.runway_heading - self.correction_angle / 2 and \
                        di > 0.75:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", -correction_roll,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("M1 pre", di)
                    half_turn_di = 1.0 - di
                elif self.vessel.flight(self.ref_frame).heading < self.runway_heading - self.correction_angle / 2 and \
                        di > 0.75:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", 0,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("M1 post", di)
                elif di > 8 * half_turn_di:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", 0,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("Z1", di)
                elif 8 * half_turn_di > di and \
                        self.vessel.flight(self.ref_frame).heading < self.runway_heading - self.correction_angle / 2:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", correction_roll,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("Z2", di)
                elif 8 * half_turn_di > di and \
                        self.vessel.flight(self.ref_frame).heading > self.runway_heading - self.correction_angle / 2:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", 0,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    if self.vessel.flight(self.ref_frame).roll < 1:
                        break
                    print("M2", di)
                else:
                    target_roll_speed = 0
                roll_derivatives = self.angular_control_from_position(
                    'roll', roll_derivatives, target_roll_speed, "roll", self.control_step / 8, sensitivity=s)
        else:
            while self.vessel.flight(self.ref_frame).latitude < self.runway_center:
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end + self.approach_offset:
                    speed = self.approach_speed
                    altitude = self.approach_altitude
                    self.correction_angle = 10
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end + self.landing_offset:
                    altitude = self.landing_altitude
                    self.approach_speed = 40
                    s = 0.75
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end and \
                        self.vessel.flight(self.ref_frame).speed > self.approach_speed:
                    altitude_quantity = "surface_altitude"
                    altitude = 35
                    speed = 40
                    a_s = 1.5
                    s = 0.5
                    self.vessel.control.gear = True
                if self.vessel.flight(self.ref_frame).longitude < self.runway_end and \
                        self.vessel.flight(self.ref_frame).speed < self.approach_speed:
                    print("LANDING")
                    self.vessel.control.gear = True
                    speed = 27
                    s = 0.15
                    a_s = 1.0
                    altitude = 25
                if self.vessel.flight(self.ref_frame).surface_altitude < 2:
                    altitude = 1
                    self.vessel.control.brakes = True
                    speed = 0
                target_climb_speed = self.get_quadratic_target_quantity_velocity(altitude_quantity, altitude,
                                                                                 self.max_target_speed)
                altitude_derivatives = self.angular_control_from_position(
                    altitude_quantity, altitude_derivatives, target_climb_speed, "pitch", self.control_step,
                    sensitivity=a_s)
                initial_speed = self.control_quantity("speed", initial_speed, speed, self.cruise_acceleration,
                                                      "throttle")
                if self.vessel.flight(self.ref_frame).latitude < initial_latitude:
                    initial_latitude = self.vessel.flight(self.ref_frame).latitude
                di = (self.runway_center - self.vessel.flight(self.ref_frame).latitude) / \
                     (self.runway_center - initial_latitude)
                if self.vessel.flight(self.ref_frame).heading < self.runway_heading + self.correction_angle / 2 and \
                        di > 0.75:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", correction_roll,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("M1 pre", di)
                    half_turn_di = 1.0 - di
                elif self.vessel.flight(self.ref_frame).heading > self.runway_heading + self.correction_angle / 2 and \
                        di > 0.75:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", 0,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("M1 post", di)
                elif di > 8 * half_turn_di:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", 0,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("Z1", di)
                elif 8 * half_turn_di > di and \
                        self.vessel.flight(self.ref_frame).heading > self.runway_heading + self.correction_angle / 2:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", -correction_roll,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    print("Z2", di)
                elif 8 * half_turn_di > di and \
                        self.vessel.flight(self.ref_frame).heading < self.runway_heading + self.correction_angle / 2:
                    target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", 0,
                                                                                              self.max_target_speed / 4,
                                                                                              anti_sensitivity=0.1)
                    if self.vessel.flight(self.ref_frame).roll < 1:
                        break
                    print("M2", di)
                else:
                    target_roll_speed = 0
                roll_derivatives = self.angular_control_from_position(
                    'roll', roll_derivatives, target_roll_speed, "roll", self.control_step / 8, sensitivity=s)

    def turn(self, altitude_quantity="mean_altitude", heading_limit=260, turning_speed=65, roll_angle=30, offset=10):
        # Guides vessel through banked turn with a constant speed. Roll angle based on progress to desired heading
        self.vessel.control.gear = False
        initial_speed = self.vessel.flight(self.ref_frame).speed
        altitude_derivatives = self.get_initial_derivatives(altitude_quantity, 4)
        roll_derivatives = self.get_initial_derivatives("roll", 4)
        initial_heading = self.vessel.flight(self.ref_frame).heading
        while self.vessel.flight(self.ref_frame).heading < heading_limit:
            initial_speed = self.control_quantity("speed", initial_speed, turning_speed, self.cruise_acceleration,
                                                  "throttle")
            target_climb_speed = self.get_quadratic_target_quantity_velocity(altitude_quantity, self.cruise_altitude,
                                                                             self.max_target_speed)
            roll = self.get_roll_angle_from_heading(initial_heading, heading_limit, roll_angle, offset)
            target_roll_speed = self.get_symmetric_quadratic_target_quantity_velocity("roll", roll,
                                                                                      self.max_target_speed / 4,
                                                                                      anti_sensitivity=0.1)
            altitude_derivatives = self.angular_control_from_position(
                altitude_quantity, altitude_derivatives, target_climb_speed, "pitch", self.control_step, sensitivity=2.5)
            roll_derivatives = self.angular_control_from_position(
                'roll', roll_derivatives, target_roll_speed, "roll", self.control_step / 10, sensitivity=0.5)
            time.sleep(self.t)

    """
    Helper Methods - Calculate Target Quantity
    ----------------
    These methods return target quantities (i.e. altitude velocity) to help Control Methods make changes to the vessel's 
    controls (i.e. pitch-up) proportionate the difference of target quantity and actual. For example the further the 
    vessel's target altitude is from its actual, the greater the target altitude velocity will be. This relation is 
    non-linear, quadratic by default. Continuing with this example, once the target altitude velocity is given 
    as a parameter to a Control Method, the further the target altitude velocity is from the vessel's actual altitude 
    velocity, the greater the change made the vessel's pitch. This relation is also non-linear, quadratic by default.
    """

    def get_roll_angle_from_heading(self, initial_heading, final_heading, max_roll, offset=10):
        current_heading = self.vessel.flight(self.ref_frame).heading
        if current_heading < initial_heading:
            return max_roll
        return (-max_roll / (final_heading - initial_heading)**3) * (current_heading - initial_heading - offset)**3 + max_roll

    def get_symmetric_quadratic_target_quantity_velocity(self, quantity_name, quantity_midpoint, velocity_bound,
                                                         multiplicity=2, anti_sensitivity=0.1):
        target_velocity = (getattr(self.vessel.flight(self.ref_frame), quantity_name) -
                           quantity_midpoint) ** multiplicity * velocity_bound / anti_sensitivity
        if getattr(self.vessel.flight(self.ref_frame), quantity_name) > quantity_midpoint:
            target_velocity = -1 * target_velocity
        if target_velocity < -velocity_bound:
            target_velocity = -velocity_bound
        if target_velocity > velocity_bound:
            target_velocity = velocity_bound
        return target_velocity

    def get_quadratic_target_quantity_velocity(self, quantity_name, quantity_midpoint, velocity_bound, multiplicity=2,
                                               sensitivity=2.0):
        target_velocity = (getattr(self.vessel.flight(self.ref_frame), quantity_name) -
                           quantity_midpoint) ** multiplicity * velocity_bound / quantity_midpoint / \
                          (quantity_midpoint / sensitivity)
        if getattr(self.vessel.flight(self.ref_frame), quantity_name) > quantity_midpoint:
            target_velocity = -1 * target_velocity
        if target_velocity < -sensitivity * velocity_bound:
            target_velocity = -sensitivity * velocity_bound
        if target_velocity > sensitivity * velocity_bound:
            target_velocity = sensitivity * velocity_bound
        return target_velocity

    def get_quadratic_target_angular_control(self, velocity, velocity_bound, control_magnitude, multiplicity=2,
                                             sensitivity=2):
        # Helper method of control method
        control = abs((velocity - velocity_bound) ** multiplicity * control_magnitude / velocity_bound /
                      (velocity_bound / sensitivity))
        if control > sensitivity * control_magnitude:
            control = sensitivity * control_magnitude
        return control

    """
    Helper Methods - Calculate Numeric Derivatives
    ----------------
    These methods calculate numeric time derivatives of various quantities (i.e. altitude, roll) over the time step 
    'self.t'. Time derivatives are calculated for control method's PID algorithm, allowing vessel control changes to 
    result in vessel's smooth flight.
    """

    def get_initial_derivatives(self, quantity_name, n):
        """ Returns list of initial derivatives of a given quantity (i.e. altitude) from 0th to nth.
            Used at the beginning of maneuver methods to begin numeric simulation"""
        derivatives_matrix = []
        zeroth_derivatives = []

        for i in range(n + 1):
            zeroth_derivatives.append(getattr(self.vessel.flight(self.ref_frame), quantity_name))
            time.sleep(self.t)
        derivatives_matrix.append(zeroth_derivatives)

        for derivatives in derivatives_matrix:
            if len(derivatives) == 1:
                break
            next_derivatives = []
            for i, derivative in enumerate(derivatives):
                if i < len(derivatives) - 1:
                    next_derivatives.append((derivatives[i + 1] - derivatives[i]) / self.t)
            derivatives_matrix.append(next_derivatives)

        initial_derivatives = [sum(derivatives) / len(derivatives) for derivatives in derivatives_matrix]
        return initial_derivatives

    def get_time_derivative(self, quantity_name, quantity_before_step):
        # Helper method of control method
        quantity_current = getattr(self.vessel.flight(self.ref_frame), quantity_name)
        instantaneous_time_derivative = (quantity_current - quantity_before_step) / self.t
        return instantaneous_time_derivative, quantity_current

    def get_time_derivatives(self, quantity_name, initial_derivatives):
        # Helper method of control method
        final_derivatives = [getattr(self.vessel.flight(self.ref_frame), quantity_name)]
        for initial_derivative in initial_derivatives:
            final_derivatives.append((final_derivatives[-1] - initial_derivative) / self.t)
        return final_derivatives

    """
    Helper Methods - Control Methods
    ----------------
    These methods call those helper methods which calculate time derivatives, then based on time derivative values, make
    changes to vessel controls.
    """

    def control_quantity(self, quantity_name, quantity_before_step, quantity_bound, time_derivative_bound,
                         control_name):
        time_derivative, quantity_current = self.get_time_derivative(quantity_name, quantity_before_step)
        if quantity_current < quantity_bound and time_derivative < time_derivative_bound:
            setattr(self.vessel.control, control_name, getattr(self.vessel.control, control_name) + self.control_step)
        else:
            setattr(self.vessel.control, control_name, getattr(self.vessel.control, control_name) - self.control_step)
        return quantity_current

    def angular_control_from_position(self, quantity_name, derivatives, speed_bound, control_name, control_magnitude,
                                      sensitivity):
        derivatives = self.get_time_derivatives(quantity_name, derivatives)
        if derivatives[1] > speed_bound and derivatives[2] > 0 > derivatives[3] and derivatives[4] < 0 and \
                derivatives[5] < 0:
            control = self.get_quadratic_target_angular_control(derivatives[1], speed_bound, control_magnitude,
                                                                sensitivity=sensitivity)
            setattr(self.vessel.control, control_name, getattr(self.vessel.control, control_name) - control)
        elif derivatives[1] < speed_bound and derivatives[2] < 0 < derivatives[3] and derivatives[4] > 0 and \
                derivatives[5] > 0:
            control = self.get_quadratic_target_angular_control(derivatives[1], speed_bound, control_magnitude,
                                                                sensitivity=sensitivity)
            setattr(self.vessel.control, control_name, getattr(self.vessel.control, control_name) + control)
        return derivatives[:-1]


vessel = Vessel(connection, cruise_altitude=100, cruise_speed=80, time_step=0.005, cruise_acceleration=150,
                control_step=0.02)
altitude_name = "mean_altitude"
print("Initiating Take Off")
vessel.take_off(altitude_name)
print("Take Off Complete")
vessel.cruise(longitude_bound=-73.2, altitude_quantity=altitude_name, direction=-1)
# print("Initiating Turn")
# vessel.turn(altitude_quantity=altitude_name, heading_limit=270, turning_speed=100, roll_angle=45, offset=15)
# print("===== Turn Complete =====")
# print("==== Runway Approach ====")
# while vessel.vessel.flight(vessel.ref_frame).speed > 1.0:
#     vessel.runway_alignment_correction()
# print("===== Landing Complete =====")
