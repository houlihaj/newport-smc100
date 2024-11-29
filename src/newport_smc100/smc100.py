import logging
import functools
import time
from enum import Enum
from typing import Optional, Callable, Any
import serial


def logging_decorator(func: Callable) -> Callable:
    """
    Logging method arguments as DEBUG logs and logging exceptions with
    an exception traceback.

    https://ankitbko.github.io/blog/2021/04/logging-in-python/

    :param func: The input function, or method, whose behavior
        will be extended by the logging decorator.
    :type func: Callable
    :return: The new function that extends the behavior of the
        input function without modifying its original behavior.
        In this case, DEBUG and ERROR messages will be logged.
    :return: Callable
    """
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        args_repr: list = [repr(a) for a in args]
        kwargs_repr: list = [f"{k}={v!r}" for k, v in kwargs.items()]
        signature: str = ", ".join(args_repr + kwargs_repr)
        logging.debug(f"function {func.__name__} called with args {signature}")
        try:
            result = func(*args, **kwargs)
            return result
        except Exception as e:
            logging.exception(
                f"Exception raised in {func.__name__}. exception: {str(e)}"
            )

    return wrapper


def retry(retries: int = 3, delay: float = 1) -> Callable:
    """
    Attempt to call a function, if it fails, try again with a specified delay.

    :param retries: The max amount of retries you want for the function call
    :type retries: int
    :param delay: The delay (in seconds) between each function retry
    :type delay: float
    :return:
    :rtype: Callable
    """

    # Do not let the user use this decorator if they are high
    if retries < 1 or delay <= 0:
        raise ValueError("Are you high, mate?")

    def decorator(func: Callable) -> Callable:
        @functools.wraps(func)
        def wrapper(*args, **kwargs) -> Any:
            for i in range(1, retries + 1):  # 1 to retries + 1 since upper bound is exclusive

                try:
                    args_repr: list = [repr(a) for a in args]
                    kwargs_repr: list = [f"{k}={v!r}" for k, v in kwargs.items()]
                    signature: str = ", ".join(args_repr + kwargs_repr)
                    # logging.debug(f"Running ({i}): {func.__name__}()")
                    logging.debug(f"Running ({i}): function {func.__name__} called with args {signature}")
                    return func(*args, **kwargs)

                except Exception as e:
                    # Break out of the loop if the max amount of retries is exceeded
                    if i == retries:
                        logging.error(f"Error raised in {func.__name__}: {repr(e)}.")
                        logging.exception(f"'{func.__name__}()' failed after {retries} retries.")
                        break
                    else:
                        logging.error(f"Error raised in {func.__name__}: {repr(e)} -> Retrying...")
                        time.sleep(delay)  # Add a delay before running the next iteration

        return wrapper

    return decorator


class InstrumentException(Exception):
    def __init__(self, message):
        super().__init__(message)


class InstrumentWarning(Warning):
    def __init__(self, message):
        super().__init__(message)


class NullCharacterReceivedWarning(InstrumentWarning):
    pass


class EmptyResponseWarning(InstrumentWarning):
    pass


class HomeSearchType(Enum):
    MZ_ENCODER = 0
    HOME = 1
    MZ_ONLY = 2
    EOR_ENCODER = 3
    EOR_ONLY = 4


class PositionerErrorCode(Enum):
    NEGATIVE_END_OF_RUN = 1
    POSITIVE_END_OF_RUN = 2
    PEAK_CURRENT_LIMIT = 4
    RMS_CURRENT_LIMIT = 8
    SHORT_CIRCUIT_DETECTION = 16
    FOLLOWING_ERROR = 32
    HOMING_TIME_OUT = 64
    WRONG_ESP_STAGE = 128
    DC_VOLTAGE_TOO_LOW = 256
    EIGHTY_WATT_OUTPUT_POWER_EXCEEDED = 512


class ControllerState(Enum):
    INVALID = ""
    NOT_REFERENCED_RESET = "0A"
    NOT_REFERENCED_HOMING = "0B"
    NOT_REFERENCED_CONFIGURATION = "0C"
    NOT_REFERENCED_DISABLE = "0D"
    NOT_REFERENCED_READY = "0E"
    NOT_REFERENCED_MOVING = "0F"
    NOT_REFERENCED_ESP = "10"  # stage error
    NOT_REFERENCED_JOGGING = "11"
    CONFIGURATION = "14"
    HOMING = "1E"
    MOVING = "28"
    READY_HOMING = "32"
    READY_MOVING = "33"
    READY_DISABLE = "34"
    READY_JOGGING = "35"
    DISABLE_READY = "3C"
    DISABLE_MOVING = "3D"
    DISABLE_JOGGING = "3E"
    JOGGING_READY = "46"
    JOGGING_DISABLE = "47"


class ControlLoopState(Enum):
    OPEN = 0
    CLOSED = 1


class SMC100:
    def __init__(self, controller_address: int):
        """

        :param controller_address: the controller address on the RS-485
            communication network
        :type controller_address: int
        """
        self.controller_address: int = controller_address

        self.dev: Optional[serial.Serial] = None

        self._positioner_error: Optional[str] = None
        self._controller_state: Optional[ControllerState] = None

    @property
    def positioner_error(self):
        return self._positioner_error

    @positioner_error.setter
    def positioner_error(self, new_value: str):
        self._positioner_error = new_value

    @property
    def controller_state(self):
        return self._controller_state

    @controller_state.setter
    def controller_state(self, new_value: ControllerState):
        self._controller_state = new_value

    def do_something(self, argument) -> None:
        """

        :param argument:
        :type argument:
        :return: None
        """
        raise NotImplementedError

    def write(self, command: str) -> None:
        """
        The SMC100 controller expects the termination character to be "\r\n".

        :param command:
        :type command: str
        :return: None
        """
        self.dev.write(f"{self.controller_address}{command}\r\n".encode())

    def read(self) -> str:
        """
        The SMC100 controller returns the termination character of "\r\n".

        :return:
        :rtype: str
        """
        response: str = self.dev.read_until(expected=b"\r\n").decode()
        return response

    def query(self, command: str) -> str:
        """

        :param command:
        :type command: str
        :return:
        :rtype: str
        """
        self.write(command=command)
        response: str = self.read()
        return response.strip("\r\n")

    @logging_decorator
    def connect(self, port: str) -> None:
        """
        Open the serial port and connect to the SMC100 device.

        :param port:
        :type port: str
        :return: None
        """
        self.dev: serial.Serial = (
            serial.Serial(
                port=port,
                baudrate=57_600,  # fixed according to Newport documentation
                timeout=1
            )
        )
        logging.info(f"self.dev.is_open : {self.dev.is_open}")

    @logging_decorator
    def tear(self) -> None:
        """

        :return: None
        """
        self.stop()  # stops the move for ALL controllers
        self.dev.flush()
        self.dev.close()
        logging.info(f"self.dev.is_open : {self.dev.is_open}")

    def set_acceleration(self, acceleration: float) -> None:
        """
        Sets the acceleration of the stage

        :param acceleration: acceleration
        :type acceleration: float
        :return: None
        """
        self.write(f"AC{acceleration}")

    def get_acceleration(self) -> float:
        """
        Gets the acceleration from the controller

        :return: the acceleration of the controller
        :rtype: float
        """
        command_name: str = "AC"
        response = self.query(f"{command_name}?")
        acceleration: float = (
            float(response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1])
        )
        return acceleration

    @logging_decorator
    def home(self, search_type: HomeSearchType.MZ_ONLY) -> None:
        """
        Sends stage to home position as defined by home switch

        :param search_type:
        :type search_type: HomeSearchType
        :return: None
        """
        self.write(command=f"HT{search_type.value}")  # set HOME search type
        self.execute_home_search()
        time.sleep(1.0)  # to mimic LabView example from Newport

    @logging_decorator
    def get_id(self) -> str:
        """
        Query the stage identifier string.

        :return: the stage identifier string
        :rtype: str
        """
        response: str = self.query(command="ID?")
        return response

    def execute_home_search(self) -> None:
        """
        Starts the execution of the HOME search as defined by the HT command.

        When in NOT REFERENCED state, for instance after system start, any positioner must
        first get homed with the OR command before further motion commands can get
        executed.

        The OR command gets accepted only in NOT REFERENCED state and only with no
        present hardware errors, except for end-of-run maybe. Refer to the TS command
        to get more information on the possible hardware errors.

        :return: None
        """
        self.write(command="OR")

    @logging_decorator
    def move_absolute(self, pos: float) -> None:
        """
        Executes a move in absolute coordinates

        :param pos: absolute position on stepper encoder
        :type pos: float
        :return: None
        """
        self.write(f"PA{pos}")

    # def move_absolute_backlash_compensated(self, pos: float) -> None:
    #     """
    #     Executes a move in absolute coordinates compensating for the motor backlash.
    #
    #     :param pos: absolute position on stepper encoder
    #     :type pos: float
    #     :return: None
    #     """
    #     if pos < self.get_position():
    #         default_velocity = self.get_velocity()
    #         self.set_velocity(velocity=0.4)
    #         self.move_absolute(pos=pos - 0.01)
    #         moving = self.is_moving()
    #         while moving is True:
    #             moving = self.is_moving()
    #         self.set_velocity(velocity=default_velocity)
    #         self.move_absolute(pos=pos)
    #     else:
    #         self.move_absolute(pos=pos)

    @logging_decorator
    def move_relative(self, pos: float) -> None:
        """
        Executes a move in relative coordinates

        :param pos: relative position on stepper encoder
        :type pos: float
        :return: None
        """
        self.write(f"PR{pos}")

    def get_motion_time_for_relative_move(self, relative_move: float) -> float:
        """
        The PT commands helps to evaluate move times for an efficient program flow.

        When receiving the PT command, the controller returns the time, in seconds, necessary
        to execute a relative move of the displacement (relative_move) with the current working
        parameters (velocity, acceleration, etc.). The controller does not execute any motion.

        :param relative_move: displacement with respect to the
            current position
        :type relative_move: float
        :return: the time in seconds necessary to execute the
            relative move
        :rtype: float
        """
        command_name: str = "PT"
        response = self.query(f"{command_name}{relative_move}")
        motion_time: float = (
            float(response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1])
        )
        return motion_time

    def set_configuration_state(self, state: int = 0) -> None:
        """
        0: Go from CONFIGURATION state to NOT REFERENCED state
        1: Go from NOT REFERENCED state to CONFIGURATION state

        :return: None
        """
        self.write(f"PW{state}")

    def get_configuration_state(self) -> str:
        """
        Query the controller to see if it is in the NOT REFERENCED
        state or the CONFIGURATION state

        :return:
        :rtype: str
        """
        configuration_state: str = self.query("PW?")
        return configuration_state

    @logging_decorator
    def reset(self) -> None:
        """
        Issues a hardware reset to the controller, equivalent to a power-up.

        To go from DISABLE or READY state to CONFIGURATION state, one must first
        reset the controller with the RS command. Then, change the state of the
        controller with the PW1 command from NOT REFERENCED to CONFIGURATION.

        :return: None
        """
        self.write("RS")

    def reset_address(self) -> None:
        """
        Resets the controller's address to 1.

        The address needs to be different for each CONEX-CC when connected
        on an RS-485 communication network.

        :return: None
        """
        self.write("RS##")

    def get_control_loop_state(self) -> ControlLoopState:
        """
        Query the controller to see if it is in Closed loop
        or Open loop control

        :return:
        :rtype: ControlLoopState
        """
        command_name: str = "SC"
        response = self.query(f"{command_name}?")
        state: int = (
            int(response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1])
        )
        state: ControlLoopState = ControlLoopState(state)
        return state

    def stop_motion(self) -> None:
        """
        Stops the move for the specified controller. The command stops a move in progress
        by decelerating the position immediately with the acceleration defined by the AC
        command until it stops.

        :return: None
        """
        self.write("ST")

    def stop(self) -> None:
        """
        Stops the move for ALL controllers.

        :return: None
        """
        self.dev.write("ST\r\n".encode())

    def get_encoder_increment_value(self) -> float:
        """
        Gets the value for one encoder count.

        :return value: The value for one encoder count. Refer to Page 43 in the User's Manual.
        :rtype value: str
        """
        command_name: str = "SU"
        response = self.query(f"{command_name}?")
        encoder_increment: float = (
            float(response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1])
        )
        return encoder_increment

    def get_setpoint_position(self) -> float:
        """
        Gets the set position

        :return: position setpoint
        :rtype: float
        """
        command_name: str = "TH"
        response = self.query(f"{command_name}?")
        setpoint_position: float = (
            float(response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1])
        )
        return setpoint_position

    def get_position(self) -> float:
        """
        Gets the current position

        :return: current position of the stepper
        :rtype: float
        """
        command_name: str = "TP"
        response = self.query(f"{command_name}?")
        position: float = (
            float(response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1])
        )
        return position

    @retry(retries=3, delay=1)
    def get_controller_status(self) -> str:
        """
        The TS command returns the positioner error and the active controller state.

        The TS command returns six characters (ex. 1TSabcdef) after the TS echo. The
        first four characters (abcd) represent the positioner error in hexadecimal. The
        last two characters (ef) represent the controller state. Refer to Page 49 in
        the Programmer's Manual.

        :return: the positioner error and controller state of the controller
        :rtype: str
        """
        command_name: str = "TS"
        response: str = self.query(command=f"{command_name}?")
        # logging.debug(f"response (str) : {response}")
        # logging.debug(f"response (bytes) : {response.encode()}")
        echo: str = f"{self.controller_address}{command_name}"
        status: str = response.split(sep=echo, maxsplit=1)[1]
        # logging.debug(f"status : {status}")

        self.positioner_error: str = status[:4]
        logging.debug(
            "Positioner Error Readable : "
            f"{self.get_positioner_errors_readable(error_code=int(self.positioner_error, 16))}"
        )
        self.controller_state: ControllerState = ControllerState(status[4:])
        logging.debug(f"controller_state : {self.controller_state}")

        return status

    def get_positioner_error(self) -> str:
        """
        Query the current controller state.

        :return positioner_error_hex: the current controller state
        :rtype positioner_error_hex: str
        """
        # Query controller status to define positioner_error
        # and controller_state instance attributes
        self.get_controller_status()
        return self.positioner_error

    def get_controller_state(self) -> ControllerState:
        """
        Query the current controller state.

        :return: the current controller state
        :rtype: ControllerState
        """
        # Query controller status to define positioner_error
        # and controller_state instance attributes
        self.get_controller_status()
        return self.controller_state

    def set_velocity(self, velocity: float) -> None:
        """
        Sets the velocity of the stage

        :param velocity: velocity
        :type velocity: float
        :return: None
        """
        self.write(f"VA{velocity}")

    def get_velocity(self) -> float:
        """
        Gets the velocity setpoint from the controller

        :return: velocity setpoint for the controller
        :rtype: float
        """
        command_name: str = "VA"
        response = self.query(f"{command_name}?")
        velocity: float = (
            float(response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1])
        )
        return velocity

    def get_revision_information(self) -> str:
        """
        Get the revision information of the controller.

        :return: the revision information of the controller
        :rtype: str
        """
        command_name: str = "VE"
        response = self.query(f"{command_name}")
        revision_information: str = (
            response.split(sep=f"{self.controller_address}{command_name}", maxsplit=1)[1]
        )
        return revision_information

    def get_all_configuration_parameters(self) -> list[str]:
        """
        Get the list of all current configuration parameters.

        :return: the list of all current configuration parameters
        :rtype: list[str]
        """
        command_name: str = "ZT"
        self.write(command_name)

        timeout: int = 1
        runtime: float = 0
        start_time: float = time.monotonic()
        parameters: list[str] = []
        while runtime <= timeout:
            parameters.append(self.read())
            runtime: float = time.monotonic() - start_time

        return parameters

    @logging_decorator
    def initialize(self, timeout: int = 30) -> ControllerState:
        """
        After connecting the SMC100 to power, the controller must be first
        initialized. If the initialization is successful, the controller gets to the
        NOT_REFERENCED_RESET state.

        To execute any move command, the controller must be in a READY state.
        To get from the NOT REFERENCED state to the READY state, the positioner
        must be homed first using the home() method.

        :param timeout: initialization timeout in units of seconds
        :type timeout: int
        :return: the controller state
        :rtype: ControllerState
        """
        self.reset()  # reset the controller, equivalent to a power-up
        runtime: float = 0.0
        start_time: float = time.monotonic()
        controller_state: ControllerState = self.get_controller_state()
        while controller_state != ControllerState.NOT_REFERENCED_RESET and runtime < timeout:
            runtime: float = time.monotonic() - start_time
            time.sleep(0.2)
            controller_state: ControllerState = self.get_controller_state()
        if runtime >= timeout:
            raise InstrumentException(
                f"[EXCEPTION] initialize() timed out after {timeout} seconds..."
            )
        return controller_state

    @logging_decorator
    def is_ready(self) -> bool:
        """
        Query the controller to see if the actuator is ready for motion control.

        :return:
        :rtype: bool
        """
        controller_state: ControllerState = self.get_controller_state()
        if controller_state in [
            ControllerState.READY_HOMING,
            ControllerState.READY_MOVING,
            ControllerState.READY_DISABLE,
            ControllerState.READY_JOGGING
        ]:
            ready: bool = True
        else:
            ready: bool = False
        return ready

    @logging_decorator
    def is_homing(self) -> bool:
        """
        Call after home_by_search_type() has been called to see if the stage is still homing.

        :raises InstrumentException: An InstrumentException if the controller state is not
            ControllerState.HOMING or ControllerState.READY_HOMING.
        :return: The controller state. Refer to Page 49 in the Programmer's Manual.
        :rtype: bool
        """
        controller_state: ControllerState = self.get_controller_state()
        if controller_state == ControllerState.HOMING:
            homing: bool = True
        elif controller_state == ControllerState.READY_HOMING:
            homing: bool = False
        else:
            raise InstrumentException(
                f"[EXCEPTION] unexpected controller state: {controller_state}..."
            )
        return homing

    @logging_decorator
    def wait_for_end_of_homing(self) -> None:
        """
        Wait for the homing operation to complete

        :return: None
        """
        # max_travel_distance: int = 12  # units of millimeters (mm)
        # velocity: float = self.get_velocity()  # units of millimeters per second (mm/s)
        # max_travel_time: float = max_travel_distance / velocity  # units of seconds (s)
        # timeout: float = 2 * max_travel_time  # make the timeout twice the max_travel_time

        timeout: float = 10

        runtime: float = 0
        start_time: float = time.monotonic()
        count: int = 0
        homing: bool = True
        while homing is True and runtime < timeout:
            if count % 10 == 0:
                logging.info(f"Is homing? : {homing}")
            runtime: float = time.monotonic() - start_time
            homing: bool = self.is_homing()
            time.sleep(0.2)
            count += 1
        logging.info(f"Is homing? : {homing}")
        if runtime >= timeout:
            raise InstrumentException("[EXCEPTION] wait_for_end_of_homing() timed out...")

    @logging_decorator
    def is_moving(self) -> bool:
        """
        Call after move_absolute() has been called to see if the stage is moving.
        It is critical to call this method only after move_absolute() has been called. The method
        can be updated to be more robust so that it can be called at any arbitrary point.

        :raises InstrumentException: Raises an InstrumentException if the controller state is not
            ControllerState.MOVING or ControllerState.READY_MOVING.
        :return: True if the stage is moving. False is the stage has finished moving.
        :rtype: bool
        """
        controller_state: ControllerState = self.get_controller_state()
        if controller_state == ControllerState.MOVING:
            moving: bool = True
        elif controller_state == ControllerState.READY_MOVING:
            moving: bool = False
        else:
            raise InstrumentException(f"[EXCEPTION] unexpected controller state: {controller_state}...")
        return moving

    @logging_decorator
    def wait_for_end_of_motion(self) -> None:
        """
        Wait for the stage motion to complete. To be used after the move_absolute() or
        move_relative() methods.

        :return: None
        """
        # max_travel_distance: int = 12  # units of millimeters (mm)
        # velocity: float = self.get_velocity()  # units of millimeters per second (mm/s)
        # max_travel_time: float = max_travel_distance / velocity  # units of seconds (s)
        # timeout: float = 2 * max_travel_time  # make the timeout twice the max_travel_time

        timeout: float = 10

        runtime: float = 0
        start_time: float = time.monotonic()
        count: int = 0
        moving: bool = True
        while moving is True and runtime < timeout:
            if count % 10 == 0:
                logging.info(f"Is moving? : {moving}")
            runtime: float = time.monotonic() - start_time
            moving: bool = self.is_moving()
            time.sleep(0.2)
            count += 1
        logging.info(f"Is moving? : {moving}")
        if runtime >= timeout:
            raise InstrumentException("[EXCEPTION] wait_for_end_of_motion() timed out...")

    @logging_decorator
    def get_positioner_errors_readable(self, error_code: int) -> list[str]:
        """
        Get the positioner errors is a human-readable format

        :param error_code:
        :type error_code: int
        :return: positioner errors is a human-readable format
        :rtype: list[str]
        """
        positioner_errors_readable: list[str] = []
        for bit_mask in PositionerErrorCode:
            if bit_mask.value & error_code != 0:
                positioner_errors_readable.append(bit_mask.name)
        return positioner_errors_readable


def main():
    """

    :return: None
    """
    smc100: Optional[SMC100] = None

    try:
        smc100: SMC100 = SMC100(controller_address=1)
        smc100.connect(port="COM4")
        logging.info(f"Identity : {smc100.get_id()}")

        smc100.get_controller_status()

        positioner_error_code: str = smc100.positioner_error
        logging.info(f"positioner_error_code (hexadecimal str) : {positioner_error_code}")
        logging.info(f"positioner_error_code (int) : {int(positioner_error_code, 16)}")
        logging.info(
            "Positioner Error Readable : "
            f"{smc100.get_positioner_errors_readable(error_code=int(positioner_error_code, 16))}"
        )

        controller_state: ControllerState = smc100.controller_state
        logging.info(f"controller_state : {controller_state}")

        # smc100.initialize()
        # smc100.home(search_type=HomeSearchType.MZ_ONLY)
        # smc100.wait_for_end_of_homing()

        # logging.info(f"Encoder Increment : {smc100.get_encoder_increment_value()}")

    except Exception as e:
        raise e

    finally:
        if smc100 is not None:
            smc100.tear()


if __name__ == "__main__":
    # Start logging
    logging.basicConfig(
        level=logging.DEBUG,
        # level=logging.INFO,
        format="%(asctime)s:%(module)s:%(levelname)s - %(message)s"
    )

    main()
