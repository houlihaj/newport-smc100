import logging
from enum import Enum
from typing import Optional
import serial


class HomeSearchType(Enum):
    MZ_ENCODER = 0
    HOME = 1
    MZ_ONLY = 2
    EOR_ENCODER = 3
    EOR_ONLY = 4


class ControllerState(Enum):
    INVALID = ""
    NOT_REFERENCED_RESET = "0A"
    NOT_REFERENCED_HOMING = "0B"
    NOT_REFERENCED_CONFIGURATION = "0C"
    NOT_REFERENCED_DISABLE = "0D"
    NOT_REFERENCED_READY = "0E"
    NOT_REFERENCED_MOVING = "0F"
    NOT_REFERENCED_ESP = "10"
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


class SMC100:
    def __init__(self, controller_address: int):
        """

        :param controller_address: the controller address on the RS-485
            communication network
        :type controller_address: int
        """
        self.controller_address: int = controller_address

        self.dev: Optional[serial.Serial] = None

    def do_something(self, argument) -> None:
        """

        :param argument:
        :type argument:
        :return: None
        """
        raise NotImplementedError

    def write(self, command: str) -> None:
        """
        The SMC100 controller expects the termination character to be '\r\n'.

        :param command:
        :type command: str
        :return: None
        """
        self.dev.write(f"{self.controller_address}{command}\r\n".encode())

    def read(self) -> str:
        """
        The SMC100 controller returns the termination character of '\r\n'.

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
        return response

    def connect(self, port: str) -> None:
        """
        Connect to the SMC100 device

        :param port:
        :type port: str
        :return: None
        """
        self.dev: serial.Serial = serial.Serial()
        self.dev.port = port
        self.dev.baudrate = 57_600  # fixed according to Newport documentation

        self.dev.write_timeout = 1

        self.dev.open()

    def tear(self) -> None:
        """

        :return: None
        """
        self.dev.flush()
        self.dev.close()
        print(f"self.dev.is_open : {self.dev.is_open}")

    def get_id(self) -> str:
        """
        Query the stage identifier string.

        :return: the stage identifier string
        :rtype: str
        """
        response: str = self.query(command="ID")
        return response


def main():
    """

    :return: None
    """
    smc100: SMC100 = SMC100(controller_address=1)
    smc100.connect(port="COM4")
    logging.info(f"Identity : {smc100.get_id()}")
    smc100.tear()


if __name__ == "__main__":
    # Start logging
    logging.basicConfig(
        level=logging.DEBUG,
        # level=logging.INFO,
        format="%(asctime)s:%(module)s:%(levelname)s:%(message)s"
    )

    main()
