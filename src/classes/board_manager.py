import time

# Vendored DFRobot driver (libraries/) rather than a pip dependency - upstream
# does not publish it to PyPI.
from libraries.DFRobot_RaspberryPi_Expansion_Board import (
    DFRobot_Expansion_Board_IIC as Board,
    DFRobot_Expansion_Board_Servo as Servo,
)

I2C_BUS = 1
BOARD_ADDRESS = 0x10
BEGIN_TIMEOUT_S = 10.0


class BoardManager:
    """
    The DFRobot expansion board: the I2C hub every analog sensor and the
    steering-independent servo hang off.

    Owns nothing application-specific - it just brings the board up, keeps the
    ADC enabled and hands out raw channel reads. LightSensorManager and
    UltraServoManager sit on top of it and give those channels meaning.
    """

    def __init__(self, bus_id=I2C_BUS, address=BOARD_ADDRESS, debug=False):
        self.bus_id = bus_id
        self.address = address
        self.debug = debug
        self.board = None

    def start(self):
        """
        Connects and enables the ADC. Raises TimeoutError if the board never
        reports STA_OK - usually a power or I2C wiring problem, and there is no
        point letting a task run blind past it.
        """
        board = Board(self.bus_id, self.address)
        deadline = time.monotonic() + BEGIN_TIMEOUT_S
        while board.begin() != board.STA_OK:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Expansion board at {hex(self.address)} on i2c-{self.bus_id} never came up")
            time.sleep(0.5)

        board.set_adc_enable()
        self.board = board
        if self.debug:
            print(f"[board] up at {hex(self.address)} on i2c-{self.bus_id}, ADC enabled")

    def read_adc(self, channel):
        """Raw ADC reading in millivolts, or None if the read failed."""
        if self.board is None:
            return None
        try:
            return self.board.get_adc_value(channel)
        except Exception:
            return None

    def create_servo(self):
        """Returns a started servo driver bound to this board."""
        if self.board is None:
            raise RuntimeError("BoardManager.start() must be called first")
        servo = Servo(self.board)
        servo.begin()
        return servo

    def stop(self):
        if self.board is None:
            return
        try:
            self.board.set_adc_disable()
        except Exception:
            pass
        self.board = None
