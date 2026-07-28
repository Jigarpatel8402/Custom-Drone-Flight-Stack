class Vehicle:
    NAME = "iris"
    FRAME = "QUAD_X"
    MOTOR_COUNT = 4
    MASS = 1.50
    
    MAX_THRUST_PER_MOTOR = 6.9243498
    MAX_COLLECTIVE_THRUST = 4 * MAX_THRUST_PER_MOTOR
    HOVER_THRUST = 17.1548      # MASS * Physics.GRAVITY
    ARM_PWM = 1050.0


class MotorGeometry:
    def __init__(self, id, x, y, direction):
        self.id = id
        self.x = x
        self.y = y
        self.direction = direction


class Frame():
    ARM_LENGTH = 0.25554
    CW = +1
    CCW = -1
    MOTORS_POSE = [
        (+1, +1),
        (+1, -1),
        (-1, -1),
        (-1, +1)
    ]
    MOTOR_POSITIONS = [
        (+0.13, +0.22),
        (+0.13, -0.22),
        (-0.13, -0.22),
        (-0.13, +0.22)
    ]
    ROTATION_DIRECTIONS = [
        +1,
        -1,
        +1,
        -1
    ]
    MOTORS = [
        MotorGeometry(0, +0.13, +0.22, CW),
        MotorGeometry(1, +0.13, -0.22, CCW),
        MotorGeometry(2, -0.13, -0.22, CW),
        MotorGeometry(3, -0.13, +0.22, CCW)
    ]


class Inertia:
    IXX = 0.008
    IYY = 0.015
    IZZ = 0.017

    IXY = 0.0
    IXZ = 0.0
    IYZ = 0.0

    # MAX_ROLL_TORQUE = 2.5
    MAX_PITCH_TORQUE = 2.5
    MAX_YAW_TORQUE = 0.8

    MAX_ROLL_TORQUE = (
        2 * 
        Vehicle.MAX_THRUST_PER_MOTOR * 
        0.22
    )


class Motors:
    MAX_RPM = 8600
    MIN_RPM = 0

    MAX_OMEGA = 900.0

    TAU_UP = 0.012
    TAU_DOWN = 0.025


class Propellers:
    KF = 8.54858e-6
    KM = 1.6e-7

    DIAMETER = 0.20
    PITCH = None


class ESC:
    MIN_PWM = 1000.0
    MAX_PWM = 2000.0
    IDLE_PWM = 1050

    THROTTLE_MIN = 0.0
    THROTTLE_MAX = 1.0


class Battery:
    CELLS = None
    NOMINAL_VOLTAGE = None
    FULL_VOLTAGE = None
    CAPACITY = None


class IMU:
    UPDATE_RATE = 1000.0

    GYRO_NOISE = None
    ACCEL_NOISE = None


class Simulation:
    CONTROL_DT = 0.004
    CONTROL_RATE = 250
