import numpy as np
import frccontrol
import frccontrol as frccnt
import control
import control as cnt


def dt():
    Gr = 9.52
    Gl = Gr

    # Drivetrain mass in kg
    m = 16.0
    # Radius of wheels in meters
    r = 0.1524 / 2.0
    # Radius of robot in meters
    rb = 0.5588 / 2.0
    # Moment of inertia of the drivetrain in kg-m^2
    J = 6.0

    motor = frccontrol.models.gearbox(frccontrol.models.MOTOR_CIM, 2)

    C1 = -Gl ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C2 = Gl * motor.Kt / (motor.R * r)
    C3 = -Gr ** 2 * motor.Kt / (motor.Kv * motor.R * r ** 2)
    C4 = Gr * motor.Kt / (motor.R * r)

    A = np.array([[(1 / m + rb**2 / J) * C1, (1 / m - rb**2 / J) * C3],
                [(1 / m - rb**2 / J) * C1, (1 / m + rb**2 / J) * C3]
                ])

    B = np.array([[(1 / m + rb**2 / J) * C2, (1 / m - rb**2 / J) * C4],
                [(1 / m - rb**2 / J) * C2, (1 / m + rb**2 / J) * C4]
                ])

    C = np.array([[1, 0],
                [0, 1]
                ])

    D = np.array([[0, 0],
                [0, 0]
                ])

    return control.ss(A, B, C, D)

class Dt(frccontrol.System):
    def __init__(self, dt):
        """Drivetrain subsystem.
        Keyword arguments:
        dt -- time between model/controller updates
        """

        u_min = np.array([[-12.0], [-12.0]])
        u_max = np.array([[12.0], [12.0]])
        frccnt.System.__init__(self, np.zeros((2, 1)), u_min, u_max, dt)

    def create_model(self, states):
        return dt()

    def design_controller_observer(self):
        q_vel = 0.95

        q = [q_vel, q_vel]
        r = [12.0, 12.0]
        self.design_lqr(q, r)

        qff_vel = 1.0
        self.design_two_state_feedforward(
                [qff_vel, qff_vel], [12.0, 12.0]
        )
        q_vel = 1.0
        q_voltage = 10.0
        q_encoder_uncertainty = 2.0
        r_pos = 0.001
        r_gyro = 0.000001
        self.design_kalman_filter([q_vel, q_vel], [r_pos, r_pos])
