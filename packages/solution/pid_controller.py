#!/usr/bin/env python3

from typing import Tuple
import numpy as np

class PIDController():
    def __init__(self):

        # We will initialize some variables that might be useful
        self.prev_e_heading = 0.0
        self.prev_e_offset = 0.0
        self.prev_int_heading = []
        self.prev_int_offset = 0.0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0


    def HeadingControl(self,
                       v_ref: float,
                       theta_ref: float,
                       theta_curr: float,
                       delta_t: float
    ) -> Tuple[float, float]:
        """
        PID performing heading control.
        Args:
            v_ref:      reference velocity.
            theta_ref:  reference heading pose.
            theta_curr: the current estimated heading.
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference heading
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_heading to track the integral term
        # self.prev_e_heading the previous error. But note that you
        # should be the one to update them also.

        integral_sum_length = 5

        theta_ref = (theta_ref + np.pi) % (2 * np.pi) - np.pi    # modulo such that heading is -pi to +pi
        theta_curr = (theta_curr + np.pi) % (2 * np.pi) - np.pi
        omega_error = (theta_ref - theta_curr + np.pi) % (2 * np.pi) - np.pi      # wrap the error such that it knows the shortest path to the goal angle  
        omega_integral_current =  delta_t*omega_error
        self.prev_int_heading.append(  omega_integral_current  ) 
        delta_e = (omega_error - self.prev_e_heading)/delta_t

        int_sum = np.sum(np.array(self.prev_int_heading)[-integral_sum_length:] )
        omega = self.kp*omega_error +self.ki* ( int_sum )  + self.kd*delta_e
        self.prev_e_heading = omega_error
        v = v_ref
        print(
            f"{theta_ref=:.2f}  "
            f"{omega_error=:.2f}  "
            f"{int_sum=:.2f}  "
            f"{omega_integral_current=:.2f}  "
            f"{delta_e=:.2f}  "
            f"{omega=:.2f}"
        )
        return v, omega

    def OffsetControl(self,
                      v_ref: float,
                      y_ref: float,
                      y_curr: float,
                      delta_t: float
                      ) -> Tuple[float, float]:
        """
        PID performing lateral offset control.
        Args:
            v_ref:      linear Duckiebot speed.
            y_ref:      reference heading pose.
            y_curr:     the current estimated "y" coordinate (offset)
            delta_t:    time interval since last call.
        Returns:
            v:          linear velocity of the Duckiebot
            omega:      angular velocity of the Duckiebot
        """

        # TODO: implement a PID controller to track the reference lateral offset
        # feel free to make use of the global variables:
        # self.kp, self.ki, and self. kd, which are
        # set either by the notebook or from noVNC
        # as well as self_prev_int_offset to track the integral term
        # self.prev_e_offset the previous error. But note that you
        # should be the one to update them also.

        y_error = y_ref - y_curr
        y_integral = self.prev_int_offset + delta_t*y_error
        delta_e = (y_error - self.prev_e_offset)/delta_t

        omega = self.kp*y_error +self.ki*y_integral + self.kd*delta_e
        self.prev_int_offset = y_integral
        self.prev_e_offset = y_error
        v = v_ref
        #omega = np.random.uniform(-8.0, 8.0)
        return v, omega



    def SetGains(self, kp: float, ki: float, kd: float) -> None:
        # Set the PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd