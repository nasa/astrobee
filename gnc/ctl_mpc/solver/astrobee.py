import numpy as np
import casadi as ca
from util import *


class Astrobee():
    def __init__(self,
                 mass=9.6,
                 inertia=np.diag([0.1534, 0.1427, 0.1623]),
                 h=0.1):

        """
        Class for dynamics of the astrobee robot

        :param mass: mass of the Astrobee
        :type mass: float
        :param inertia: inertia tensor of the Astrobee
        :type inertia: np.diag
        :param h: sampling time of the discrete system, defaults to 0.01
        :type h: float, optional
        """

        # Model properties
        self.mass = ca.MX.sym('mass',1)
        self.inertia = ca.MX.sym('inertia',3)

        self.mass_numeric = mass
        self.inertia_numeric = inertia

        # Model - This model uses Euler angles, while NASA packages use quaternions!
        # State vector: [pos, vel, theta, omega], all in x,y,z so 12 states
        self.n = 12
        # Input vector: [force, torque], all in x,y,z so 6 inputs
        self.m = 6

        self.dt = h

        # Linearized model for continuous and discrete time
        self.Ac, self.Bc = self.CreateLinearizedDynamics()
        self.Ad, self.Bd = self.CreateDiscreteDynamics()

    def NonlinearDynamics(self, x, u, mass, inertia):
        """
        Pendulum nonlinear dynamics.

        :param x: state
        :type x: ca.MX
        :param u: control input
        :type u: ca.MX
        :return: state time derivative
        :rtype: ca.MX
        """

        # State extraction
        p = x[0:3]
        v = x[3:6]
        e = x[6:9]
        w = x[9:]

        # 3D Force
        f = u[0:3]

        # 3D Torque
        tau = u[3:]

        # Model
        pdot = v
        vdot = ca.mtimes(r_mat(e), f) / mass
        edot = ca.mtimes(rot_jac_mat(e), w)
        wdot = ca.mtimes(ca.inv(inertia), tau + ca.mtimes(skew(w),
                         ca.mtimes(inertia, w)))

        dxdt = [pdot, vdot, edot, wdot]

        return ca.vertcat(*dxdt)

    def LinearizedDynamics(self, x, u):
        """
        Linear dynamics for the Astrobee, continuous time.

        :param x: state
        :type x: np.ndarray, ca.DM, ca.MX
        :param u: control input
        :type u: np.ndarray, ca.DM, ca.MX
        :return: state derivative
        :rtype: np.ndarray, ca.DM, ca.MX
        """

        xdot = self.Ac @ x + self.Bc @ u

        return xdot

    def LinearizedDiscreteDynamics(self, x, u):
        """
        Method to propagate discrete-time dynamics for Astrobee

        :param x: state
        :type x: np.ndarray, ca.DM
        :param u: control input
        :type u: np.ndarray, ca.DM
        :return: state after dt seconds
        :rtype: np.ndarray, ca.DM
        """

        #x_next = np.matmul(self.Ad, x.squeeze()) + np.matmul(self.Bd, u.squeeze())
        x_next = ca.mtimes(self.Ad, x) + ca.mtimes(self.Bd, u)
        return x_next

    def CreateLinearizedDynamics(self):
        """
        Helper function to populate Ac and Bc with continuous-time
        dynamics of the system.
        """

        # Set CasADi variables
        x = ca.MX.sym('x', self.n)
        u = ca.MX.sym('u', self.m)
        # Jacobian of exact discretization
        Ac = ca.Function('Ac', [x, u], [ca.jacobian(
                         self.NonlinearDynamics(x, u, self.mass_numeric, self.inertia_numeric), x)])
        Bc = ca.Function('Bc', [x, u], [ca.jacobian(
                         self.NonlinearDynamics(x, u, self.mass_numeric, self.inertia_numeric), u)])

        # Linearization points
        x_bar = np.zeros((self.n, 1))
        u_bar = np.zeros((self.m, 1))

        self.Ac = np.asarray(Ac(x_bar, u_bar))
        self.Bc = np.asarray(Bc(x_bar, u_bar))

        return self.Ac, self.Bc

    def CasadiC2D(self, A, B, C, D):
        """
        Continuous to Discrete-time dynamics
        """
        # Set CasADi variables
        x = ca.MX.sym('x', A.shape[1])
        u = ca.MX.sym('u', B.shape[1])

        # Create an ordinary differential equation dictionary. Notice that:
        # - the 'x' argument is the state
        # - the 'ode' contains the equation/function we wish to discretize
        # - the 'p' argument contains the parameters that our function/equation
        #   receives. For now, we will only need the control input u
        ode = {'x': x, 'ode': ca.DM(A) @ x + ca.DM(B) @ u, 'p': ca.vertcat(u)}

        # Here we define the options for our CasADi integrator - it will take care of the
        # numerical integration for us: fear integrals no more!
        options = {"abstol": 1e-5, "reltol": 1e-9, "max_num_steps": 100, "tf": self.dt}

        # Create the integrator
        self.Integrator = ca.integrator('integrator', 'cvodes', ode, options)

        # Now we have an integrator CasADi function. We wish now to take the partial
        # derivaties w.r.t. 'x', and 'u', to obtain Ad and Bd, respectively. That's wher
        # we use ca.jacobian passing the integrator we created before - and extracting its
        # value after the integration interval 'xf' (our dt) - and our variable of interest
        Ad = ca.Function('jac_x_Ad', [x, u], [ca.jacobian(
                         self.Integrator(x0=x, p=u)['xf'], x)])
        Bd = ca.Function('jac_u_Bd', [x, u], [ca.jacobian(
                         self.Integrator(x0=x, p=u)['xf'], u)])

        # If you print Ad and Bd, they will be functions that can be evaluated at any point.
        # Now we must extract their value at the linearization point of our chosing!
        x_bar = np.zeros((12, 1))
        u_bar = np.zeros((6, 1))

        return np.asarray(Ad(x_bar, u_bar)), np.asarray(Bd(x_bar, u_bar)), C, D

    def CreateDiscreteDynamics(self):
        A, B = self.CreateLinearizedDynamics()
        C = np.diag(np.ones(12))
        D = np.zeros((12, 6))
        Ad, Bd, _, _ = self.CasadiC2D(A, B, C, D)

        return Ad, Bd