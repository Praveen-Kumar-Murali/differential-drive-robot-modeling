from src.simulation.simulated_robot import *
from src.utils.index_struct import *
from src.models.pose3d import *
import scipy
from roboticstoolbox.mobile.Animations import *
import numpy as np


class DifferentialDriveSimulatedRobot(SimulatedRobot):
    """
    This class implements a simulated differential drive robot. It inherits from the :class:`SimulatedRobot` class and
    overrides some of its methods to define the differential drive robot motion model.
    """
    def __init__(self, xs0, map=[],*args):
        """
        :param xs0: initial simulated robot state :math:`\\mathbf{x_{s_0}}=[^Nx{_{s_0}}~^Ny{_{s_0}}~^N\psi{_{s_0}}~]^T` used to initialize the  motion model
        :param map: feature map of the environment :math:`M=[^Nx_{F_1},...,^Nx_{F_{nf}}]`

        Initializes the simulated differential drive robot. Overrides some of the object attributes of the parent class :class:`SimulatedRobot` to define the differential drive robot motion model:

        * **Qsk** : Object attribute containing Covariance of the simulation motion model noise.

        .. math::
            Q_k=\\begin{bmatrix}\\sigma_{\\dot u}^2 & 0 & 0\\\\
            0 & \\sigma_{\\dot v}^2 & 0 \\\\
            0 & 0 & \\sigma_{\\dot r}^2 \\\\
            \\end{bmatrix}
            :label: eq:Qsk

        * **usk** : Object attribute containing the simulated input to the motion model containing the forward velocity :math:`u_k` and the angular velocity :math:`r_k`

        .. math::
            \\bf{u_k}=\\begin{bmatrix}u_k & r_k\\end{bmatrix}^T
            :label: eq:usk

        * **xsk** : Object attribute containing the current simulated robot state

        .. math::
            x_k=\\begin{bmatrix}{^N}x_k & {^N}y_k & {^N}\\theta_k & {^B}u_k & {^B}v_k & {^B}r_k\\end{bmatrix}^T
            :label: eq:xsk

        where :math:`{^N}x_k`, :math:`{^N}y_k` and :math:`{^N}\\theta_k` are the robot position and orientation in the world N-Frame, and :math:`{^B}u_k`, :math:`{^B}v_k` and :math:`{^B}r_k` are the robot linear and angular velocities in the robot B-Frame.

        * **zsk** : Object attribute containing :math:`z_{s_k}=[n_L~n_R]^T` observation vector containing number of pulses read from the left and right wheel encoders.
        * **Rsk** : Object attribute containing :math:`R_{s_k}=diag(\\sigma_L^2,\\sigma_R^2)` covariance matrix of the noise of the read pulses`.
        * **wheelBase** : Object attribute containing the distance between the wheels of the robot (:math:`w=0.5` m)
        * **wheelRadius** : Object attribute containing the radius of the wheels of the robot (:math:`R=0.1` m)
        * **pulses_x_wheelTurn** : Object attribute containing the number of pulses per wheel turn (:math:`pulseXwheelTurn=1024` pulses)
        * **Polar2D_max_range** : Object attribute containing the maximum Polar2D range (:math:`Polar2D_max_range=50` m) at which the robot can detect features.
        * **Polar2D\_feature\_reading\_frequency** : Object attribute containing the frequency of Polar2D feature readings (50 tics -sample times-)
        * **Rfp** : Object attribute containing the covariance of the simulated Polar2D feature noise (:math:`R_{fp}=diag(\\sigma_{\\rho}^2,\\sigma_{\\phi}^2)`)

        Check the parent class :class:`prpy.SimulatedRobot` to know the rest of the object attributes.
        """
        super().__init__(xs0, map,*args) # call the parent class constructor

        # Initialize the motion model noise
        self.Qsk = np.diag(np.array([0.1 ** 2, 0.01 ** 2, np.deg2rad(1) ** 2]))  # simulated acceleration noise
        self.usk = np.zeros((3, 1))  # simulated input to the motion model
        self.K = np.eye(3)    
        # Inititalize the robot parameters
        self.wheelBase = 0.5  # distance between the wheels
        self.wheelRadius = 0.1  # radius of the wheels
        self.pulse_x_wheelTurns = 1024  # number of pulses per wheel turn

        # Initialize the sensor simulation
        self.encoder_reading_frequency = 1  # frequency of encoder readings
        self.Re= np.diag(np.array([22 ** 2, 22 ** 2]))  # covariance of simulated wheel encoder noise

        self.Polar2D_feature_reading_frequency = 50  # frequency of Polar2D feature readings
        self.Polar2D_max_range = 50  # maximum Polar2D range, used to simulate the field of view
        self.Rfp = np.diag(np.array([1 ** 2, np.deg2rad(5) ** 2]))  # covariance of simulated Polar2D feature noise

        self.xy_feature_reading_frequency = 50  # frequency of XY feature readings
        self.xy_max_range = 50  # maximum XY range, used to simulate the field of view

        self.yaw_reading_frequency = 10  # frequency of Yasw readings
        self.v_yaw_std = np.deg2rad(5)  # std deviation of simulated heading noise

    def fs(self, xsk_1, usk):  # input velocity motion model with velocity noise
        """ Motion model used to simulate the robot motion. Computes the current robot state :math:`x_k` given the previous robot state :math:`x_{k-1}` and the input :math:`u_k`:

        .. math::
            \\eta_{s_{k-1}}&=\\begin{bmatrix}x_{s_{k-1}} & y_{s_{k-1}} & \\theta_{s_{k-1}}\\end{bmatrix}^T\\\\
            \\nu_{s_{k-1}}&=\\begin{bmatrix} u_{s_{k-1}} &  v_{s_{k-1}} & r_{s_{k-1}}\\end{bmatrix}^T\\\\
            x_{s_{k-1}}&=\\begin{bmatrix}\\eta_{s_{k-1}}^T & \\nu_{s_{k-1}}^T\\end{bmatrix}^T\\\\
            u_{s_k}&=\\nu_{d}=\\begin{bmatrix} u_d& r_d\\end{bmatrix}^T\\\\
            w_{s_k}&=\\dot \\nu_{s_k}\\\\
            x_{s_k}&=f_s(x_{s_{k-1}},u_{s_k},w_{s_k}) \\\\
            &=\\begin{bmatrix}
            \\eta_{s_{k-1}} \\oplus (\\nu_{s_{k-1}}\\Delta t + \\frac{1}{2} w_{s_k}) \\\\
            \\nu_{s_{k-1}}+K(\\nu_{d}-\\nu_{s_{k-1}}) + w_{s_k} \\Delta t
            \\end{bmatrix} \\quad;\\quad K=diag(k_1,k_2,k_3) \\quad k_i>0\\\\
            :label: eq:fs

        Where :math:`\\eta_{s_{k-1}}` is the previous 3 DOF robot pose (x,y,yaw) and :math:`\\nu_{s_{k-1}}` is the previous robot velocity (velocity in the direction of x and y B-Frame axis of the robot and the angular velocity).
        :math:`u_{s_k}` is the input to the motion model contaning the desired robot velocity in the x direction (:math:`u_d`) and the desired angular velocity around the z axis (:math:`r_d`).
        :math:`w_{s_k}` is the motion model noise representing an acceleration perturbation in the robot axis. The :math:`w_{s_k}` acceleration is the responsible for the slight velocity variation in the simulated robot motion.
        :math:`K` is a diagonal matrix containing the gains used to drive the simulated velocity towards the desired input velocity.

        Finally, the class updates the object attributes :math:`xsk`, :math:`xsk\_1` and  :math:`usk` to made them available for plotting purposes.

        **To be completed by the student**.

        :parameter xsk_1: previous robot state :math:`x_{s_{k-1}}=\\begin{bmatrix}\\eta_{s_{k-1}}^T & \\nu_{s_{k-1}}^T\\end{bmatrix}^T`
        :parameter usk: model input :math:`u_{s_k}=\\nu_{d}=\\begin{bmatrix} u_d& r_d\\end{bmatrix}^T`
        :return: current robot state :math:`x_{s_k}`
        """

        # TODO: to be completed by the student
        #
        x_prev = np.asarray(xsk_1, dtype=float).ravel()
        if x_prev.size != 6:
            raise ValueError("xsk_1 must have 6 elements: [x,y,theta,vx,vy,omega]")
        dt = getattr(self, 'dt', 0.1)
        # split pose and body velocities
        eta_prev = Pose3D(x_prev[0:3].reshape(3, 1))
        nu = x_prev[3:6].copy()   # [vx, vy, omega] in body frame
        # parse input desired velocities
        u_vec = np.asarray(usk, dtype=float).ravel()
        if u_vec.size == 2:
            nu_d = np.array([u_vec[0], 0.0, u_vec[1]])
        elif u_vec.size == 3:
            nu_d = u_vec.copy()
        else:
            raise ValueError("usk must be length 2 ([u_d, r_d]) or 3 ([u_d, v_d, r_d])")
        self.usk = nu_d.reshape(3, 1)
        w = np.random.multivariate_normal(np.zeros(3), self.Qsk)
        delta_body = nu * dt + 0.5 * w * (dt ** 2)
        delta_pose = Pose3D(delta_body.reshape(3, 1))
        eta_new = eta_prev.oplus(delta_pose)
        # velocity update: nu_new = nu + K*(nu_d - nu) + w*dt
        nu_new = nu + self.K.dot(nu_d - nu) + w * dt
        xsk = np.vstack((eta_new.reshape(3, 1), nu_new.reshape(3, 1)))
        self.xsk_1 = np.asarray(xsk_1, dtype=float).reshape(6, 1)
        self.xsk = xsk
        #

        if self.k % self.visualizationInterval == 0:
                self.PlotRobot()
                self.xTraj.append(self.xsk[0, 0])
                self.yTraj.append(self.xsk[1, 0])
                self.trajectory.pop(0).remove()
                self.trajectory = plt.plot(self.xTraj, self.yTraj, marker='.', color='orange', markersize=1)

        self.k += 1
        return self.xsk


    def ReadEncoders(self):
        """ Simulates the robot measurements of the left and right wheel encoders.

        **To be completed by the student**.

        :return zsk,Rsk: :math:`zk=[n_L~n_R]^T` observation vector containing number of pulses read from the left and right wheel encoders. :math:`R_{s_k}=diag(\\sigma_L^2,\\sigma_R^2)` covariance matrix of the read pulses.
        """

        # TODO: to be completed by the student

            # Ensure current state exists
        if not hasattr(self, 'xsk'):
            raise ValueError("Current robot state self.xsk not initialized")

        # Extract body velocities (vx, vy, omega)
        nu = self.xsk[3:6].ravel()
        vx, vy, omega = nu[0], nu[1], nu[2]

        # Differential drive forward kinematics
        # Compute left and right wheel linear velocities
        v_right = vx + (self.wheelBase / 2.0) * omega
        v_left  = vx - (self.wheelBase / 2.0) * omega

        # Convert linear velocities to wheel rotation per dt
        dt = getattr(self, 'dt', 0.1)
        theta_right = v_right * dt / self.wheelRadius  # radians
        theta_left  = v_left  * dt / self.wheelRadius  # radians

        # Convert wheel rotation to pulses
        pulses_per_rev = self.pulse_x_wheelTurns
        n_R = theta_right / (2 * np.pi) * pulses_per_rev
        n_L = theta_left  / (2 * np.pi) * pulses_per_rev

        # Add Gaussian encoder noise
        n_R_noisy =  n_R + np.random.randn() * np.sqrt(self.Re[1, 1])
        n_L_noisy = n_L + np.random.randn() * np.sqrt(self.Re[0, 0])

        # Build observation vector and return
        zsk = np.array([[n_L_noisy], [n_R_noisy]])
        Rsk = self.Re.copy()

        return zsk, Rsk

    def ReadCompass(self):
        """ Simulates the compass reading of the robot.

        :return: yaw and the covariance of its noise *R_yaw*
        """

        # TODO: to be completed by the student

            # Ensure current state exists
        if not hasattr(self, 'xsk'):
            raise ValueError("Current robot state self.xsk not initialized")

        # true yaw from current robot state
        theta = self.xsk[2, 0]

        # simulate noisy compass measurement
        yaw_measured = theta + np.random.randn() * self.v_yaw_std

        # wrap yaw to [-pi, pi]
        yaw_measured = (yaw_measured + np.pi) % (2 * np.pi) - np.pi

        # variance of compass noise
        R_yaw = self.v_yaw_std ** 2

        return yaw_measured, R_yaw

    def PlotRobot(self):
        """ Updates the plot of the robot at the current pose """

        self.vehicleIcon.update([self.xsk[0], self.xsk[1], self.xsk[2]])
        plt.pause(0.0000001)
        return

