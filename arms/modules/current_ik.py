#thetalist_dot = [0]*5

def jacobian(self, theta: list = None):
        """
        Compute the Jacobian matrix for the current robot configuration.

        Args:
            theta (list, optional): The joint angles for the robot. Defaults to self.theta.
        
        Returns:
            Jacobian matrix (3x5).
        """
        # Use default values if arguments are not provided
        if theta is None:
            theta = self.theta

        # Define DH parameters
        DH = np.zeros((5, 4))
        DH[0] = [theta[0], self.l1, 0, np.pi/2]
        DH[1] = [theta[1] + np.pi/2, 0, self.l2, np.pi]
        DH[2] = [theta[2], 0, self.l3, np.pi]
        DH[3] = [theta[3] - np.pi/2, 0, 0, -np.pi/2]
        DH[4] = [theta[4], self.l4 + self.l5, 0, 0]

        # Compute transformation matrices
        T = np.zeros((self.num_dof,4,4))
        for i in range(self.num_dof):
            T[i] = dh_to_matrix(DH[i])

        # Precompute transformation matrices for efficiency
        T_cumulative = [np.eye(4)]
        for i in range(self.num_dof):
            T_cumulative.append(T_cumulative[-1] @ T[i])

        # Define O0 for calculations
        O0 = np.array([0, 0, 0, 1])
        
        # Initialize the Jacobian matrix
        jacobian = np.zeros((3, self.num_dof))

        # Calculate the Jacobian columns
        for i in range(self.num_dof):
            T_curr = T_cumulative[i]
            T_final = T_cumulative[-1]
            
            # Calculate position vector r
            r = (T_final @ O0 - T_curr @ O0)[:3]

            # Compute the rotation axis z
            z = T_curr[:3, :3] @ np.array([0, 0, 1])

            # Compute linear velocity part of the Jacobian
            jacobian[:, i] = np.cross(z, r)

        # Replace near-zero values with zero, primarily for debugging purposes
        return near_zero(jacobian)
    
   
def damped_inverse_jacobian(self, q = None, damping_factor=0.025):
    if q is not None:
        J = self.jacobian(q)
    else:
        J = self.jacobian()

    JT = np.transpose(J)
    I = np.eye(3)
    return JT @ np.linalg.inv(J @ JT + (damping_factor**2)*I)

def dh_to_matrix(self, dh_params: list) -> np.ndarray:
        """Converts Denavit-Hartenberg parameters to a transformation matrix.

        Args:
            dh_params (list): Denavit-Hartenberg parameters [theta, d, a, alpha].

        Returns:
            np.ndarray: A 4x4 transformation matrix.
        """
        theta, d, a, alpha = dh_params
        return np.array([
            [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
            [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
            [0, sin(alpha), cos(alpha), d],
            [0, 0, 0, 1]
        ])

def solve_forward_kinematics(self, theta: list, radians=False):

    # Convert degrees to radians
    if not radians:
        for i in range(len(theta)):
            theta[i] = np.deg2rad(theta[i])

    # DH parameters = [theta, d, a, alpha]
    DH = np.zeros((5, 4))
    DH[0] = [theta[0],   self.l1,    0,       np.pi/2]
    DH[1] = [theta[1]+np.pi/2,   0,          self.l2, np.pi]
    DH[2] = [theta[2],   0,          self.l3, np.pi]
    DH[3] = [theta[3]-np.pi/2,   0,          0,       -np.pi/2]
    DH[4] = [theta[4],   self.l4+self.l5, 0, 0]

    T = np.zeros((self.num_dof,4,4))
    for i in range(self.num_dof):
        T[i] = dh_to_matrix(DH[i])

    return T[0] @ T[1] @ T[2] @ T[3] @ T[4] @ np.array([0, 0, 0, 1])


def solve_inverse_kinematics(self, EE: EndEffector, tol=1e-3, ilimit=500):

    Te_d = [EE.x, EE.y, EE.z]
        
    # Iteration count
    i = 0
    q = self.theta.copy()

    # move robot slightly out of zeros singularity
    q = [q[i] + np.random.rand()*0.05 for i in range(self.num_dof)]
        
    while i < ilimit:
        i += 1

        # compute current EE position based on q
        Te = self.solve_forward_kinematics(q, radians=True)

        # print(f"Current position: {Te}")
        # calculate the EE position error
        e = [0, 0, 0]
        e[0] = Te_d[0] - Te[0]
        e[1] = Te_d[1] - Te[1]
        e[2] = Te_d[2] - Te[2]

        # update q
        J = self.jacobian(q)
        # q += np.linalg.pinv(J) @ e
        q += self.damped_inverse_jacobian(q) @ e

        # check for joint limits
        for j, th in enumerate(q):
            q[j] = np.clip(th, self.theta_limits[j][0], self.theta_limits[j][1])

            # Check if we have arrived
            if abs(max(e, key=abs)) < tol:
                break 
        
        if abs(max(e, key=abs)) > tol:
            print("\n [ERROR] Numerical IK solution failed to converge... \n \
                  Possible causes: \n \
                  1. cartesian position is not reachable by the robot, given the joint limits \n \
                  2. desired joint configuration is very close to OR at a singularity \n \
                  3. iterative algorithm is stuck at a local minima \n \
                  4. solver is taking too long to converge  \n")
            print(f"Max position error: {max(e, key=abs)} | # iterations: {i}/{ilimit} ")
            # raise ValueError
            return False

        return q