
```python
def log_so3(R):
    cos_theta = (np.trace(R) - 1) / 2
    cos_theta = np.clip(cos_theta, -1, 1)  # numerical safety
    theta = np.arccos(cos_theta)
    if theta < 1e-10:
        return np.zeros(3)
    if abs(theta - np.pi) < 1e-10:
        # Special case: 180-degree rotation, axis from largest diagonal
        # ... (omitted for brevity)
        pass
    K = (R - R.T) / (2 * np.sin(theta))
    return theta * np.array([K[2, 1], K[0, 2], K[1, 0]])
```

With `exp` and `log` you can do calculus on rotations. Need to take a tiny step from rotation `R` in direction `δω` (a small vector)? Compute `R_new = exp(δω_hat) * R`. Want to integrate angular velocity `ω(t)` from `t=0` to `t=T`? In the simplest case (constant `ω`), `R(T) = exp(T·ω_hat) * R(0)`. For varying `ω`, you discretize and step.

This is what underwrites every modern numerical integrator for rigid body simulation, every IMU-driven orientation estimator, every visual-inertial odometry system. The Lie group structure is not academic — it is the right tool for the job.

## 3.4 SE(3): rigid motions in three dimensions

The same story extends to rigid motions (rotation plus translation). The set of all rigid motions in 3D is the Special Euclidean group *SE(3)*. It is a Lie group too. Its Lie algebra *se(3)* consists of *twists* — six-dimensional vectors `ξ = (v, ω)` combining a linear velocity `v` and an angular velocity `ω`.

A twist `ξ` corresponds to a 4-by-4 matrix:

$$\hat\xi = \begin{bmatrix} \hat\omega & v \\ 0 & 0 \end{bmatrix}$$

The exponential map sends a twist to a homogeneous transformation. The closed-form expression is somewhat involved (it comes from integrating constant-twist motion over unit time), but `expm` (the matrix exponential, a function in both MATLAB and SciPy) computes it numerically without trouble:

```python
from scipy.linalg import expm

def hat_se3(twist):
    """Convert a 6-vector twist to a 4x4 matrix."""
    v = twist[:3]
    omega = twist[3:]
    M = np.zeros((4, 4))
    M[:3, :3] = skew(omega)
    M[:3, 3] = v
    return M

# A twist combining 90-degree rotation around z and 1m forward translation
twist = np.array([1.0, 0, 0, 0, 0, np.pi/2])
T = expm(hat_se3(twist))
print(T)
```

The composability of rigid motions is matrix multiplication, just as in chapter 9 of the first book. The new piece is that we can now *take derivatives* and *integrate* in this space, using the Lie algebra structure.

This is the mathematical infrastructure behind:

- Odometry estimation, where a robot integrates noisy measurements of its motion to track its pose.
- Visual SLAM, where camera poses are estimated and refined as Lie group elements.
- Trajectory optimization, where smoothness of a trajectory in pose space is measured using Lie group calculus.
- Modern manipulation planners, which represent gripper poses as SE(3) elements and optimize over them.

You will see the notation `∈ SE(3)`, `∈ SO(3)`, `exp_SE(3)`, `log_SE(3)` in many papers. Now you know what they mean. The next time you see a paper that says "we optimize over a sequence of SE(3) poses using Levenberg-Marquardt with Lie algebra parameterization," you will recognize it as standard nonlinear optimization on the manifold of rigid motions, which is just calculus done correctly on this curved space.

## 3.5 Twists, screws, and the geometric flavor

Beyond the algebraic machinery, twists have a beautiful geometric interpretation. Every twist corresponds to a *screw motion*: a simultaneous rotation around an axis and translation along the same axis. Pure rotation is a degenerate screw with zero translation; pure translation is a degenerate screw with infinite axis distance ("axis at infinity"). All other rigid motions are genuine screws.

This is *Chasles' theorem*: any rigid body displacement can be expressed as a screw motion around some axis with some pitch (translation per radian). When you watch a door close, that is a screw motion with very small pitch. When you twist a screw into wood, that is a screw motion with deliberate pitch. The mathematics is the same.

Why does this matter to you? Because joint motions in robotics are screws. A revolute joint is a pure rotation — a screw with zero pitch. A prismatic joint is a pure translation — a screw at infinity. A helical (screw) joint is a genuine screw. The forward kinematics formulation called the *Product of Exponentials* (PoE), used in the *Modern Robotics* textbook by Lynch and Park, builds forward kinematics by composing screw motions, one per joint. It is more geometrically natural than the older Denavit-Hartenberg formulation, and increasingly standard.

I will use the PoE formulation in the next chapter when we revisit forward kinematics in depth.

\newpage

# Chapter 4 — Forward Kinematics, Properly

The first volume gave you forward kinematics through the Denavit-Hartenberg convention and a worked example for a 2-link planar arm. This chapter goes much deeper: the Product of Exponentials formulation, the body Jacobian and spatial Jacobian, manipulability, kinematic singularities, redundancy resolution, and a complete worked example for a six-degree-of-freedom arm.

## 4.1 The Product of Exponentials formulation

Recall that a revolute joint produces a rotation around an axis. As the joint angle changes, the body downstream of the joint moves through a screw motion (a rotation in this case, a pure rotation since revolute joints have no pitch).

The PoE formulation captures this as follows. For each joint `i`, define a *screw axis* `S_i`, a six-vector encoding the joint's axis of motion. For a revolute joint, `S_i = (ω_i, -ω_i × q_i)` where `ω_i` is the unit vector along the joint axis and `q_i` is any point on the joint axis. (This formula, while initially mysterious, comes naturally out of the screw-motion interpretation: the angular velocity is `ω`, and the linear velocity at the origin due to the rotation is `-ω × q`.) For a prismatic joint, `S_i = (0, ω_i)` where `ω_i` is the unit vector along the joint's translation direction.

Given the screw axes and the home configuration `M` (the end-effector pose when all joint angles are zero), the forward kinematics is:

$$T(\theta) = \exp(\hat S_1 \theta_1) \cdot \exp(\hat S_2 \theta_2) \cdots \exp(\hat S_n \theta_n) \cdot M$$

This is conceptually clean. Each joint contributes one factor. Each factor is the screw motion produced by that joint at its current angle, expressed in the *space frame* (a fixed reference frame, often at the robot's base). The order matters — joint 1 is the first factor, joint `n` is the last — because the joints are applied in order from base to tip.

There is also a body-frame variant where each `S_i` is expressed in the end-effector's frame; the formula then becomes `T(θ) = M · exp(B_1 θ_1) · ... · exp(B_n θ_n)`. The two forms give the same answer for the same robot; choose whichever is more convenient.

## 4.2 Implementing PoE

Let me build PoE in code for a simple six-degree-of-freedom arm. I will use the well-known UR5 specifications as the example, since the UR5 is widely simulated and has public parameters.

In Python:

```python
import numpy as np
from scipy.linalg import expm

def skew(w):
    return np.array([
        [0,    -w[2],  w[1]],
        [w[2],  0,    -w[0]],
        [-w[1], w[0],  0   ]
    ])

def screw_to_se3_matrix(S):
    """Convert a 6-vector screw [omega; v] to a 4x4 se(3) matrix."""
    omega = S[:3]
    v = S[3:]
    M = np.zeros((4, 4))
    M[:3, :3] = skew(omega)
    M[:3, 3] = v
    return M

def fk_poe(M, screws_space, thetas):
    """
    Forward kinematics using PoE in the space frame.
    M: 4x4 home configuration (end-effector pose when theta = 0)
    screws_space: list of 6-vector screw axes in space frame
    thetas: list of joint angles
    """
    T = np.eye(4)
    for S, theta in zip(screws_space, thetas):
        T = T @ expm(screw_to_se3_matrix(S) * theta)
    T = T @ M
    return T

# UR5 parameters
W1, W2 = 0.109, 0.082    # wrist offsets
L1, L2 = 0.425, 0.392    # link lengths
H1, H2 = 0.089, 0.095    # base height, end height

# Home configuration of the end-effector
M = np.array([
    [-1, 0,  0, L1 + L2],
    [ 0, 0,  1, W1 + W2],
    [ 0, 1,  0, H1 - H2],
    [ 0, 0,  0, 1      ]
])

# Screw axes for each joint, in space frame
screws_space = [
    np.array([0, 0, 1, 0, 0, 0]),
    np.array([0, 1, 0, -H1, 0, 0]),
    np.array([0, 1, 0, -H1, 0, L1]),
    np.array([0, 1, 0, -H1, 0, L1 + L2]),
    np.array([0, 0, -1, -W1, L1 + L2, 0]),
    np.array([0, 1, 0, H2 - H1, 0, L1 + L2])
]

# Compute FK for a particular configuration
thetas = [np.pi/2, -np.pi/3, np.pi/4, np.pi/6, -np.pi/4, np.pi/3]
T = fk_poe(M, screws_space, thetas)
print("End-effector pose:")
print(T)
```

In MATLAB, the same algorithm:

```matlab
function S = skew(w)
    S = [   0   -w(3)  w(2);
          w(3)   0   -w(1);
         -w(2)  w(1)   0  ];
end

function M = screw_to_se3(S)
    omega = S(1:3);
    v = S(4:6);
    M = zeros(4);
    M(1:3, 1:3) = skew(omega);
    M(1:3, 4) = v;
end

function T = fk_poe(M, screws_space, thetas)
    T = eye(4);
    for i = 1:length(thetas)
        T = T * expm(screw_to_se3(screws_space(:, i)) * thetas(i));
    end
    T = T * M;
end
```

The MATLAB code is parallel to the Python code, with slight syntactic differences. Both use the matrix exponential — `expm` in both languages.

## 4.3 The space Jacobian and the body Jacobian

We saw the Jacobian briefly in volume one. Let me go deeper.

There are two kinds of Jacobians for a serial-chain robot, called the *space Jacobian* `J_s` and the *body Jacobian* `J_b`.

The space Jacobian relates joint velocities to a *spatial twist* — a six-vector `(v_s, ω_s)` representing the end-effector's velocity expressed in the space (base) frame. Specifically:

$$V_s = J_s(\theta) \cdot \dot\theta$$

The body Jacobian relates joint velocities to a *body twist* — a six-vector `(v_b, ω_b)` representing the end-effector's velocity expressed in its own frame:

$$V_b = J_b(\theta) \cdot \dot\theta$$

The two Jacobians are related by the *adjoint map*: `J_s = Ad_T · J_b` where `T` is the current end-effector pose and `Ad_T` is a 6-by-6 matrix derived from `T`. They contain the same information, just in different frames.

For practical purposes, the space Jacobian is convenient when you want to specify a desired end-effector velocity in the world frame (move the gripper one centimeter to the east). The body Jacobian is convenient when you want to specify motion relative to the gripper itself (move forward along the gripper's approach direction).

The space Jacobian for a PoE formulation has a beautiful structure. Each column is a screw axis, propagated through the previous joints:

$$J_s(\theta) = [S_1, \text{Ad}_{T_1}(S_2), \text{Ad}_{T_1 T_2}(S_3), \ldots]$$

where `T_i = exp(S_1 θ_1) ... exp(S_i θ_i)` is the partial product up to joint `i`. The first column is just `S_1` (joint 1's screw axis, since no prior joints have moved). The second column is joint 2's screw axis transformed by joint 1's current motion. And so on.

```python
def adjoint(T):
    """Compute the 6x6 adjoint of a 4x4 transformation."""
    R = T[:3, :3]
    p = T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, 3:] = R
    Ad[3:, :3] = skew(p) @ R
    return Ad

def jacobian_space(screws_space, thetas):
    """Compute the space Jacobian for PoE."""
    n = len(thetas)
    J = np.zeros((6, n))
    T_partial = np.eye(4)
    for i in range(n):
        if i == 0:
            J[:, 0] = screws_space[0]
        else:
            J[:, i] = adjoint(T_partial) @ screws_space[i]
        T_partial = T_partial @ expm(screw_to_se3_matrix(screws_space[i]) * thetas[i])
    return J
```

I am switching the convention slightly compared to some textbooks; check Lynch and Park's *Modern Robotics* chapter 5 for the canonical form.

## 4.4 Manipulability and singularities

Once you have the Jacobian, you can ask: in what directions can the robot easily move at the current configuration? In what directions is it stuck?

The **manipulability ellipsoid** answers this. It is defined by the equation `V^T (J J^T)^(-1) V ≤ 1`, where `V` is a unit-magnitude end-effector twist. The set of `V` satisfying this inequality is an ellipsoid in the six-dimensional twist space. The principal axes of the ellipsoid are the eigenvectors of `J J^T`, and their lengths are the square roots of the eigenvalues.

A long axis means the robot can move easily in that direction. A short axis means it has little leverage. When an axis shrinks to zero (an eigenvalue of `J J^T` is zero), the robot is at a *singularity* — it has temporarily lost a degree of freedom.

The **Yoshikawa manipulability measure** condenses the ellipsoid into a single number: `μ = sqrt(det(J J^T))`. It is the volume (up to a constant) of the manipulability ellipsoid. As the robot approaches a singularity, `μ → 0`. Modern controllers monitor `μ` and either avoid low-manipulability configurations or switch to special control modes near them.

```python
def manipulability(J):
    """Compute Yoshikawa manipulability measure."""
    return np.sqrt(np.linalg.det(J @ J.T))
```

## 4.5 Redundancy and the null-space projection

A robot with more than six degrees of freedom is *redundant* for typical tasks (which require six degrees in three-dimensional space). A seven-degree-of-freedom arm can reach a given pose with infinitely many configurations, differing in the position of the elbow and other internal joints.

This redundancy is useful. You can reach a target *and* simultaneously avoid obstacles, optimize joint comfort, stay away from singularities, or keep the arm in a nice configuration for the next motion.

The trick is *null-space projection*. The Jacobian's null space is the set of joint motions that produce zero end-effector motion — internal motions that reconfigure the arm without moving the gripper. You can use these motions to satisfy secondary objectives.

The standard formulation: pick a desired end-effector velocity `V` and a secondary objective gradient `g` (a vector in joint space pointing toward your secondary goal). Then:

$$\dot\theta = J^+ V + (I - J^+ J) g$$

The first term `J^+ V` (using the pseudoinverse) gives the joint motion needed to produce `V`. The second term is the projection of `g` onto the null space of `J`, contributing nothing to `V` but pursuing the secondary goal.

This is one of the most useful techniques in modern manipulator control. Agricultural robotics with redundant arms (the Franka Panda has seven joints, for instance) takes advantage of redundancy to avoid branches while still reaching fruits.

\newpage

# Chapter 5 — Inverse Kinematics, Methods and Their Trade-offs

The inverse kinematics problem — given a desired end-effector pose, find joint angles that achieve it — has multiple solution methods, each with different strengths. This chapter surveys them, explains when to use which, and provides working code in both languages.

## 5.1 Analytical inverse kinematics

For specific arm geometries with kinematic decouplings, IK admits closed-form solutions. The most famous case is a six-DOF arm with a *spherical wrist* — the last three joints all intersect at a point. Such arms can be solved by separating position (handled by the first three joints) from orientation (handled by the wrist).

The PUMA 560, the Stäubli arms, and several Universal Robots variants have spherical wrists. The closed-form solutions for these are derived in classical textbooks (Craig's *Introduction to Robotics* covers PUMA in detail; Lynch and Park cover several modern arms). The solutions involve careful trigonometric manipulation and yield up to eight discrete solutions for a given target pose, corresponding to "elbow up vs. down," "wrist flip," and other branches.

Analytical IK is fast (milliseconds) and exact. Its limitations: it is fragile (changes to the arm geometry require redoing the math), specific (each arm geometry has its own solution), and does not generalize to arbitrary kinematic chains.

## 5.2 Numerical inverse kinematics: the Jacobian-based methods

For general arms, numerical IK is the standard. Start with an initial guess `θ_0`. Compute the current end-effector pose `T(θ_0)`. The error compared to the target is some twist `V_error`. Update `θ` to reduce the error. Iterate until convergence.

The simplest update uses the pseudoinverse:

$$\theta \leftarrow \theta + \alpha \cdot J^+ \cdot V_{error}$$

where `α` is a step size. This is gradient-descent-like — fast when the Jacobian is well-conditioned, slow or oscillatory near singularities.

The **damped least squares** method (DLS, also called the *Levenberg-Marquardt* method) is more robust. It modifies the pseudoinverse to:

$$J^+_{DLS} = J^T (J J^T + \lambda^2 I)^{-1}$$

The damping term `λ^2 I` regularizes near singularities, trading some accuracy for stability. This is the method most production IK solvers use.

```python
def ik_dls(target_T, fk_func, jacobian_func, theta_init, max_iter=100, tol=1e-4, lam=0.01):
    """Inverse kinematics by damped least squares."""
    theta = theta_init.copy()
    for i in range(max_iter):
        T_current = fk_func(theta)
        V_error = pose_error_twist(T_current, target_T)
        if np.linalg.norm(V_error) < tol:
            return theta, True
        J = jacobian_func(theta)
        J_dls = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))
        theta = theta + J_dls @ V_error
    return theta, False

def pose_error_twist(T_current, T_target):
    """Compute a 6-vector twist that, applied to T_current, would yield T_target."""
    T_diff = np.linalg.inv(T_current) @ T_target
    log_diff = log_se3(T_diff)  # logarithm map back to twist
    return log_diff
```

(I have left `log_se3` as a placeholder; its implementation is similar to `log_so3` but for 4-by-4 matrices, with the position component handled accordingly.)

## 5.3 Optimization-based IK

For more sophisticated problems — IK with joint limits, with collision avoidance, with secondary objectives — frame IK as a nonlinear optimization problem:

minimize `|| T(θ) - T_target ||^2 + secondary_costs`
subject to `θ_min ≤ θ ≤ θ_max`, collision constraints

Solvers like SLSQP (Sequential Least SQuares Programming), interior-point methods, or modern packages like CasADi can handle this. The TRAC-IK solver and BioIK (used in MoveIt) are open-source examples.

The trade-off: optimization-based IK is slower per call (tens of milliseconds versus single milliseconds for closed-form), but it handles constraints and secondary objectives that simpler methods cannot.

## 5.4 Learning-based IK

A more recent direction trains a neural network to map target poses to joint configurations. For a given robot, you can generate a large dataset of (pose, configuration) pairs by running forward kinematics on randomly sampled configurations, then train a network to invert the mapping.

Pros: very fast at inference (microseconds), can learn complex configurations like elbow positions that classical methods struggle with.

Cons: not exact, requires retraining for each robot, has trouble handling joint limits and collision constraints natively.

For most robotics research, classical IK methods remain the workhorse. Learning-based IK is a niche tool for high-throughput applications.

\newpage

# Chapter 6 — Robot Dynamics: From Newton to Code

The first volume gave you the dynamics equation `τ = M(q)q̈ + C(q,q̇)q̇ + g(q)` and moved on. This chapter unpacks where each term comes from, derives the equation in two equivalent ways, and shows how to compute its components for a real robot.

## 6.1 The Lagrangian formulation

For a mechanical system with generalized coordinates `q`, the *Lagrangian* is `L = T - V` where `T` is kinetic energy and `V` is potential energy. The Euler-Lagrange equations give the dynamics:

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot q}\right) - \frac{\partial L}{\partial q} = \tau$$

For a robot arm, `q` is the joint angle vector. The kinetic energy is the sum of each link's kinetic energy. Each link `i` has mass `m_i`, inertia tensor `I_i`, linear velocity `v_{Ci}` of its center of mass, and angular velocity `ω_i`. The kinetic energy of link `i` is:

$$T_i = \frac{1}{2} m_i v_{Ci}^T v_{Ci} + \frac{1}{2} \omega_i^T I_i \omega_i$$

The total kinetic energy is the sum over all links, and it can be written as a quadratic form in the joint velocities:

$$T = \frac{1}{2} \dot q^T M(q) \dot q$$

where `M(q)` is the *mass matrix* (also called the inertia matrix). It is symmetric, positive definite, and depends on the configuration `q` because the geometry of the arm — and thus its effective inertia — changes with the joint angles.

The potential energy is the sum of `m_i g h_i(q)` over all links, where `g` is gravitational acceleration and `h_i(q)` is the height of link `i`'s center of mass.

Plugging into Euler-Lagrange and grinding through algebra (which I will not do here; the derivation is in Lynch and Park chapter 8) gives the manipulator equation:

$$\tau = M(q)\ddot q + C(q, \dot q)\dot q + g(q)$$

The terms:

- `M(q)\ddot q` is the inertial term — the torque needed to accelerate the joints.
- `C(q, \dot q)\dot q` captures Coriolis and centrifugal effects — torques that arise from the motion itself, not from acceleration. These are velocity-quadratic (involve products of joint velocities).
- `g(q)` is the gravity vector — the torques needed to hold the arm against gravity at configuration `q`.
- `τ` is the joint torque vector.

To simulate or control the arm, you need all three terms.

## 6.2 The Newton-Euler formulation

The Lagrangian approach is elegant but symbolic; for a six-link arm, the algebra produces hundreds of trigonometric terms. The *Newton-Euler* approach is more efficient in code.

Newton-Euler proceeds in two passes through the kinematic chain.

**Forward pass.** Starting from the base, compute each link's angular velocity, angular acceleration, linear acceleration of the center of mass. These propagate outward from the base through the chain.

**Backward pass.** Starting from the end-effector, compute each link's net force and torque from its own equations of motion (`F = ma`, `τ = Iα + ω × Iω`). These propagate inward, accumulating constraint forces from each joint.

The result is the joint torques `τ` for given `q`, `q̇`, `q̈`. By choosing different inputs, you can extract any of `M(q)`, `C(q,q̇)`, `g(q)` from this same procedure.

Newton-Euler is recursive, computationally efficient (linear in the number of links), and lends itself to clean implementation. Most modern robot dynamics libraries (Pinocchio, RBDL, the MATLAB Robotics System Toolbox) use it under the hood.

## 6.3 Computing dynamics in MATLAB

The Robotics System Toolbox provides built-in dynamics:

```matlab
% Load a robot
robot = loadrobot('frankaEmikaPanda', 'DataFormat', 'column');

% A configuration and velocity
q = [0; -pi/4; 0; -3*pi/4; 0; pi/2; pi/4];
qd = [0.1; 0.2; -0.1; 0.05; 0; 0.1; 0];

% Mass matrix
M = massMatrix(robot, q);

% Velocity product (Coriolis/centrifugal: C(q, qd) * qd)
Cqd = velocityProduct(robot, q, qd);

% Gravity torques
g = gravityTorque(robot, q);

% Total torque needed to maintain qdd = 0
qdd = zeros(7, 1);
tau = inverseDynamics(robot, q, qd, qdd);

% Or equivalently:
tau_check = M * qdd + Cqd + g;
disp(norm(tau - tau_check));   % should be near zero
```

In Python, the Pinocchio library is the open-source equivalent:

```python
import pinocchio as pin
import numpy as np

# Load a robot model (URDF)
model = pin.buildModelFromUrdf("panda.urdf")
data = model.createData()

q = np.array([0, -np.pi/4, 0, -3*np.pi/4, 0, np.pi/2, np.pi/4])
qd = np.array([0.1, 0.2, -0.1, 0.05, 0, 0.1, 0])
qdd = np.zeros(model.nv)

# Compute dynamics
M = pin.crba(model, data, q)              # mass matrix
Cqd = pin.computeCoriolisMatrix(model, data, q, qd) @ qd
g = pin.computeGeneralizedGravity(model, data, q)
tau = pin.rnea(model, data, q, qd, qdd)   # full inverse dynamics

# Check
print(np.linalg.norm(tau - (M @ qdd + Cqd + g)))
```

Both libraries are mature and fast. For research, you will often write your own simplified dynamics (for a 2-link or 3-link toy arm) to develop control algorithms, then validate against these libraries before deploying to real robots.

## 6.4 Forward dynamics: simulating motion

The reverse direction — given joint torques, compute the resulting accelerations — is *forward dynamics*:

$$\ddot q = M(q)^{-1} (\tau - C(q,\dot q)\dot q - g(q))$$

You compute `M`, `C`, `g`, then solve for `q̈`, then integrate forward in time using your favorite ODE integrator. Repeat. This is how all robot simulators work internally.

The two main choices in numerical integration are explicit methods (Euler, Runge-Kutta) and implicit methods (backward Euler, implicit midpoint). Explicit methods are simple but unstable for stiff systems; implicit methods are more complicated but stable. For most robot simulation, RK4 (fourth-order Runge-Kutta) with a small time step (1 millisecond or so) is a fine default.

