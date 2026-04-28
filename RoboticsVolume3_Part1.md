---
title: "Robotics in Depth"
subtitle: "A Second Volume on Kinematics, Dynamics, Control, Motion Planning, Mobile Robotics, and SLAM, with MATLAB and Python Throughout"
author: "Written for Rupesh Shrestha"
---

# Preface — A Second Pass at Robotics

This is a companion volume to *From Zero to AgRobotics PhD*. That first book established the foundations: linear algebra, calculus, probability, basic kinematics, computer vision, machine learning, and an introduction to ROS2. This book picks up from those foundations and goes deeper in the directions a PhD student in agricultural robotics actually needs depth.

Three things motivate this second volume.

First, MATLAB. The first book taught Python only, which was a real omission. MATLAB remains the language of mechanical engineering education, of many robotics textbooks, and of legacy lab code at Cornell and elsewhere. The *Modern Robotics* textbook by Lynch and Park ships with a MATLAB toolbox. Peter Corke's *Robotics, Vision and Control* is built around MATLAB. Simulink, MATLAB's block-diagram environment, is the standard tool for control system prototyping. You will encounter MATLAB regularly, and you cannot ignore it. So this book teaches MATLAB from the basics in Chapter 2 and gives MATLAB code alongside Python code throughout, side by side, so you become bilingual.

Second, depth. The first book had to cover everything in twenty-one thousand words. That meant some topics — dynamics, control, motion planning, mobile robotics, SLAM, sensor fusion — got shorter treatments than they deserve. This book gives them their proper space. You will see derivations, not just statements. You will see the *why* behind every result, not just the *what*. You will see worked examples that you would meet in a real research project, not just textbook toys.

Third, breadth. The first book focused tightly on what you needed to be ready for Karkee's lab. This book covers a wider robotics landscape — including topics like trajectory optimization, model predictive control, behavior trees, multi-robot systems, simulation-to-real transfer, and reinforcement learning for robotics — that you will encounter in research papers and in adjacent labs. You may not use all of it, but you should be conversant.

A few promises about how I will write, the same as in the previous books.

Plain language first, notation second. I will not introduce a symbol before I have explained the idea in words.

Every abbreviation spelled out at first appearance, and again when it has been a while. PID means Proportional-Integral-Derivative. MPC means Model Predictive Control. SLAM means Simultaneous Localization and Mapping. RRT means Rapidly-exploring Random Tree. EKF means Extended Kalman Filter. There will be many.

Code in both MATLAB and Python, side by side, as much as possible. When something is only available in one, I will say so and explain why.

Real research applications throughout. When we discuss model predictive control, the example will be an arm tracking an apple in wind. When we discuss SLAM, the example will be a ground robot mapping an orchard. The math will not be abstract; it will be in service of problems you will actually face.

Honest about hard parts. Some material in this book is genuinely difficult. I will say when something is hard. I will give you intuition first, then the rigorous form, then code so you can play with it. If you read the rigorous form and feel lost, go back to the intuition and the code. That cycle is how mathematical concepts settle into your brain permanently.

Fun where I can. Robotics is one of the few engineering disciplines where mathematics, physics, computation, and creativity all meet. There are problems in this field that are genuinely beautiful — the way a robot's kinematics decompose into geometric layers, the way the Kalman filter elegantly fuses noisy sensors, the way a sampling-based motion planner finds paths through cluttered space by random trial and persistent improvement. I will try to share why I find these things beautiful.

How to read this book. Read sequentially. Run every code example, in both languages where I provide both. Build the projects at the end of each chapter. Keep a notebook (paper or digital) where you re-derive equations in your own hand — typing through code is not the same as deriving math; both matter. Plan to spend four to six months on this volume at your part-time pace, after completing the first volume.

We begin with a chapter on what robotics actually is, because the field has expanded so much in the last decade that even people in it disagree about its boundaries.

\newpage

# Part I — Foundations and Tools

# Chapter 1 — What Robotics Is, Today

Before we touch any equation, let us be honest about what robotics is and how it has changed.

## 1.1 The classical definition and why it is now incomplete

The classical definition: robotics is the engineering discipline concerned with the design, construction, operation, and use of robots — physical machines that sense, decide, and act in the physical world. By that definition, a factory arm welding car bodies is a robot, and so is a lawnmower that cuts grass on a fixed pattern.

The classical definition is fine as far as it goes, but it underestimates what modern robotics has become. A modern robot is rarely just hardware. It is a tightly coupled system of mechanical structure, electrical components, sensors, computing, software, machine learning models, and increasingly cloud services. A self-driving car perceives the world with cameras, radars, and lidars, runs neural networks on GPUs to identify objects, plans motion through optimization, executes commands through low-level controllers, and updates itself over the air from a fleet's collected experience. To call it just a "physical machine" is technically true and practically useless.

So I will use a working definition that suits a modern PhD researcher: **robotics is the discipline of building autonomous systems that perceive their environment, decide how to act, and act on that decision in the physical world**. The three verbs — perceive, decide, act — are the three pillars of the field. Each is its own enormous subdiscipline.

**Perception** is computer vision plus other sensors. It is how the robot turns raw measurements (images, depths, lasers, inertial accelerations) into understanding (the apple is here, the tree branch is there, the floor is uneven).

**Decision-making** is planning, control, and increasingly learned policy. It is how the robot turns understanding into intentions (move there, grasp this, avoid that).

**Action** is actuation, motion control, mechanical design, and the physical execution of intentions. It is how the robot turns intentions into motion (motors spin at these speeds, joints reach these angles, the gripper closes with this much force).

A modern robot is the integration of all three pillars. A research career typically specializes in one or two pillars while remaining literate in all three.

## 1.2 The pillars in agricultural robotics

For Karkee's lab specifically, the picture maps onto agriculture as follows.

**Perception** in agriculture is brutal. Outdoor lighting changes by the minute. Crops are visually similar to weeds. Fruits are partially occluded by leaves and other fruits. Wind moves everything. Surfaces vary from wet to dry to dusty. The perception techniques that work in factories — controlled lighting, fixed camera poses, repeatable scenes — do not transfer. Agricultural perception is a frontier for computer vision research, and it is much of what makes Karkee's group interesting.

**Decision-making** in agriculture has its own specifics. A robot picking apples must choose which apple to pick first, plan a motion that avoids branches and other fruits, recover when a grasp slips, and decide when a tree is fully harvested versus when more searching is warranted. The decisions span time scales — milliseconds for low-level reflexes, seconds for grasps, minutes for tree-level strategy, hours for orchard-level coverage.

**Action** in agriculture means manipulators in cluttered canopies, mobile bases on uneven terrain, end-effectors that can grasp without bruising, and locomotion schemes that minimize soil compaction. Mechanical design choices have consequences for everything else.

Throughout this book I will draw examples from agricultural settings whenever the example illustrates a general concept. When the general concept is more cleanly illustrated in a different setting, I will use it but return to agriculture when I can.

## 1.3 Five trends that shape the field today

Five trends in the last fifteen years have changed what robotics research looks like. You should be aware of all five so that you can read modern papers in context.

**Deep learning.** Ten years ago, robot perception relied on hand-engineered features and classical algorithms. Today, virtually all serious perception systems use neural networks, and many decision-making systems do too. This shift is irreversible. Agricultural robotics has been an enthusiastic adopter, with deep learning now standard for fruit detection, weed identification, yield estimation, and disease diagnosis.

**Simulation.** Modern robots are developed primarily in simulation. Hardware is expensive, slow, dangerous when controllers misbehave, and often unavailable. Simulators like Gazebo, Isaac Sim, MuJoCo, and PyBullet let researchers train, test, and iterate in software at speeds that hardware cannot match. The challenge of transferring controllers from simulation to reality — *sim-to-real* — is itself an active research area.

**Data and benchmarks.** Robotics research has moved toward shared datasets and benchmarks. Open agricultural datasets like MinneApple (apple detection), CWFID (crop-weed segmentation), and Sugar Beets 2016 let researchers compare methods fairly. Open robot platforms like the UR5 arm and Turtlebot let methods be reproduced. This was not the norm a decade ago.

**Reinforcement learning and learning-based control.** A growing fraction of robotics research uses reinforcement learning — algorithms that learn policies from trial and error — for tasks that are hard to program by hand. Locomotion of legged robots, dexterous manipulation, and certain perception-action loops are increasingly learned rather than engineered. The field is in transition.

**Foundation models.** Large language models and vision-language models are entering robotics in 2023-2025. Models that ground language to action, that follow natural-language commands, that reason about scenes — these are reshaping what robots can do at the high-level decision tier. By the time you are deep in your PhD, foundation models will be central to many robotics research agendas.

You should keep all five trends in mind as you read papers and choose your own research directions.

## 1.4 The structure of this book

I will spend Part I on tools — MATLAB and Python in detail, the math we will need beyond what the first book established. Part II goes deep into kinematics and dynamics, including topics the first book only sketched. Part III covers control in earnest, from PID to optimal control to model predictive control. Part IV is motion planning and trajectory optimization. Part V is mobile robotics, sensor fusion, and SLAM. Part VI covers perception in more depth than the first book, with attention to multimodal sensing for agriculture. Part VII surveys learning-based robotics — reinforcement learning, imitation learning, sim-to-real transfer. Part VIII brings everything together with research-level case studies and closes with advice on the research career itself.

We begin with the tools you will use every day.

\newpage

# Chapter 2 — MATLAB from First Principles

You may not have used MATLAB before, or you may have used it casually for homework problems and forgotten everything. This chapter assumes nothing.

## 2.1 What MATLAB is and why it persists

MATLAB stands for Matrix Laboratory. It was created in the late 1970s by Cleve Moler at the University of New Mexico to give students access to numerical linear algebra without having to write Fortran. It became commercial in 1984 through MathWorks, and it has been the standard scientific computing environment in engineering for forty years.

There is a real question about why MATLAB persists when Python is free and increasingly capable. The honest answer has several parts. MATLAB has excellent numerical libraries that are tightly integrated and well-documented. Its plotting is straightforward and produces publication-quality figures by default. Simulink, the block-diagram extension, has no equivalent in Python — it is the standard tool for control engineers worldwide. The Robotics Toolbox by Peter Corke, the *Modern Robotics* code from Lynch and Park, and many academic robotics packages were written in MATLAB. Industry partners, especially in aerospace and automotive, run MATLAB-based pipelines. Cornell's mechanical engineering program teaches MATLAB as the default. You will use it.

Python, of course, is free and open source. MATLAB is licensed; you will need access through Cornell or through your engagement with VPE. Both have their place; you will work in both.

## 2.2 Getting MATLAB

If you have access through an institution, install from MathWorks' site with your institutional credentials. Otherwise, MATLAB Online provides a browser-based version that you can use freely with a MathWorks account, with limitations on session time and storage.

For learning, MATLAB Online is sufficient. For research-grade work later, you will want the desktop version with appropriate toolboxes — at minimum the Robotics System Toolbox and the Optimization Toolbox.

## 2.3 The MATLAB workspace

When you launch MATLAB, you see four panes. The Command Window is where you type commands and see results. The Workspace shows all variables currently in memory. The Current Folder shows files in your working directory. The Editor (which opens when you create or open a file) is where you write scripts and functions.

A first session:

```matlab
>> a = 3
a =
     3

>> b = 4;     % the semicolon suppresses the output
>> c = a + b
c =
     7

>> v = [1 2 3]    % a row vector
v =
     1     2     3

>> u = [1; 2; 3]  % a column vector
u =
     1
     2
     3

>> v * u   % row times column = scalar dot product (matrix mult)
ans =
    14
```

A few MATLAB conventions to absorb early.

The semicolon at end of a line suppresses output. Without it, MATLAB prints the result of every statement. In scripts you almost always want semicolons; you only omit them when you specifically want to see a value during debugging.

`ans` is the default variable for any unassigned result. You can refer to it in subsequent commands but it gets overwritten quickly.

Comments start with `%`. Two `%%` together start a *cell* (a section), and the editor lets you run cells individually.

Whitespace mostly does not matter, but commas separate row elements and semicolons separate rows in matrix literals. So `[1 2 3]` and `[1, 2, 3]` are the same row vector; `[1 2 3; 4 5 6]` is a 2-by-3 matrix.

## 2.4 MATLAB versus Python: side-by-side basics

Let me show you the same operations in both languages so the parallel becomes natural.

```matlab
% MATLAB

% Vector creation
v = [1 2 3];
zeros_vec = zeros(1, 5);
ones_vec  = ones(1, 5);
range_vec = 1:10;          % 1, 2, ..., 10
spaced    = linspace(0, 1, 11);

% Matrix creation
A = [1 2 3; 4 5 6; 7 8 9];
I = eye(3);
R = rand(3, 4);            % uniform [0,1] random

% Indexing - MATLAB uses 1-based indexing
A(1, 1)        % = 1
A(2, :)        % the second row: [4 5 6]
A(:, 3)        % the third column: [3; 6; 9]
A(1:2, 2:3)    % submatrix: [2 3; 5 6]
A(end, end)    % last element: 9
```

```python
# Python (NumPy)
import numpy as np

# Vector creation
v = np.array([1, 2, 3])
zeros_vec = np.zeros(5)
ones_vec  = np.ones(5)
range_vec = np.arange(1, 11)            # 1, 2, ..., 10
spaced    = np.linspace(0, 1, 11)

# Matrix creation
A = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
I = np.eye(3)
R = np.random.rand(3, 4)

# Indexing - Python uses 0-based indexing
A[0, 0]        # = 1
A[1, :]        # the second row: array([4, 5, 6])
A[:, 2]        # the third column: array([3, 6, 9])
A[0:2, 1:3]    # submatrix
A[-1, -1]      # last element: 9
```

Three differences will trip you up between languages. First, MATLAB indexes from 1, Python from 0. Second, MATLAB uses parentheses for indexing, Python uses square brackets. Third, MATLAB's range syntax is `start:end` and is *inclusive* of end; Python's `np.arange(start, end)` is *exclusive* of end.

These differences are small but you will misindex daily for the first few weeks of using both. Just expect it.

## 2.5 Linear algebra in MATLAB

MATLAB was built for linear algebra; it shines here.

```matlab
% Define matrices
A = [3 1; 1 2];
b = [9; 8];

% Matrix-vector product
A * b

% Matrix-matrix product
B = [1 0; 2 3];
A * B

% Element-wise product (the dot prefix indicates element-wise)
A .* B

% Inverse (rarely use directly; use \ instead)
A_inv = inv(A);

% Solve A*x = b efficiently
x = A \ b;       % This is MATLAB's "matrix left divide" - solves systems
                 % It is faster and more stable than inv(A) * b

% Determinant
det(A)

% Transpose
A'               % conjugate transpose; for real matrices same as A.'
A.'              % plain transpose

% Eigenvalues and eigenvectors
[V, D] = eig(A); % V is eigenvector matrix, D is diagonal of eigenvalues

% SVD
[U, S, V] = svd(A);

% Norm
norm(b)           % 2-norm by default
norm(b, 1)        % 1-norm
norm(b, inf)      % infinity-norm
```

Compare to Python:

```python
import numpy as np

A = np.array([[3, 1], [1, 2]])
b = np.array([9, 8])

A @ b                 # matrix-vector product
B = np.array([[1, 0], [2, 3]])
A @ B                 # matrix-matrix product
A * B                 # element-wise product
A_inv = np.linalg.inv(A)
x = np.linalg.solve(A, b)

np.linalg.det(A)
A.T                   # transpose
eigenvalues, V = np.linalg.eig(A)
U, S, Vt = np.linalg.svd(A)
np.linalg.norm(b)
np.linalg.norm(b, 1)
np.linalg.norm(b, np.inf)
```

The structures are nearly identical. MATLAB's slightly more compact syntax (the backslash for solve, the apostrophe for transpose) is partly why people who came up in MATLAB find Python more verbose; partly why Python feels less idiosyncratic to people who came up in general-purpose programming. Use whichever feels natural; both work.

## 2.6 Plotting in MATLAB

MATLAB plotting is one of its most polished features.

```matlab
% A basic plot
x = linspace(0, 2*pi, 100);
y = sin(x);
plot(x, y)
xlabel('x')
ylabel('sin(x)')
title('A sine wave')
grid on

% Multiple lines
y2 = cos(x);
plot(x, y, 'b-', x, y2, 'r--')   % blue solid, red dashed
legend('sin(x)', 'cos(x)')

% Subplots
figure;
subplot(2, 1, 1); plot(x, y);  title('sin')
subplot(2, 1, 2); plot(x, y2); title('cos')

% 3D plotting
[X, Y] = meshgrid(linspace(-3, 3, 50));
Z = X.^2 + Y.^2;
figure;
surf(X, Y, Z)
shading interp
colormap parula
xlabel('x'); ylabel('y'); zlabel('z')
```

Compare to Python:

```python
import matplotlib.pyplot as plt
import numpy as np

x = np.linspace(0, 2*np.pi, 100)
y = np.sin(x)
plt.plot(x, y)
plt.xlabel('x')
plt.ylabel('sin(x)')
plt.title('A sine wave')
plt.grid(True)
plt.show()

# Multiple lines
y2 = np.cos(x)
plt.plot(x, y, 'b-', label='sin(x)')
plt.plot(x, y2, 'r--', label='cos(x)')
plt.legend()
plt.show()

# Subplots
fig, axes = plt.subplots(2, 1)
axes[0].plot(x, y); axes[0].set_title('sin')
axes[1].plot(x, y2); axes[1].set_title('cos')
plt.show()

# 3D plotting
from mpl_toolkits.mplot3d import Axes3D
X, Y = np.meshgrid(np.linspace(-3, 3, 50), np.linspace(-3, 3, 50))
Z = X**2 + Y**2
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X, Y, Z, cmap='viridis')
plt.show()
```

MATLAB plotting feels slightly more polished out of the box; matplotlib gives more control if you want it. Both are fine.

## 2.7 Functions and scripts in MATLAB

A *script* is a file with a `.m` extension containing a sequence of commands. Scripts run in the base workspace and can see and modify variables there.

A *function* is a file with a `.m` extension whose first line begins with `function`. Functions have their own local workspace.

```matlab
% A function in a file called add_numbers.m
function y = add_numbers(a, b)
    y = a + b;
end
```

You call it like any built-in:

```matlab
>> add_numbers(3, 4)
ans =
     7
```

Functions can return multiple outputs:

```matlab
function [mean_val, std_val] = describe(data)
    mean_val = mean(data);
    std_val = std(data);
end
```

```matlab
>> [m, s] = describe([1 2 3 4 5])
m =
     3
s =
    1.5811
```

Anonymous functions are short inline functions, like Python's lambdas:

```matlab
square = @(x) x.^2;
square(5)   % returns 25
```

The `@` introduces an anonymous function. The `.^` is element-wise power, which works on arrays.

## 2.8 Object-oriented MATLAB

MATLAB has classes too, though the syntax is verbose. You will see classes in robotics toolboxes. A typical class file:

```matlab
classdef Robot
    properties
        name
        num_joints
        joint_angles
    end
    
    methods
        function obj = Robot(name, n)   % constructor
            obj.name = name;
            obj.num_joints = n;
            obj.joint_angles = zeros(1, n);
        end
        
        function obj = set_joints(obj, angles)
            obj.joint_angles = angles;
        end
        
        function pos = forward_kinematics(obj)
            % ... computation ...
            pos = [0 0 0];   % placeholder
        end
    end
end
```

Note the unusual MATLAB convention: methods that modify the object return the modified object, because MATLAB's default value semantics mean the caller's object is otherwise not changed. So `robot = robot.set_joints([0.1 0.2 0.3])` is correct, while `robot.set_joints([0.1 0.2 0.3])` alone would not modify `robot` in the caller's workspace. Python's reference semantics work differently (and arguably more naturally for OOP), but MATLAB has its reasons.

## 2.9 The MATLAB Robotics System Toolbox

For robotics work, MATLAB's Robotics System Toolbox provides:

- Rigid body tree models for representing robots.
- Forward and inverse kinematics solvers.
- Path planners (RRT, A-star).
- Sensor models (cameras, lidars, IMUs).
- ROS and ROS2 integration (you can publish and subscribe to ROS topics from MATLAB).
- A simulator with collision checking.

```matlab
% Load a built-in robot model
robot = loadrobot("kinovaGen3", "DataFormat", "row");
show(robot);          % visualize the robot

% Forward kinematics
config = robot.homeConfiguration;
T = getTransform(robot, config, robot.BodyNames{end});
disp(T);

% Inverse kinematics
ik = inverseKinematics('RigidBodyTree', robot);
target = trvec2tform([0.5 0.0 0.5]);    % target pose
weights = ones(1, 6);                    % equal weight on all DOF
init_guess = robot.homeConfiguration;
[soln, info] = ik(robot.BodyNames{end}, target, weights, init_guess);
disp(soln);
```

This is actually a very capable system. For prototyping kinematics and motion planning algorithms before deploying to ROS2, MATLAB is genuinely competitive with any Python alternative.

## 2.10 Simulink, briefly

Simulink is MATLAB's block-diagram environment. You build a system as a graph of blocks — sources (signal generators, sensor readings), operators (sums, integrators, transfer functions), controllers (PID blocks), and sinks (scopes, file outputs). The simulator integrates the resulting system of differential equations forward in time.

For control engineers, Simulink is the standard prototyping tool. You design a controller, simulate the closed-loop system, tune parameters, generate code that runs on real hardware. Many automotive and aerospace control systems are designed this way.

I will not cover Simulink in depth here because it is graphical and not amenable to text. But know it exists, know it is the standard, and plan to learn at least its basics if your research takes you toward control. The MathWorks free online courses ("Simulink Onramp") are a reasonable starting point, two or three hours of work.

## 2.11 Practical bilingualism: when to use which

A pragmatic guide to choosing language for a given task:

For prototyping a kinematics or control algorithm: MATLAB is faster, especially with the Robotics Toolbox. The plotting just works. The math feels mathematical.

For deep learning, computer vision, or anything involving large modern ML models: Python wins decisively. PyTorch and TensorFlow have no MATLAB equivalents at the same maturity.

For ROS2 nodes that talk to real robots: Python or C++. ROS2's MATLAB support exists but is awkward and less feature-complete.

For Simulink-based control prototyping: MATLAB, no contest.

For data analysis from experiments: either works. Python's pandas is excellent; MATLAB's table type is also good.

For research code you intend to publish and have other people reproduce: Python wins, because more people can run it without a license.

For agriculture-specific simulation environments: usually Python (Gazebo, Isaac Sim) or C++. MATLAB simulation is fine for kinematics, less appropriate for full multi-physics agricultural environments.

In your PhD work you will toggle between both languages weekly. Get comfortable in both.

\newpage

# Chapter 3 — The Mathematics of Three-Dimensional Motion, Revisited

The first volume covered the basics of rotation matrices, quaternions, and homogeneous transformations. This chapter goes deeper. It introduces the Lie group and Lie algebra structure of rotations and rigid motions, which sounds intimidating but is genuinely useful and shows up in modern robotics papers constantly. I will build it from intuition and tie it firmly to code.

## 3.1 Why we need more than rotation matrices

A rotation matrix is a perfectly fine way to *store* an orientation. Three by three, nine numbers, six constraints, three effective degrees of freedom. You can apply it to vectors. You can compose rotations by multiplication. What more could you need?

Here is what you need. Suppose you want to take a *small step* of orientation. You have a current rotation `R`, and you want to nudge it a little — say, by 0.01 radians around some axis. How do you parameterize that nudge so that you can compute it, integrate it over time, take derivatives of functions that depend on it?

You cannot simply add a small matrix to `R`, because the result will not be a rotation matrix in general. Rotation matrices live on a curved surface in the space of 3-by-3 matrices — the surface defined by the constraints `R^T R = I` and `det(R) = 1`. Adding a generic small matrix moves you off the surface. You need a way to move on the surface.

This is where the Lie group structure comes in. The set of all rotations in three dimensions, called *SO(3)* (the Special Orthogonal group of dimension three), is a Lie group: a smooth, curved space that also has a multiplication operation. Attached to every Lie group is a Lie algebra — a flat space (specifically, a vector space) that locally approximates the curved group near the identity. For SO(3), the Lie algebra is called *so(3)* (lowercase, distinguishing it from the group), and it consists of *skew-symmetric* matrices.

The exponential map takes you from the Lie algebra (flat) to the Lie group (curved), and the logarithm map takes you back. With these two maps, you can do calculus on rotations: take derivatives, integrate small motions, optimize over orientations.

This sounds abstract. Let me make it concrete.

## 3.2 The skew-symmetric matrix

A skew-symmetric three-by-three matrix has the form:

$$\hat\omega = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}$$

It has three independent numbers (`ω_1`, `ω_2`, `ω_3`) — the same number as the degrees of freedom in a rotation.

The hat operator `^` converts a 3-vector `ω = (ω_1, ω_2, ω_3)` into this skew-symmetric matrix. There is also an inverse, the *vee* operator `v`, that extracts the vector from the matrix.

A useful identity: for any vectors `ω` and `v`, we have `ω̂ v = ω × v`. The skew-symmetric matrix multiplication implements the cross product. Let me verify:

In MATLAB:

```matlab
function S = skew(w)
    S = [   0   -w(3)  w(2);
          w(3)   0   -w(1);
         -w(2)  w(1)   0  ];
end

w = [1; 2; 3];
v = [4; 5; 6];

% Test: skew(w) * v should equal cross(w, v)
S = skew(w);
disp(S * v)
disp(cross(w, v))
% Both should give the same answer
```

In Python:

```python
import numpy as np

def skew(w):
    return np.array([
        [0,    -w[2],  w[1]],
        [w[2],  0,    -w[0]],
        [-w[1], w[0],  0   ]
    ])

w = np.array([1, 2, 3])
v = np.array([4, 5, 6])

S = skew(w)
print(S @ v)
print(np.cross(w, v))
# Same answer: [-3, 6, -3]
```

The fact that the cross product can be written as a matrix multiplication is one of those quietly important facts. It lets you convert nonlinear-looking expressions involving cross products into linear-algebra expressions, which is useful in derivations and in optimization.

## 3.3 The exponential map: from angular velocity to rotation

Now the payoff. Suppose a body is rotating around axis `ω̂` (a unit vector) at angular speed `θ` rad/sec. After time `t = 1` second, the rotation accumulated is by angle `θ` around axis `ω̂`. This rotation is given by *Rodrigues' formula*:

$$R = I + \sin\theta \cdot \hat{ω̂} + (1 - \cos\theta) \cdot \hat{ω̂}^2$$

where `ω̂_hat` is the skew-symmetric matrix of `ω̂` (a unit vector).

This formula tells you how to convert axis-angle representation (axis `ω̂`, angle `θ`) into a rotation matrix. It is the closed form of the *exponential map* `exp: so(3) → SO(3)`. Specifically, if `Ω = θ · ω̂_hat` is the skew-symmetric matrix encoding "rotation by `θ` around `ω̂`," then `R = exp(Ω)` where the exponential is the matrix exponential.

Why is this called "exp"? Because the matrix exponential is defined by the same Taylor series as the scalar exponential: `exp(M) = I + M + M^2/2! + M^3/3! + ...`. For skew-symmetric matrices, this infinite series happens to sum to Rodrigues' formula. The connection between the matrix exponential and Rodrigues' formula is one of those mathematical satisfactions: deep theory, clean formula.

In MATLAB:

```matlab
function R = expSO3(omega)
    % Convert axis-angle vector omega = theta * omega_hat to rotation matrix
    theta = norm(omega);
    if theta < 1e-10
        R = eye(3);
        return;
    end
    omega_hat = omega / theta;
    K = skew(omega_hat);
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end

% Test: 90-degree rotation around z-axis
omega = [0; 0; pi/2];
R = expSO3(omega);
disp(R);
% Should give:
% [ 0 -1 0;
%   1  0 0;
%   0  0 1]
```

In Python:

```python
import numpy as np

def exp_so3(omega):
    theta = np.linalg.norm(omega)
    if theta < 1e-10:
        return np.eye(3)
    omega_hat = omega / theta
    K = skew(omega_hat)
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

omega = np.array([0, 0, np.pi/2])
R = exp_so3(omega)
print(R)
```

The inverse — converting a rotation matrix back to axis-angle form — is the *logarithm map*. It is also derivable in closed form, with edge cases when `θ` is near 0 or `π`: