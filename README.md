# ASV_Controller

ASV_Controller is a project that implements a Lie MPC (Model Predictive Control) controller in both C++ and Python. This project contains two main components:

- **mpc**: a Lie MPC controller implemented in C++.
- **mpc_python**: a Lie MPC controller implemented in Python.

## Requirements

- ROS 2 (Humble or later)
- colcon build tool
- C++ and Python development environment

## Used Packages

### OSQP - QP Solver

This project uses the [OSQP](https://osqp.org/docs/get_started/) solver to solve the optimization problem in the Lie MPC controller. Make sure to install this package to run the code.

## Installation

Follow these steps to build and install the project:

1. Clone the repository to your workspace:

    ```bash
    git clone <repository_url>
    ```

2. Build the project:

    ```bash
    colcon build --merge-install
    ```

## Running the Code

### C++ Lie MPC Controller

After building the project, you can run the C++ Lie MPC controller using the following command:

```bash
ros2 run mpc mpc
```

### Python Lie MPC Controller

After building the project, you can run the Python Lie MPC controller using the following command:

```bash
ros2 run mpc_python main
```