# ASV_Controller

ASV_Controller is a project that implements a Lie MPC (Model Predictive Control) controller in both C++ and Python. This project contains two main components:

- **mpc**: A Lie MPC controller implemented in C++.
- **mpc_python**: A Lie MPC controller implemented in Python.

## Requirements

- ROS 2 (Rolling or later)
- colcon build tool
- C++ and Python development environment

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

After building the project, you can run the C++ Lie MPC controller using the following command:

```bash
ros2 run mpc_python main
```