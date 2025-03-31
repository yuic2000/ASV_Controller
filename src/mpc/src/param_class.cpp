#include "../include/mpc/param_class.h"

using namespace Eigen;
using namespace std;

// Param class constructor
Param::Param() {
    mode = 1;                       // 1: zigzag, 2: circling
    dt = 0.04;                      // MPC simulation time step, (for blueboat: 25Hz)
    Nx = 12; 
    Nu = 2; 
    Nt = 125;                       // MPC planning time, 5 sec
    Nsim = (int)(ceil(65.0 / dt));  // Total simulation time, (60 + Nt*dt) sec
    V_c = 0.0;                      // current speed (m/s)
    beta_c = 1.0;                   // current direction (rad)

    // Mass and Inertia matrix
    // from Blueboat CAD model, 28.63 kg
    I = MatrixXd(6, 6);
    I << 14.47,  -0.35,  6.55,    0,   17.66,   0.25,
         -0.35,  21.55,  0.13, -17.66,   0,   -10.79,
          6.55,   0.13, 12.33,  -0.25, 10.79,      0,
          0,    -17.66, -0.25,  34.31,   0,        0,
          17.66,   0,   10.79,    0,   71.58,      0,
          0.25, -10.79,    0,     0,     0,    57.26;


    // state and control limits
    xmax = VectorXd::Constant(Nx, numeric_limits<double>::infinity()); 
    xmin = -xmax; 

    // // for otter simulation model control limits, used for vrx
    // umax = VectorXd::Constant(Nu, 115); 
    // umin = -VectorXd::Constant(Nu, 65);

    // // for blueboat model control limits, used for blueboat
    umax = VectorXd::Constant(Nu, 50);      // < 5.1 kgf 
    umin = -VectorXd::Constant(Nu, 26);     // > -2.7 kgf
    //////////////////////////////////////////////////////////////////////////////////

    // MPC cost matrices
    // [0:3] is angle: roll, pitch, yaw ,
    // [3:6] is position: x, y, z
    // [6:9] is angular velocity: roll, pitch, yaw,
    // [9:12] is linear velocity: surge (Vx), sway (Vy), heave (Vz),
    diag_values_cost = VectorXd(12);
    diag_values_cost << 1e-5, 1e-5, 0.5, 10, 10, 1e-5, 1e-5, 1e-5, 1e-5, 1, 1e-5, 1e-5;
    P = diag_values_cost.asDiagonal() * 10;
    Q = diag_values_cost.asDiagonal() * 1;
    R = MatrixXd::Identity(2, 2) * 1e-5;
}

void Param::set_desired_init_position(VectorXd& x_init) {
    this->q0_ref << 0, 0, 0, 1;                 // desired quaternion, x, y, z, w
    // this->p0_ref << 0, 0, 0;                 // desired position
    this->p0_ref << x_init.segment<3>(4);       // desired position
    this->w0_ref << 0, 0, 0.1;                  // desired yaw velocity, 0.1 rad/s
    this->v0_ref << 0.5, 0, 0;                  // desired surge velocity, 0.5 m/s

    this->x0_ref = VectorXd(7);
    this->x0_ref << this->q0_ref, this->p0_ref;
    this->xid_ref = VectorXd(6);
    this->xid_ref << this->w0_ref, this->v0_ref;

    this->init_pose = Matrix4d::Identity();
    this->init_pose.block<3, 1>(0, 3) = this->p0_ref;
}

void Param::set_reference_trajectory() {
    Matrix4d X = this->init_pose;

    MatrixXd X_ref(1, 7);
    X_ref.row(0) = this->x0_ref.transpose();
    MatrixXd xi_ref(1, 6);
    xi_ref.row(0) = this->xid_ref.transpose();

    for (int i = 0; i < Nsim; ++i) {
        VectorXd xid_ref_rt = this->xid_ref;

        if (mode == 1) {
            xid_ref_rt(2) = 0.1 * cos(i / 100.0);
        }

        Matrix4d Xi = Matrix4d::Zero();
        Xi.block<3, 3>(0, 0) = skew(xid_ref_rt.head<3>());
        Xi.block<3, 1>(0, 3) = xid_ref_rt.tail<3>();

        X = X * (Xi * dt).exp();
        Quaterniond new_quat(X.block<3, 3>(0, 0));
        VectorXd new_state(7);
        new_state << new_quat.coeffs(), X.block<3, 1>(0, 3);    // qx, qy, qz, qw, x, y, z
        X_ref.conservativeResize(X_ref.rows() + 1, NoChange);
        X_ref.row(X_ref.rows() - 1) = new_state.transpose();
        xi_ref.conservativeResize(xi_ref.rows() + 1, NoChange);
        xi_ref.row(xi_ref.rows() - 1) = xid_ref_rt.transpose();
    }

    this->X_ref = X_ref;
    this->xi_ref = xi_ref;
}

void Param::generate_zigzag_waypoints() {
    double amplitude = 1.0;
    double wavelength = 10.0;
    int segment_steps = Nsim;  // Number of waypoints

    MatrixXd X_ref(1, 7);
    X_ref.row(0) = this->x0_ref.transpose();

    double x_start = this->x0_ref[4];
    double y_start = this->x0_ref[5];   // Starting position

    VectorXd x_vals = VectorXd::LinSpaced(segment_steps, x_start, x_start + 3 * wavelength);  // X-axis progression
    VectorXd y_vals = y_start + amplitude * (2 * M_PI / wavelength * (x_vals.array() - x_start)).sin();  // Sinusoidal y

    for (int i = 0; i < segment_steps; ++i) {
        double x = x_vals[i];
        double y = y_vals[i];

        // Compute yaw angle as tangent to the sine curve
        double theta = atan2(
            (2 * M_PI / wavelength) * amplitude * cos((2 * M_PI / wavelength) * x),
            1
        );

        Quaterniond quaternion(AngleAxisd(theta, Vector3d::UnitZ()));   // quat: w, x, y, z

        // Store waypoint
        VectorXd waypoint(7);
        waypoint << quaternion.coeffs(), x, y, 0;                       // qx, qy, qz, qw, x, y, z
        X_ref.conservativeResize(X_ref.rows() + 1, NoChange);
        X_ref.row(X_ref.rows() - 1) = waypoint.transpose();
    }

    this->X_ref = X_ref;
}

void Param::generate_circle_waypoints() {
    double radius = 5.0;

    double x_start = this->x0_ref[4];
    double y_start = this->x0_ref[5];   // Starting position

    Vector2d center(x_start, y_start + radius);  // Center at (0,5)
    int num_waypoints = Nsim;  // Number of waypoints

    MatrixXd X_ref(1, 7);
    X_ref.row(0) = this->x0_ref.transpose();

    VectorXd angles = VectorXd::LinSpaced(num_waypoints, -M_PI / 2, 3 * M_PI / 2);  // Counterclockwise from (0,0)

    for (int i = 0; i < num_waypoints; ++i) {
        double theta = angles[i];
        double x = center[0] + radius * cos(theta);
        double y = center[1] + radius * sin(theta);

        double yaw_angle = theta + M_PI / 2;  // Align tangent direction
        Quaterniond quaternion(AngleAxisd(yaw_angle, Vector3d::UnitZ()));  // quat: w, x, y, z

        VectorXd waypoint(7);
        waypoint << quaternion.coeffs(), x, y, 0;                          // qx, qy, qz, qw, x, y, z

        X_ref.conservativeResize(X_ref.rows() + 1, NoChange);
        X_ref.row(X_ref.rows() - 1) = waypoint.transpose();
    }

    this->X_ref = X_ref;
}

void Param::waypoints2twists() {
    MatrixXd waypoints = this->X_ref;

    MatrixXd xi_ref(1, 6);
    xi_ref.row(0) = this->xid_ref.transpose();

    // Generate twist based on waypoints
    for (int k = 0; k < waypoints.rows() - 1; ++k) {
        // Get the current and next waypoint
        Matrix4d X1 = pose_to_T_matrix(waypoints.row(k));  // Transformation of waypoint k
        Matrix4d X2 = pose_to_T_matrix(waypoints.row(k + 1));  // Transformation of waypoint k+1

        // Compute the relative transformation
        Matrix4d delta_X = X1.inverse() * X2;

        // Compute the twist using logm (logarithm map of the relative transformation)
        Matrix4d twist_matrix = delta_X.log() / dt;  // Divide by dt to get the twist

        // Angular velocity (from skew-symmetric part of matrix)
        // Vector3d omega(twist_matrix(2, 1), twist_matrix(0, 2), twist_matrix(1, 0));  
        // Vector3d omega(0, 0, 0.1);
        Vector3d omega(0, 0, twist_matrix(1, 0));

        // Linear velocity (from translational part)
        // Vector3d v = twist_matrix.block<3, 1>(0, 3);
        // Vector3d v(0.5, 0, 0);
        Vector3d v(twist_matrix(0, 3), 0, 0);

        VectorXd twist(6);
        twist << omega, v;
        xi_ref.conservativeResize(xi_ref.rows() + 1, NoChange);
        xi_ref.row(xi_ref.rows() - 1) = twist.transpose();
    }

    this->xi_ref = xi_ref;
}

// Logger class constructor
Logger::Logger(int Nsim, int Nt) {
    X = MatrixXd::Zero(13, Nsim - Nt + 1);  // +1 for the initial state
    Err = MatrixXd::Zero(6, Nsim - Nt);
    U = MatrixXd::Zero(2, Nsim - Nt);
    Err2 = MatrixXd::Zero(4, Nsim - Nt);
    taken_time = 0.0;
}