#include "../../include/mpc/utils/gnc.h"

// Saturates a signal x such that x_min <= x <= x_max
double sat(double x, double x_min, double x_max) {
    return std::max(x_min, std::min(x, x_max));
}

// Computes the 3x3 vector skew-symmetric matrix, S(a) = -S(a)'
// S(a)b = a x b
Matrix3d Smtrx(const Vector3d& a) {
    Matrix3d S;
    S << 0, -a.z(), a.y(),
         a.z(), 0, -a.x(),
         -a.y(), a.x(), 0;

    return S;
}

// Computes the 6x6 system transformation matrix, inv(H(r)) = H(-r)
MatrixXd Hmtrx(const Vector3d& r) {
    
    /*
    H = [eye(3)     S'
         zeros(3,3) eye(3) ]

    If r = r_bg is the vector from the CO to the CG, the model matrices in CO and
    CG are related by: M_CO = H(r_bg)' * M_CG * H(r_bg). Generalized position and
    force satisfy: eta_CO = H(r_bg)' * eta_CG and tau_CO = H(r_bg)' * tau_CG 
    */

    Matrix3d S = Smtrx(r);
    
    MatrixXd H(6, 6);
    H << Matrix3d::Identity(), S.transpose(),
         Matrix3d::Zero(), Matrix3d::Identity();
    return H;
}

// Computes the Euler angle rotation matrix R in SO(3) using the zyx convention
Matrix3d Rzyx(double phi, double theta, double psi) {
    double cphi = cos(phi);
    double sphi = sin(phi);
    double cth = cos(theta);
    double sth = sin(theta);
    double cpsi = cos(psi);
    double spsi = sin(psi);

    Matrix3d R;
    R << cpsi * cth, -spsi * cphi + cpsi * sth * sphi, spsi * sphi + cpsi * cphi * sth,
         spsi * cth, cpsi * cphi + sphi * sth * spsi, -cpsi * sphi + sth * spsi * cphi,
         -sth, cth * sphi, cth * cphi;
    return R;
}

// Computes the Euler angle attitude transformation matrix T using the zyx convention
Matrix3d Tzyx(double phi, double theta) {
    double cphi = cos(phi);
    double sphi = sin(phi);
    double cth = cos(theta);
    double sth = sin(theta);

    if (fabs(cth) < 1e-8) {
        throw std::runtime_error("Tzyx is singular for theta = +-90 degrees.");
    }

    Matrix3d T;
    T << 1, sphi * sth / cth, cphi * sth / cth,
         0, cphi, -sphi,
         0, sphi / cth, cphi / cth;
    return T;
}

// Computes the Euler angle transformation matrix
MatrixXd eulerang(double phi, double theta, double psi) {

    /*
    J = [ Rzyx     0
            0    Tzyx ]
    */
    
    Matrix3d J1 = Rzyx(phi, theta, psi);
    Matrix3d J2 = Tzyx(phi, theta);

    MatrixXd J(6, 6);
    J << J1, Matrix3d::Zero(),
         Matrix3d::Zero(), J2;
    return J;
}

// Computes the Coriolis and centripetal matrix C(nu) from the mass matrix M and generalized velocity vector nu
MatrixXd m2c(const MatrixXd& M, const VectorXd& nu) {

    /*
    If M is a 6x6 matrix and nu = [u, v, w, p, q, r]', the output is a 6x6 C matrix
    If M is a 3x3 matrix and nu = [u, v, r]', the output is a 3x3 C matrix.
    */    

    MatrixXd M_sym = 0.5 * (M + M.transpose()); // systematization of the inertia matrix

    MatrixXd C;
    if (nu.size() == 6) {   // 6-DoF model
        Matrix3d M11 = M_sym.block<3, 3>(0, 0);
        Matrix3d M12 = M_sym.block<3, 3>(0, 3);
        Matrix3d M21 = M12.transpose();
        Matrix3d M22 = M_sym.block<3, 3>(3, 3);

        Vector3d nu1 = nu.segment<3>(0);
        Vector3d nu2 = nu.segment<3>(3);
        Vector3d dt_dnu1 = M11 * nu1 + M12 * nu2;
        Vector3d dt_dnu2 = M21 * nu1 + M22 * nu2;

        // C  = [  zeros(3,3)      -Smtrx(dt_dnu1)
        //       -Smtrx(dt_dnu1)  -Smtrx(dt_dnu2) ]
        C = MatrixXd::Zero(6, 6);
        C.block<3, 3>(0, 3) = -Smtrx(dt_dnu1);
        C.block<3, 3>(3, 0) = -Smtrx(dt_dnu1);
        C.block<3, 3>(3, 3) = -Smtrx(dt_dnu2);
    } 
    
    else {                  // 3-DoF model (surge, sway and yaw)
        
        /*
        C = [ 0             0            -M(1,1)*nu(1)-M(1,2)*nu(2)
              0             0             M(0,0)*nu(0)
              M(1,1)*nu(1)+M(1,2)*nu(2)  -M(0,0)*nu(0)          0  ]  
        */

        C = MatrixXd::Zero(3, 3);
        C(0, 2) = -M_sym(1, 1) * nu(1) - M_sym(1, 2) * nu(2);
        C(1, 2) = M_sym(0, 0) * nu(0);
        C(2, 0) = -C(0, 2);
        C(2, 1) = -C(1, 2);
    }

    return C;
}

// Interpolates a value y for a given x using linear interpolation
double interpolate(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
    auto it = std::lower_bound(xs.begin(), xs.end(), x);
    if (it == xs.begin()) {
        return ys.front();
    }
    if (it == xs.end()) {
        return ys.back();
    }
    size_t idx = std::distance(xs.begin(), it);
    double x1 = xs[idx - 1];
    double x2 = xs[idx];
    double y1 = ys[idx - 1];
    double y2 = ys[idx];
    return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
}

// Computes the 2D Hoerner cross-flow form coefficient as a function of beam B and draft T
double Hoerner(double B, double T) {
    std::vector<double> DATA1 = {
        0.0109, 0.1766, 0.3530, 0.4519, 0.4728, 0.4929, 0.4933, 0.5585, 0.6464, 0.8336,
        0.9880, 1.3081, 1.6392, 1.8600, 2.3129, 2.6000, 3.0088, 3.4508, 3.7379, 4.0031
    };
    std::vector<double> DATA2 = {
        1.9661, 1.9657, 1.8976, 1.7872, 1.5837, 1.2786, 1.2108, 1.0836, 0.9986, 0.8796,
        0.8284, 0.7599, 0.6914, 0.6571, 0.6307, 0.5962, 0.5868, 0.5859, 0.5599, 0.5593
    };

    double ratio = B / (2 * T);
    return interpolate(ratio, DATA1, DATA2);
}

// Computes the cross-flow drag integrals for a marine craft using strip theory
VectorXd crossFlowDrag(double L, double B, double T, const VectorXd& nu_r) {

    /*    
    tau_crossflow = crossFlowDrag(L,B,T,nu_r)
    M d/dt nu_r + C(nu_r)*nu_r + D*nu_r + g(eta) = tau + tau_crossflow
    Inputs: L:  length
            B:  beam
            T:  draft 
            nu_r = [u-u_c, v-v_c, w-w_c, p, q, r].T: relative velocity vector

    Output: tau_crossflow = [0 Yh 0 0 0 Nh]:  cross-flow drag in sway and yaw
    */

    double rho = 1026;          // density of seawater
    int n = 20;                 // number of strips
    double dx = L / n;
    double Cd_2D = Hoerner(B, T);   // 2D drag coefficient based on Hoerner's curve

    double Yh = 0;
    double Zh = 0;
    double Mh = 0;
    double Nh = 0;

    for (int i = 0; i <= n; ++i) {
        double xL = -L / 2 + i * dx;
        double v_r = nu_r(1);       // relative sway velocity
        double w_r = nu_r(2);       // relative heave velocity
        double r = nu_r(5);         // yaw rate

        double U_h = std::abs(v_r + xL * r) * (v_r + xL * r);
        double U_v = std::abs(w_r + xL * r) * (w_r + xL * r);

        Yh -= 0.5 * rho * T * Cd_2D * U_h * dx;         // sway force
        Zh -= 0.5 * rho * T * Cd_2D * U_v * dx;         // heave force
        Mh -= 0.5 * rho * T * Cd_2D * xL * U_v * dx;    // pitch moment
        Nh -= 0.5 * rho * T * Cd_2D * xL * U_h * dx;    // yaw moment
    }

    VectorXd tau_crossflow(6);
    tau_crossflow << 0, Yh, Zh, 0, Mh, Nh;
    return tau_crossflow;
}

// Approximates the added mass in surge using Söding's (1982) formula
std::pair<double, double> addedMassSurge(double m, double L, double rho) {
    
    /*
    Approximates the added mass in surge using Söding's (1982) formula:
        A11 = -Xudot = 2.7 * rho * nabla^(5/3) / L^2

    Inputs:
        m (float): ship/vehicle mass (kg)
        L (float): ship/vehicle length (m)
        rho (float, optional): density of water (kg/m3). Defaults to 1025.

    Outputs:
        A11: added mass in surge due to linear acceleration in surge (kg)
        ratio: mass ratio A11/m (typical values < 0.20)

    Reference: 
        H. Söding (1982). Prediction of Ship Steering Capabilities.
        Schiffstechnik, 3-29.
    */

    double nabla = m / rho;
    double A11 = 2.7 * rho * std::pow(nabla, 5.0/3.0) / (L * L);
    double ratio = A11 / m;
    return {A11, ratio};
}

// int main() {
//     // Example usage
//     double x = 5.0;
//     double x_min = 1.0;
//     double x_max = 10.0;
//     std::cout << "Saturated value: " << sat(x, x_min, x_max) << std::endl;

//     Vector3d a(1.0, 2.0, 3.0);
//     std::cout << "Skew-symmetric matrix:\n" << Smtrx(a) << std::endl;

//     Vector3d r(1.0, 2.0, 3.0);
//     std::cout << "System transformation matrix:\n" << Hmtrx(r) << std::endl;

//     double phi = 0.1, theta = 0.2, psi = 0.3;
//     std::cout << "Euler angle rotation matrix:\n" << Rzyx(phi, theta, psi) << std::endl;
//     std::cout << "Euler angle attitude transformation matrix:\n" << Tzyx(phi, theta) << std::endl;
//     std::cout << "Euler angle transformation matrix:\n" << eulerang(phi, theta, psi) << std::endl;

//     MatrixXd M = MatrixXd::Random(6, 6);
//     VectorXd nu = VectorXd::Random(6);
//     std::cout << "Coriolis and centripetal matrix:\n" << m2c(M, nu) << std::endl;

//     double L = 10.0, B = 2.0, T = 1.0;
//     VectorXd nu_r(6);
//     nu_r << 1.0, 2.0, 3.0, 0.1, 0.2, 0.3;
//     std::cout << "Cross-flow drag:\n" << crossFlowDrag(L, B, T, nu_r) << std::endl;

//     double m = 1000.0;
//     auto [A11, ratio] = addedMassSurge(m, L);
//     std::cout << "Added mass in surge: " << A11 << ", mass ratio: " << ratio << std::endl;

//     return 0;
// }

