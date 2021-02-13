#include <acado_code_generation.hpp>
#include <ros/ros.h>
#include <ros/package.h>

namespace params
{
const double alpha_l = 1.0;  // slip parameter left wheel
const double alpha_r = 1.0;  // slip parameter right wheel
const double x_icr = 0.0;
const double y_icr_l = 0.215;
const double y_icr_r = -0.215;

const double a_min = -5.0;
const double a_max = 5.0;
}  // namespace params

int main()
{
    USING_NAMESPACE_ACADO;
    using namespace params;

    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
    DifferentialState x;    // position in x-direction
    DifferentialState y;    // position in y-direction
    DifferentialState psi;  // robot orientation
    DifferentialState w;    // robot orientation rate
    OnlineData v_l;         // linear velocity of left wheel
    OnlineData v_r;         // linear velocity of right wheel

    Control a_l;  // linear acceleration of left wheel
    Control a_r;  // linear acceleration of right wheel

    IntermediateState vx = (alpha_l * y_icr_r * v_l - alpha_r * y_icr_l * v_r) / (y_icr_r - y_icr_l);

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    DifferentialEquation f;
    f << dot(x) == vx * cos(psi) + x_icr * sin(psi) * w;
    f << dot(y) == vx * sin(psi) - x_icr * cos(psi) * w;
    f << dot(psi) == w;
    f << dot(w) == (alpha_l * a_l - alpha_r * a_r) / (y_icr_r - y_icr_l);

    // Reference functions and weighting matrices:
    Function h, hN;
    h << x;
    h << y;
    h << psi;
    h << w;
    h << a_l;
    h << a_r;

    hN << x;
    hN << y;
    hN << psi;
    hN << w;

    BMatrix W = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());

    //
    // Optimal Control Problem
    //
    int N = 30;        // horizon step length
    double Ts = 0.01;  // prediction horizon steps
    OCP ocp(0.0, N * Ts, N);

    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    //    ocp.subjectTo(a_min <= a_l <= a_max);
    //    ocp.subjectTo(a_min <= a_r <= a_max);
    //ocp.subjectTo( dot(x)*cos(psi)-dot(y)*cos(psi)== 0);

    // Export the code:
    OCPexport mpc(ocp);

    mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mpc.set(INTEGRATOR_TYPE, INT_RK4);
    mpc.set(NUM_INTEGRATOR_STEPS, 2 * N);

    mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
    mpc.set(QP_SOLVER, QP_QPOASES);

    // 	mpc.set( SPARSE_QP_SOLUTION, SPARSE_SOLVER );
    // 	mpc.set( QP_SOLVER, QP_QPDUNES );

    mpc.set(HOTSTART_QP, YES);

    mpc.set(CG_HARDCODE_CONSTRAINT_VALUES, YES);  // Possible to Change Constraints Afterwards (only with qpOASES)

    mpc.set(GENERATE_TEST_FILE, NO);
    mpc.set(GENERATE_MAKE_FILE, NO);
    mpc.set(GENERATE_MATLAB_INTERFACE, NO);
    mpc.set(GENERATE_SIMULINK_INTERFACE, NO);

    // Optionally set custom module name:
    mpc.set(CG_MODULE_NAME, "nmpc");
    mpc.set(CG_MODULE_PREFIX, "NMPC");

    std::string path = ros::package::getPath("acado_ccode_generation");
    std::string path_dir = path + "/solver/NMPC_rover";
    ROS_INFO("%s", path_dir.c_str());

    try
    {
        ROS_WARN("TRYING TO EXPORT");
        if (mpc.exportCode(path_dir) != SUCCESSFUL_RETURN)
            ROS_ERROR("FAIL EXPORT CODE");
    }
    catch (...)
    {
        ROS_ERROR("FAIL TO EXPORT");
    }

    mpc.printDimensionsQP();

    ROS_WARN("DONE CCODE");

    return EXIT_SUCCESS;
}
