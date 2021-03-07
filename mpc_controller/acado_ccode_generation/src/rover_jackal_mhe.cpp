#include <acado_code_generation.hpp>
#include <ros/ros.h>
#include <ros/package.h>

namespace params
{
const double alpha_max = 1.0;
const double x_icr = 0.214;
const double y_icr_l = 0.215;
const double y_icr_r = -0.215;

const double v_min = -0.6;
const double v_max = 0.6;
const double a_min = -2.0;
const double a_max = 2.0;
}  // namespace params

int main()
{
    USING_NAMESPACE_ACADO;
    using namespace params;

    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
    DifferentialState x;        // position in x-direction
    DifferentialState y;        // position in y-direction
    DifferentialState psi;      // robot heading
    DifferentialState v_l;      // linear velocity of left wheel
    DifferentialState v_r;      // linear velocity of right wheel
    DifferentialState alpha_l;  // slip parameter left wheel
    DifferentialState alpha_r;  // slip parameter right wheel

    Control a_l;  // linear acceleration of left wheel
    Control a_r;  // linear acceleration of right wheel

    IntermediateState vx = (alpha_l * y_icr_r * v_l - alpha_r * y_icr_l * v_r) / (y_icr_r - y_icr_l);
    IntermediateState w = (alpha_l * v_l - alpha_r * v_r) / (y_icr_r - y_icr_l);

    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
    DifferentialEquation f;
    f << dot(x) == vx * cos(psi) + x_icr * sin(psi) * w;
    f << dot(y) == vx * sin(psi) - x_icr * cos(psi) * w;
    f << dot(psi) == w;
    f << dot(v_l) == a_l;
    f << dot(v_r) == a_r;
    f << dot(alpha_l) == 0;
    f << dot(alpha_r) == 0;

    // Reference functions and weighting matrices:
    Function h, hN;
    h << x;
    h << y;
    h << psi;
    h << v_l;
    h << v_r;
    h << a_l;
    h << a_r;

    hN << x;
    hN << y;
    hN << psi;
    hN << v_l;
    hN << v_r;

    BMatrix W = eye<bool>(h.getDim());
    BMatrix WN = eye<bool>(hN.getDim());

    //
    // Optimal Control Problem
    //
    int N = 40;        // horizon step length
    double Ts = 0.01;  // prediction horizon steps
    OCP ocp(0.0, N * Ts, N);

    ocp.subjectTo(f);

    ocp.minimizeLSQ(W, h);
    ocp.minimizeLSQEndTerm(WN, hN);

    ocp.subjectTo(0.1 <= alpha_l <= alpha_max);
    ocp.subjectTo(0.1 <= alpha_r <= alpha_max);

    // Export the code:
    OCPexport mhe(ocp);

    mhe.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
    mhe.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
    mhe.set(INTEGRATOR_TYPE, INT_RK4);
    mhe.set(NUM_INTEGRATOR_STEPS, 2 * N);

    // NOTE: Those three options define MHE configuration!
    mhe.set(FIX_INITIAL_STATE, NO);
    mhe.set(SPARSE_QP_SOLUTION, CONDENSING);
    mhe.set(QP_SOLVER, QP_QPOASES);
    mhe.set(HOTSTART_QP, YES);

    mhe.set(CG_HARDCODE_CONSTRAINT_VALUES, YES);  // Possible to Change Constraints Afterwards (only with qpOASES)
                                                  //    mhe.set( LEVENBERG_MARQUARDT, 1.0e-4 );

    mhe.set(GENERATE_TEST_FILE, NO);
    mhe.set(GENERATE_MAKE_FILE, NO);
    mhe.set(GENERATE_MATLAB_INTERFACE, NO);
    mhe.set(GENERATE_SIMULINK_INTERFACE, NO);
    mhe.set(CG_USE_ARRIVAL_COST, YES);

    mhe.set(PRINTLEVEL, DEBUG);

    // Optionally set custom module name:
    mhe.set(CG_MODULE_NAME, "nmhe");
    mhe.set(CG_MODULE_PREFIX, "NMHE");

    std::string path = ros::package::getPath("acado_ccode_generation");
    std::string path_dir = path + "/solver/NMHE_rover";
    ROS_INFO("%s", path_dir.c_str());

    try
    {
        ROS_WARN("TRYING TO EXPORT");
        if (mhe.exportCode(path_dir) != SUCCESSFUL_RETURN)
            ROS_ERROR("FAIL EXPORT CODE");
    }
    catch (...)
    {
        ROS_ERROR("FAIL TO EXPORT");
    }

    mhe.printDimensionsQP();

    ROS_WARN("DONE CCODE");

    return EXIT_SUCCESS;
}
