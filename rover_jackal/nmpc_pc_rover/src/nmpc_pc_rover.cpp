/**
 * @file   nmpc_pc_rover.cpp
 * @author Mohit Mehndiratta
 * @date   November 2020
 *
 * @copyright
 * Copyright (C) 2020.
 */

#include <nmpc_pc_rover.h>

using namespace Eigen;

NMPCworkspace nmpcWorkspace;
NMPCvariables nmpcVariables;

NMPC_PC::NMPC_PC(struct nmpc_struct_& _nmpc_inp_struct)
{
    is_control_init = false;

    nmpc_inp_struct = _nmpc_inp_struct;

    WN.resize(NMPC_NYN);
    for (int i = 0; i < NMPC_NYN; ++i)
        WN[i] = nmpc_inp_struct.W_Wn_factor * nmpc_inp_struct.W[i];

    // --------------------
    // ACADO NMPC CONTROLLER
    // --------------------

    nmpc_struct.initializeSolver = boost::bind(nmpc_initializeSolver);
    nmpc_struct.preparationStep = boost::bind(nmpc_preparationStep);
    nmpc_struct.feedbackStep = boost::bind(nmpc_feedbackStep);
    nmpc_struct.getKKT = boost::bind(nmpc_getKKT);
    nmpc_struct.getObjective = boost::bind(nmpc_getObjective);
    nmpc_struct.printDifferentialVariables = boost::bind(nmpc_printDifferentialVariables);
    nmpc_struct.printControlVariables = boost::bind(nmpc_printControlVariables);

    nmpc_struct.acado_N = NMPC_N;
    nmpc_struct.acado_NX = NMPC_NX;
    nmpc_struct.acado_NY = NMPC_NY;
    nmpc_struct.acado_NYN = NMPC_NYN;
    nmpc_struct.acado_NU = NMPC_NU;
    nmpc_struct.acado_NOD = NMPC_NOD;

    nmpc_struct.x0 = &nmpcVariables.x0[0];
    nmpc_struct.x = &nmpcVariables.x[0];
    nmpc_struct.y = &nmpcVariables.y[0];
    nmpc_struct.yN = &nmpcVariables.yN[0];
    nmpc_struct.u = &nmpcVariables.u[0];
    nmpc_struct.W = &nmpcVariables.W[0];
    nmpc_struct.WN = &nmpcVariables.WN[0];

    nmpc_cmd_struct.control_vel_vec.resize(NMPC_NU, 0.0);
    nmpc_cmd_struct.control_acc_vec.resize(NMPC_NU, 0.0);
    nmpc_cmd_struct.exe_time = 0.0;
    nmpc_cmd_struct.kkt_tol = 0.0;
    nmpc_cmd_struct.obj_val = 0.0;

    if (nmpc_inp_struct.verbose)
    {
        std::cout << "***********************************\n";
        std::cout << "Constructor of the class NMPC_PC is created\n";
        std::cout << "***********************************\n";
    }
}

NMPC_PC::~NMPC_PC()
{
    if (nmpc_inp_struct.verbose)
    {
        std::cout << "*********************************\n";
        std::cout << "Destructor of the class NMPC_PC\n";
        std::cout << "*********************************\n";
    }
}

bool NMPC_PC::return_control_init_value()
{
    return NMPC_PC::is_control_init;
}

void NMPC_PC::nmpc_init(struct acado_struct& acadostruct)
{
    if (nmpc_inp_struct.verbose)
    {
        std::cout << "***********************************\n";
        std::cout << "outer_nmpc_initController - start\n";
    }

    // Initialize the solver
    // ---------------------
    acadostruct.initializeSolver();

    // NMPC: initialize/set the states
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NX * (acadostruct.acado_N + 1); ++i)
    {
        // TODO: check if these have to be changed for some states?
        acadostruct.x[i] = 0.0;
    }

    // NMPC: initialize/set the controls
    // ---------------------
    for (int i = 0; i < acadostruct.acado_N; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NU; ++j)
            acadostruct.u[(i * acadostruct.acado_NU) + j] = nmpc_inp_struct.U_ref(j);
    }

    // NMPC: initialize the measurements/reference
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NY * acadostruct.acado_N; ++i)
    {
        acadostruct.y[i] = 0.0;
    }
    for (int i = 0; i < acadostruct.acado_NYN; ++i)
    {
        acadostruct.yN[i] = 0.0;
    }

// NMPC: initialize the current state feedback
// ---------------------
#if ACADO_INITIAL_STATE_FIXED
    for (int i = 0; i < acadostruct.acado_NX; ++i)
    {
        acadostruct.x0[i] = 0;
    }
#endif

    // NMPC: initialize the weight matrices
    // ---------------------
    for (int i = 0; i < acadostruct.acado_NY; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NY; ++j)
        {
            if (i == j)
                acadostruct.W[(i * acadostruct.acado_NY) + j] = nmpc_inp_struct.W[i];
            else
                acadostruct.W[(i * acadostruct.acado_NY) + j] = 0.0;
        }
    }
    //  std::cout<<"W_0 = "<<nmpc_inp_struct.W<<"\n";
    for (int i = 0; i < acadostruct.acado_NYN; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NYN; ++j)
        {
            if (i == j)
                acadostruct.WN[(i * acadostruct.acado_NYN) + j] = WN[i];
            else
                acadostruct.WN[(i * acadostruct.acado_NYN) + j] = 0.0;
        }
    }
    //  std::cout<<"WN_0 = "<<WN<<"\n";

    // Prepare first step
    // ------------------
    acadostruct.preparationStep();

    if (nmpc_inp_struct.verbose)
    {
        std::cout << "Outer NMPC: initialized correctly\n";
        std::cout << "***********************************\n";
    }
    is_control_init = true;
}

void NMPC_PC::nmpc_core(struct acado_struct& acadostruct,
                        struct command_struct& commandstruct,
                        std::vector<double>& reftrajectory,
                        std::vector<double>& statesmeas)
{
    // set the current state feedback
    set_measurements(acadostruct, statesmeas);

    // set the reference path
    set_reftrajectory(acadostruct, reftrajectory);

    // NMPC: calc and apply control and prepare optimization for the next step
    // ----------------------------------------------------------------------

    // Execute Calculation (Optimization)
    clock_t stopwatch;
    stopwatch = clock();
    acado_feedbackStep_fb = acadostruct.feedbackStep();

    if (nmpc_inp_struct.verbose && acado_feedbackStep_fb != 0)
    {
        std::cout << "ACADO ERROR: " << acado_feedbackStep_fb << "\n";
        std::cout << "acado outer nmpc controller states: x, y, z, ... = " << std::endl;
        for (int i = 0; i < acadostruct.acado_NX; ++i)
            std::cout << acadostruct.x0[i] << ", ";
        std::cout << std::endl;
    }

    // Rover controls
    for (int i = 0; i < acadostruct.acado_NU; ++i)
    {
        // v(t+1) = v(t) + sample_time*dot_v(t)
        // omega(t+1) = v(t+1)/wheel_radius
        commandstruct.control_vel_vec[i] =
            (statesmeas.at(4 + i) + nmpc_inp_struct.sampleTime * acadostruct.u[i]) / WHEEL_RADIUS_M;
        commandstruct.control_acc_vec[i] = acadostruct.u[i];
    }

    commandstruct.kkt_tol = acadostruct.getKKT();
    commandstruct.obj_val = acadostruct.getObjective();

    // Settings for the next iteration
    acadostruct.preparationStep();

    // Calculate the entire execution time!
    commandstruct.exe_time = ((double)(clock() - stopwatch)) / CLOCKS_PER_SEC;

    //    ROS_INFO_STREAM("Stoptime outer NMPC: " << ros::Time::now().toSec() - stopwatch.toSec() << " (sec)");

    /* ------ NMPC_DEBUG ------*/
    //  acadostruct.printDifferentialVariables();
    //  acadostruct.printControlVariables();
}

void NMPC_PC::set_measurements(struct acado_struct& acadostruct, std::vector<double>& statesmeas)
{
    for (int i = 0; i < statesmeas.size(); i++)
        acadostruct.x0[i] = statesmeas.at(i);
}

void NMPC_PC::set_reftrajectory(struct acado_struct& acadostruct, std::vector<double>& reftrajectory)
{
    // Refrences for states
    for (int i = 0; i < acadostruct.acado_NYN; i++)
        acadostruct.yN[i] = reftrajectory[i];

    for (int i = 0; i < acadostruct.acado_N; ++i)
    {
        for (int j = 0; j < acadostruct.acado_NY; ++j)
        {
            if (j < acadostruct.acado_NYN)
                acadostruct.y[(i * acadostruct.acado_NY) + j] = acadostruct.yN[j];
            else
                acadostruct.y[(i * acadostruct.acado_NY) + j] = nmpc_inp_struct.U_ref(j - acadostruct.acado_NYN);
        }
    }
}
