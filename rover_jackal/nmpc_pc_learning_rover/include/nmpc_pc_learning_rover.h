#ifndef _NMPC_PC_LEARNING_ROVER_H
#define _NMPC_PC_LEARNING_ROVER_H

#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#include "nmpc_common.h"
#include "nmpc_auxiliary_functions.h"

#define WHEEL_RADIUS_M 0.098
#define WHEEL_WIDTH_M 0.040
#define TRACK_WIDTH_M 0.37559

extern NMPCworkspace nmpcWorkspace;
extern NMPCvariables nmpcVariables;

struct nmpc_struct_
{
	double sampleTime;
	bool verbose;
	double W_Wn_factor;
	std::vector<double> init_params;

	Eigen::VectorXd U_ref;
	Eigen::VectorXd W;	
};
class NMPC_PC
{
	private:
		bool is_control_init;

		nmpc_struct_ nmpc_inp_struct;
		Eigen::VectorXd WN;

	public:	
		int acado_feedbackStep_fb;

		struct acado_struct
		{
			boost::function<int(void)> initializeSolver;
			boost::function<int(void)> preparationStep;
			boost::function<int(void)> feedbackStep;
			boost::function<real_t(void)> getKKT;
			boost::function<real_t(void)> getObjective;
			boost::function<void(void)> printDifferentialVariables;
			boost::function<void(void)> printControlVariables;

			int acado_N;
			int acado_NX;
			int acado_NY;
			int acado_NYN;
			int acado_NU;
			int acado_NOD;

			real_t * x0;
			real_t * u;
			real_t * x;
			real_t * od;
			real_t * y;
			real_t * yN;
			real_t * W;
			real_t * WN;
		} nmpc_struct;

		struct command_struct
		{
			std::vector<double> control_vel_vec;
			std::vector<double> control_acc_vec;
			double exe_time;
			double kkt_tol;
			double obj_val;

		} nmpc_cmd_struct;

		NMPC_PC(struct nmpc_struct_ &_nmpc_inp_struct);
		~NMPC_PC();

		bool return_control_init_value();

		void nmpc_init(struct acado_struct &acadostruct);
		
		void nmpc_core(struct acado_struct &acadostruct, struct command_struct &commandstruct, std::vector<double> &reftrajectory, std::vector<double> &params, std::vector<double> &statesmeas);

		void publish_cmd(struct command_struct &commandstruct);
		void publish_zerocmd();

	protected:

		void set_measurements(struct acado_struct &acadostruct, std::vector<double> &params, std::vector<double> &statesmeas);

		void set_reftrajectory(struct acado_struct &acadostruct, std::vector<double> &reftrajectory);

		void nan_check_for_estimates(std::vector<double> &params);
};

#endif
