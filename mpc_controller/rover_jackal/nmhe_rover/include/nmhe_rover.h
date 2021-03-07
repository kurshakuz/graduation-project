#ifndef _NMHE_ROVER_H
#define _NMHE_ROVER_H

#include <math.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#include "nmhe_common.h"
#include "nmhe_auxiliary_functions.h"

extern NMHEworkspace nmheWorkspace;
extern NMHEvariables nmheVariables;

struct nmhe_struct_
{
	double sampleTime;
	bool verbose;

    int num_params;
	Eigen::VectorXd X0;
	Eigen::VectorXd W;
	Eigen::VectorXd WN;
	Eigen::VectorXd process_noise_cov;
	Eigen::VectorXd SAC;
	Eigen::VectorXd xAC;
};
class NMHE
{
	private:
		bool is_estimator_init, is_prediction_init;

		nmhe_struct_ nmhe_inp_struct;

		Eigen::MatrixXd WL_mat;

		int run_cnt;

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
			boost::function<int(int)> updateArrivalCost;

			int acado_N;
			int acado_NX;
			int acado_NY;
			int acado_NYN;
			int acado_NU;
			int acado_NOD;

			real_t * x;
			real_t * u;
			real_t * od;
			real_t * y;
			real_t * yN;
			real_t * W;
			real_t * WN;
			real_t * SAC;
			real_t * xAC;
			real_t * WL;
		} nmhe_struct;

		struct estimation_struct
		{
			std::vector<double> states_vec;
			std::vector<double> params_vec;
			double exe_time;
			double kkt_tol;
			double obj_val;

		} nmhe_est_struct;

		NMHE(struct nmhe_struct_ &_nmhe_inp_struct);
		~NMHE();

		bool return_estimator_init_value();

		void nmhe_init(struct acado_struct &acadostruct);
		
		void nmhe_core(struct acado_struct &acadostruct, struct estimation_struct &estimationstruct, std::vector<double> &statesmeas, std::vector<double> &nmpccmd);

		void publish_est(struct estimation_struct &estimationstruct);
		void publish_zeroest();

	protected:

		void set_measurements(struct acado_struct &acadostruct, std::vector<double> &statesmeas, std::vector<double> &nmpccmd);

};

#endif
