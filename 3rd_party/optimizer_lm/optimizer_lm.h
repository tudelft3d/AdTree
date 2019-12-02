#ifndef _MATH_OPTIMIZER_LM_H_
#define _MATH_OPTIMIZER_LM_H_


/**
Optimizer_LM for nonlinear least squares problems using Levenberg-Marquardt method.
It wraps the lmdif() part of cminpack (see http://devernay.free.fr/hacks/cminpack/index.html)

min Sum_{i=0}^{M} ( F(x0,..,xN)_i )^2 )
Where: 
	F: R^n->R^m is a user defined function

****** basic examples *******

1) minimization of a function (other values can be provided via "data")

void evaluate(void* data, int num_fun, int num_var, double* par, double* fvec, int iflag)
{
	for(int i=0 ; i<num_fun; ++i)
		fvec[i] = i * par[0] + (i/2.0) * par[1] * par[1];
}

int main(){
	int m = 3;
	int n = 2;

	Optimizer_LM lm;
	double par[2] = {4.0, -4.0};
	lm.run(3, 2, par, (Optimizer_LM::lm_evaluate_func*)evaluate, 0);

	std::cout << "the solution is: " << par[0] << "  " << par[1] << std::endl;
	// the results are: -12.3333 -4.96655
}

*********************************

2) for fitting problems, you can provide the "user data"

 // fit the function: f(x1, x2) = exp(a*x1 + b*x2)
 // objective function to be minimized: Sum_i[ exp(a*x1 + b*x2) - f(x1, x2) ]
 
 void evaluate(double* data, int m, int n, double* var, double* fvec, int iflag)
 {
	 for(int i=0; i<m; ++i)
		 fvec[i] = std::exp(var[0] * data[i] + var[1] * data[i+m]) - data[i+m*2];
 }
 
 void main() {
 	int m = 3;
 	int n = 2;
 	double par[] = {1.0, 1.0};
 	double data[] = {
 		1.0, 2.0, 3.0,    // t0
 		2.0, 3.0, 4.0,    // t1
 		2.0, 4.0, 3.0     // y
 	};
 
 	Optimizer_LM lmo;
 	lmo.run(m, n, par, (Optimizer_LM::lm_evaluate_func*)evaluate, data);
 
	std::cout << "the solution is: " << par[0] << "  " << par[1] << std::endl;
	// the results are: -0.664837  0.807553
 }
*/


class Optimizer_LM
{
public:
	Optimizer_LM();

	// prototype of the evaluate function (Jacobian calculated by by finite differences)
	/* calculate the functions at x and */
	/* return this vector in fvec. */
	/* if iflag = 1 the result is used to compute the residuals. */
	/* if iflag = 2 the result is used to compute the Jacobian by finite differences. */
	/* Jacobian computation requires exactly n function calls with iflag = 2. */
	/* return a negative value to terminate lmdif1/lmdif */
	typedef int (lm_evaluate_func) (
		void *data,			// user data (the same passed to run)
		int num_fun,		// Number of functions
		int num_var,		// Number of variables where n<=m
		const double* var,	// the n parameters to compute your F
		double* fvec,		// the values computed by F
		int iflag			// status
		);

	// parameters for calling the high-level interface functions
	struct lm_parameters {
		double ftol; 		// relative error desired in the sum of squares.
		double xtol; 		// relative error between last two approximations.
		double gtol; 		// orthogonality desired between fvec and its derivs.
		double epsilon; 	// step used to calculate the Jacobian.
		double stepbound; 	// initial bound to steps in the outer loop.
		double fnorm; 		// norm of the residue vector fvec.
		int maxcall; 		// maximum number of iterations.
		int nfev; 			// actual number of iterations.
		int nprint; 		// desired frequency of print outs.
		int info; 			// status of minimization.
	} ;

public:

	// Jacobian calculated by by finite differences
	bool optimize(
		int num_fun,				// Number of functions
		int num_var,				// Number of variables where num_var<=num_fun
		double* par,				// starting values of the parameters, it also return the results
		lm_evaluate_func* func,		// your evaluate function (no need to provide Jacobian)
		void* data,					// user data (will be passed to func)
		lm_parameters* ctrl = 0		// use default parameter if ctrl == 0
	);

private:
	/// Initialize the control: termination condition, steps size..
	void lm_initialize_control();

private:
	lm_parameters default_control_;		// control of this object
};



#endif 