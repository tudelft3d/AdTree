#include "optimizer_lm.h"
#include "cminpack.h"


Optimizer_LM::Optimizer_LM() {
	// Initialize the control: termination condition, steps size..
	lm_initialize_control();
}

void Optimizer_LM::lm_initialize_control()
{
	default_control_.maxcall = 10000;
	default_control_.epsilon = 1.e-14;
	default_control_.stepbound = 100.;
	default_control_.ftol = 1.e-14;
	default_control_.xtol = 1.e-14;
	default_control_.gtol = 1.e-14;
	default_control_.nprint = 0;
}

bool Optimizer_LM::optimize(int num_fun, int num_var, double* par, lm_evaluate_func* func, void* data, lm_parameters* control)
{
	// use default parameter if ctrl == 0
	if (!control)
		control = &default_control_;

	// *** allocate work space.

	double *fvec, *diag, *fjac, *qtf, *wa1, *wa2, *wa3, *wa4;
	int *ipvt;

	int m = num_fun;
	int n = num_var;

	if (
		!(fvec = new double[m]) ||
		!(diag = new double[n]) ||
		!(qtf = new double[n]) ||
		!(fjac = new double[n*m]) ||
		!(wa1 = new double[n]) ||
		!(wa2 = new double[n]) ||
		!(wa3 = new double[n]) ||
		!(wa4 = new double[m]) ||
		!(ipvt = new int[n]))
	{
		control->info = 9;
		return false;
	}

	// *** perform fit.

	control->info = 0;
	control->nfev = 0;
	// this goes through the modified legacy interface:
	control->info =
		lmdif(
		func,
		data,
		m,
		n,
		par,
		fvec,
		control->ftol,
		control->xtol,
		control->gtol,
		control->maxcall*(n + 1),
		control->epsilon,
		diag,
		1,
		control->stepbound,
		control->nprint,
		&(control->nfev),
		fjac,
		m,
		ipvt,
		qtf,
		wa1,
		wa2,
		wa3,
		wa4
		);

	if (control->info >= 8)
		control->info = 4;

	// *** clean up.

	delete[](fvec);
	delete[](diag);
	delete[](qtf);
	delete[](fjac);
	delete[](wa1);
	delete[](wa2);
	delete[](wa3);
	delete[](wa4);
	delete[](ipvt);

	return true;
}