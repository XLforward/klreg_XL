/*
 *	Written by;
 *
 *		Peter I. Corke
 *		CSIRO Division of Manufacturing Technology
 *		Preston, Melbourne.  Australia. 3072.
 *
 *		pic@mlb.dmt.csiro.au
 *
 *  Permission to use and distribute is granted, provided that this message
 * is retained, and due credit given when the results are incorporated in
 * publised work.
 *
 */
/*
 * Main body function to simulate dynamics over a timestep.
 *
 *  This can be modified to use your favorite numerical integration package.
 * By default it uses a Runge-Kutta routine dverk() which is provided.  I've
 * had some problems with this, and find the function odeint() from 
 *	`Numerical Recipes in C`
 *	Press, Flannery, Teukolsky, Vetterling
 *	Cambridge University Press, 1988
 *
 * is quicker and more stable.  NOTE that we have modified odeint() to use
 * all doubles rather than floats.
 *
 *  If NUM_RECIPES is defined (by the Makefile) then odeint() will be used
 * instead of dverk().
 *	
 */
#include	"dynlib.h"

#ifdef	NUM_RECIPES
#include	"nr.h"
#include	"nrutil.h"
#endif

#ifdef	NUM_RECIPES
#define	ARRAY_START	1
#else
#define	ARRAY_START	0
#endif

#define	TOL	1.0e-4

int	dyn_verbose;	/* verbosity flag for this package */

static Manipulator	*ps;
static double		*tau_act;
static Load		*load;

dyn_integrate(p, tau, l, s_time, step, flg, t_func)
Manipulator	*p;
double		*tau;
Load		*l;
double		s_time,
		step;
int		(*t_func)(),
		flg;
{
	double		*y, tol = TOL;
	register int	i, k;
	int		r;
	static Load	noload = {0};	/* no load */
	int		system();
	void		rkqc(), bsstep();
	int		nok, nbad;

	/*
	 * check input data for consistency
	 */
	if (l == NULL)
		l = &noload;
	if (p == NULL) {
		fprintf (stderr, "puma_dyn: null manipulator pointer\n");
		return (0);
	}

	/* kludge to pass this to the integrated function */
	ps = p;
	load = l;
	tau_act = tau;

#ifdef	NUM_RECIPES
	y = dvector(1, p->naxes*2);
#else
	y = (double *)calloc(p->naxes*2, sizeof(double));
#endif

        /*
         * start integration of system
         * set up initial conditions for position, velocity
         */
        for (i = 0, k = ARRAY_START; i < p->naxes; i++) {
                y[k++] = Q(p)[i];
                y[k++] = QD(p)[i];
        }

#ifndef	NUM_RECIPES
	if ((r = dverk (s_time, s_time + step, y, tol, system, p->naxes*2)) < 0)
		return (-1);
#else
	odeint(y, p->naxes*2, s_time, s_time + step, tol, step/2.0,
		0.0, &nok, &nbad, system, bsstep);
#endif

	/*
	 * update joint state variables with new values 
	 */
	for (i = 0, k = ARRAY_START; i < p->naxes; i++) {
		Q(p)[i] = y[k++];
		QD(p)[i] = y[k++];
	}

#ifdef	NUM_RECIPES
	free_dvector(y, 1, p->naxes*2);
#else
	free(y);
#endif
	return (r);
}

/*
 * Called by the numerical integrator dverk().
 *
 * put equations into a form that the integration routine, dverk expects.
 *
 * The state vector is  [q1 q1' q2 q2' q3 q3' ...]
 */
static
system (t, y, yd)
double	t, *y, *yd;
{
	register int	i,j;

	/*
	 * Copy state from input vector to manip struct
	 */
	for (i = 0, j = ARRAY_START; i < ps->naxes; i++) {
		Q(ps)[i] = y[j++];
		QD(ps)[i] = y[j++];
	}

	dyn_accel(ps, tau_act, load);

	/*
	 * Copy state' from manip struct to y'
	 */
	for (i = 0, j = ARRAY_START; i < ps->naxes; i++) {
		yd[j++] = QD(ps)[i];
		yd[j++] = QDD(ps)[i];
	}
}
