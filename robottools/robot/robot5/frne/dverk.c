#include	<stdio.h>
#include	<math.h>
#include	"dverk.h"

#define	N_TEMPS	10

/*
 * differential equation solver
 * fnct(x, y, yprime)
 *		is a function pointer to a function leaving an integer
 * 		it should evaluate yprime(0),..., yprime(n-1) given x and 
 *		y(0),...,y(n-1), where yprime is the first derivative of y 
 *		with respect to x.
 * x		independent variable
 * xend 	value of x at which a solution is required
 * y		dependent variable, holds approximate solution to y on output
 * tol		norm of global error is proportional to tol
 * n		dimension of system
 */

struct com_link	c;

dverk (x, xend, y, tol, fnct, n)
double	x, xend, y[], tol;
int	(*fnct)(), n;
{
	double		*k1, *k2, *k3, *k4, *k5, *k6, *k7, *k8; 
	double		*err, *ytrial;
	double		temp, temp1;
	register int	i;

	/*
	 * malloc out memory for intermediate variables
	 * call calloc once for speed
	 */
	if ((k1 = (double *)calloc (N_TEMPS * n, sizeof(double))) == NULL)
		return (MEM_ERR);
	k2 = k1 + n; k3 = k2 + n; k4 = k3 + n; k5 = k4 + n;
	k6 = k5 + n; k7 = k6 + n; k8 = k7 + n;
	err = k8 + n; ytrial = err + n;

	do {
		(*fnct)(x, y, k1);
		c.fns++;
		for (i = 0; i < n; i++)
			temp = MAX(1.0, fabs(y[i]));
		c.wgt_norm = MIN(temp, 1.0);
		temp1 = MAX((c.wgt_norm / tol), fabs (x));
		c.hmin = 10.0* MAX(DWARF, (RREB * temp1));
		c.scale = 1;
		c.hmax = 2.0;		/* default value */
		if (c.hmin > c.hmax) {
			cfree ((char *)k1);
			return (INITIAL_ERR);
		}
		c.hstart = c.hmax * pow (tol, SIXTH);
		if (c.fns <= 1) {
			temp = 2.0 * c.hstart;
			if (tol < C6 * c.est) {
				temp = c.hstart * 0.9 * pow (tol /c.est, SIXTH);
				c.hstart = MAX(temp, (0.5*c.hstart));
			}
		}
		else
			c.hstart = 0.5 * c.hstart;
		c.hstart = MIN(c.hstart, c.hmax);
		c.hstart = MAX(c.hstart, c.hmin);
		if ( c.hstart < fabs (xend - x)) {
			c.hstart = MIN(c.hstart, (0.5 * fabs (xend - x)));
			c.xtrial = x + SIGN(c.hstart, xend - x);
		}
		else {
			c.hstart = fabs (xend - x);
			c.xtrial = xend;
		}
		c.htrial = c.xtrial - x;

		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial* k1[i] * R1;
		(*fnct)(x + c.htrial*C1, ytrial, k2);
		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial * (k1[i]*R2 + k2[i]*R3);
		(*fnct)(x + c.htrial*C2, ytrial, k3);
		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial * (k1[i]*R4 + k2[i]*R5 +
								k3[i]*R6);
		(*fnct)(x + c.htrial*C3, ytrial, k4);
		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial * (k1[i]*R7 + k2[i]*R8 +
						k3[i]*R9 + k4[i]*R10);
		(*fnct)(x + c.htrial*C4, ytrial, k5);
		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial * (k1[i]*R11 + k2[i]*R12 +
					k3[i]*R13 + k4[i]*R14 + k5[i]*R15);
		(*fnct)(x + c.htrial, ytrial, k6);
		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial * (k1[i]*R16 + k2[i]*R17 +
					k3[i]*R18 + k4[i]*R19 + k5[i]*R20);
		(*fnct)(x + c.htrial*C5, ytrial, k7);
		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial * (k1[i]*R21 + k2[i]*R22 +
				k3[i]*R23 + k4[i]*R24 + k5[i]*R25 + k7[i]*R26);
		(*fnct)(x + c.htrial, ytrial, k8);
		for (i = 0; i < n; i++)
			ytrial[i] = y[i] + c.htrial * (k1[i]*R27 + k3[i]*R28 +
				k4[i]*R29 + k5[i]*R30 + k7[i]*R31 + k8[i]*R32);
		c.fns += 7;
		if (c.fns > MAX_FN) {
			cfree ((char *)k1);
			return (TOO_MANY_FN_CALLS_ERR);
		}
		/*
		 * error estimate
		 */
		for (i = 0; i < n; i++)
			err[i] = k1[i]*R33 + k3[i]*R34 + k4[i]*R35 + k5[i]*R36 +
					k6[i]*R37 + k7[i]*R38 + k8[i]*R39;
		temp = 0.0;
		for (i = 0; i < n; i++) {
			temp1 = MAX(1.0, fabs (y[i]));
			temp = MAX(temp, (fabs (err[i]) / temp1));
		}
		c.est = temp * c.hmag * c.scale;
		if (c.est <= tol) {
			x =  c.xtrial;
			for (i = 0; i < n; i++)
				y[i] = ytrial[i];
			c.succ++;
			c.fails = 0;
			if (x == xend) {
				cfree ((char *)k1);
				return (c.fns);
			}
		}
		else {
			c.fails++;
			if (c.hmag <= c.hmin) {
				cfree ((char *)k1);
				return (STEP_SIZE_TOO_SMALL_ERR);
			}
		}
	} while (TRUE);
}

dv_status ()
{
	fprintf (stderr, "%d function calls made\n", c.fns);
	fprintf (stderr, "%d successful steps\n", c.succ);
	fprintf (stderr, "%d successive failures\n", c.fails);
	fprintf (stderr, "current scale is %d\n", c.scale);
	fprintf (stderr, "%g weighted norm\n", c.wgt_norm);
	fprintf (stderr, "minimum step size %g\n", c.hmin);
	fprintf (stderr, "maximum step size %g\n", c.hmax);
	fprintf (stderr, "current step size %g\n", c.hmag);
	fprintf (stderr, "start step size %g\n", c.hstart);
	fprintf (stderr, "trial step size %g\n", c.htrial);
}
