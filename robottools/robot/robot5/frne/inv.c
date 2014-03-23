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
#include	"dynlib.h"
#define	Reg	register

inv (a, ainv, n)
double	a[N][N], ainv[N][N];
{
	double	unit[N];
	Reg int	i, j, r;

	if ((r = decompose (a, n)) != 0)
		return (1);
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++)
			unit[j] = 0.0;
		unit[i] = 1.0;
		solve (a, ainv[i], unit, n);
	}
	return (0);
}

/*
 * In place decomposition of a matrix A into lower triangular form L,
 * where LU = A and L = U^t.
 */

decompose (a, n)
double	a[N][N];
{
	Reg int	i, j, k;

	for (i = 0; i < n; i++) {
		for (j = 0; j <= i; j++)
			if (i == j) {
				if (i)
					for (k = 0; k < i; k++)
						a[i][i] -= a[i][k]*a[i][k];
				if (a[i][i] <= 0.0)
					return(1);	/* not +ve definite */
				a[i][i] = sqrt (a[i][i]);
			}
			else {
				for (k = 0; k < j; k++)
					a[i][j] -= a[i][k]*a[j][k];
				a[i][j] /= a[j][j];
			}
	}
		/*
		 * fix up the symmetric part of the matrix 
		 */
	for (i = 0; i < n; i++) {
		if (i < n-1)
			for (j = i+1; j < n; j++)
				a[i][j] = a[j][i];
	}
	return (0);
}

/*
 * Solve the equation LUx = b, by Cholsky's method p808, E.Kreyszig
 */

solve (a, x, b, n)
double a[N][N], x[N], b[N];
{
	Reg int	i, j;

	/*
	 * solve Ly = b
	 */

	for (i = 0; i < n; i++) {
		if (i)
			for (j = 0; j < i; j++)
				b[i] -= a[i][j] * x[j];
		x[i] = b[i] / a[i][i];
	}
	/*
	 * solve Ux = y
	 */

	for (i = n-1; i >= 0; i--) {
		if (i != n-1)
			for (j = n-1; j > i; j--)
				x[i] -= a[i][j] * x[j];
		x[i] /= a[i][i];
	}
}
