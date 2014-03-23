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
 * Compute the inverse dynamics via the recursive Newton-Euler formulation
 *
 *	Requires:	Qdot	current joint velocities
 *			Qddot	current joint accelerations
 *			f	applied tip force or load
 *			grav	the gravitational constant
 *
 *	Returns:	tau	vector of bias torques
 */
#include	"dynlib.h"

static void print_matrix(char *s, Mat *m);
static void print_vector(char *s, double *v);

/*
 * bunch of macros to make the main code easier to read.  Dereference vectors
 * from the DynVar or Link structures for the manipulator.
 *
 * Note that they return pointers (except for M(i) which is a scalar)
 */
#undef	N

#define	OMEGA(i)	(&dv[i].omega)		/* ang.velocity */
#define	ALPHA(i)	(&dv[i].alpha)		/* ang.acceleration */
#define	ACC(i)		(&dv[i].acc)
#define	ABAR(i)		(&dv[i].abar)
#define	ROT(i)		(&dv[i].Rot)
#define	F(i)		(&dv[i].f)
#define	N(i)		(&dv[i].n)

#define	AMAT(i)		(&dv[i].A)

#define	M(i)		(l[i].m)	/* mass */
#define	PSTAR(i)		(&l[i].r)	/* offset link i from link (i-1) */
#define	RBAR(i)		(&l[i].rbar)	/* COG link i wrt link i */
#define	IMAT(i)		(&l[i].I)

dyn_newton_euler (m, tau, Qdot, Qddot, load, grav)
Manipulator	*m;
double		tau[];
Vect		Qdot[], Qddot[];
Load		*load;
Vect		*grav;
{
	Vect			t1, t2, t3, t4;
	static Vect		z0 = {0.0, 0.0, 1.0};
	static Vect		zero = {0.0, 0.0, 0.0};
	register int		i;
	register DynVars	*dv = m->dv;
	register Link		*l = m->link;
	double			t;

	/*
	 * start forward iteration for all joints
	 */
	for (i = 0; i < m->naxes; i++) {
		switch (m->link[i].axistype) {
		case REVOLUTE:
			/* 
			 * calculate omega[i]
			 */
			if (i == 0)
				t1 = Qdot[i];
			else
				vect_add (&t1, OMEGA(i-1), &Qdot[i]);
			matt_vect_mult (OMEGA(i), ROT(i), &t1);

			/*
			 * calculate alpha[i] 
			 */
			if (i > 0) {
				vect_add (&t1, ALPHA(i-1), &Qddot[i]);
				vect_cross (&t2, OMEGA(i-1), &Qdot[i]);
				vect_add (&t3, &t1, &t2);
			}
			else
				t3 = Qddot[i];
			matt_vect_mult (ALPHA(i), ROT(i), &t3);

			/*
			 * compute acc[i]
			 */
			vect_cross(&t1, ALPHA(i), PSTAR(i));
			vect_cross(&t2, OMEGA(i), PSTAR(i));
			vect_cross(&t3, OMEGA(i), &t2);
			vect_add(ACC(i), &t1, &t3);
			if (i > 0) {
				matt_vect_mult(&t1, ROT(i), ACC(i-1));
			}
			else {
				scal_mult(&t2, &z0, grav);
				matt_vect_mult(&t1, ROT(i), grav);
			}
			vect_add(ACC(i), ACC(i), &t1);

			break;

		case PRISMATIC:
			/* 
			 * calculate omega[i]
			 */
			if (i == 0)
				*(OMEGA(i)) = zero;
			else
				matt_vect_mult (OMEGA(i), ROT(i), OMEGA(i-1));

			/*
			 * calculate alpha[i] 
			 */
			if (i == 0)
				*(ALPHA(i)) = zero;
			else
				matt_vect_mult (ALPHA(i), ROT(i), ALPHA(i-1));

			/*
			 * compute acc[i]
			 */
			if (i > 0) {
				vect_add(&t1, &Qddot[i], ACC(i-1));
				matt_vect_mult(ACC(i), ROT(i), &t1);

			} else {
				matt_vect_mult(ACC(i), ROT(i), &Qddot[i]);
			}

			vect_cross(&t1, ALPHA(i), PSTAR(i));
			vect_add(ACC(i), ACC(i), &t1);

			matt_vect_mult(&t1, ROT(i), &Qdot[i]);
			vect_cross(&t2, OMEGA(i), &t1);
			scal_mult(&t2, &t2, 2.0);
			vect_add(ACC(i), ACC(i), &t2);

			vect_cross(&t2, OMEGA(i), PSTAR(i));
			vect_cross(&t3, OMEGA(i), &t2);
			vect_add(ACC(i), ACC(i), &t3);

			break;
		}
		/*
		 * compute abar[i]
		 */
		vect_cross(&t1, ALPHA(i), RBAR(i));
		vect_cross(&t2, OMEGA(i), RBAR(i));
		vect_cross(&t3, OMEGA(i), &t2);
		vect_add(ABAR(i), &t1, &t3);
		vect_add(ABAR(i), ABAR(i), ACC(i));
	}
	/*
	 * backward recursion part --the kinetics
	 */
	for (i = m->naxes - 1; i >= 0; i--) {

		/*
		 * compute f[i]
		 */
		scal_mult (F(i), ABAR(i), M(i));
		t4 = *F(i);
		if (i != (m->naxes-1)) {
			mat_vect_mult (&t1, ROT(i+1), F(i+1));
			vect_add (F(i), F(i), &t1);
		}

		 /*
		  * compute n[i]
		  */
		vect_add(&t2, PSTAR(i), RBAR(i));
		vect_cross(&t1, &t2, &t4);

		if (i != (m->naxes-1)) {
		/*
		 * do the vector product properly
		 */
			mat_vect_mult(&t2, ROT(i+1), F(i+1));
			vect_cross(&t3, PSTAR(i), &t2);
			vect_add(&t1, &t1, &t3);

			mat_vect_mult(&t2, ROT(i+1), N(i+1));
			vect_add(&t1, &t1, &t2);
		}

		mat_vect_mult(&t2, IMAT(i), ALPHA(i));
		mat_vect_mult(&t3, IMAT(i), OMEGA(i));
		vect_cross(&t4, OMEGA(i), &t3);
		vect_add(&t2, &t2, &t4);

		vect_add(N(i), &t1, &t2);
		
	}

	/*
	 *  Compute the torque total for each axis
	 *
	 */

	for (i = 0; i < m->naxes; i++) {
		Motor	*motor = &m->motor[i];
		double	n;

		matt_vect_mult(&t1, ROT(i), &z0);

		switch (m->link[i].axistype) {
		case REVOLUTE:
			tau[i] = vect_dot(N(i), &t1);
			break;
		case PRISMATIC:
			tau[i] = vect_dot(F(i), &t1);
			break;
		}

		/*
		 * add actuator dynamics and friction
		 */
		n = motor->n * motor->n;

		tau[i] += n * (motor->J * Qddot[i].z +
				motor->B * Qdot[i].z +
				(Qdot[i].z > 0 ? motor->Cp : 0.0) +
				(Qdot[i].z < 0 ? motor->Cn : 0.0)
				);
	}
}

/* some functions to help in debugging */
static void
print_vector(char *s, double *v)
{
	int	j;

	printf("%10s: ", s);
	for (j=0; j<3; j++)
		printf("%15.3f", v[j]);
	putchar('\n');
}


static void
print_matrix(char *s, Mat *m)
{
	int	j;

	printf("%s:\n", s);
	printf(" %15.3f%15.3f%15.3f\n", m->a.x, m->b.x, m->c.x);
	printf(" %15.3f%15.3f%15.3f\n", m->a.y, m->b.y, m->c.y);
	printf(" %15.3f%15.3f%15.3f\n", m->a.z, m->b.z, m->c.z);
}
