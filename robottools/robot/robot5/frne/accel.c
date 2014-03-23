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
 * Compute the forward dynamics, joint accelerations given joint torques.
 *
 * Implement Method 1 in the paper by Orin and Walker to compute the
 * inertia matrix, and bias torques.
 *
 *  Uses the NE kernel.
 *
 * Inputs:	q	current joint angles
 *		qd	current joint speed
 *		tau	applied acutator torques
 *		load	external tool tip load force/moment
 *
 * Output:	qdd	resultant joint acceleration
 */
#include	"dynlib.h"

extern int	dyn_verbose;
static Vect	zerograv = {0.0, 0.0, 0.0};

/*
 * Return the acceleration of the axes given the input torque and present state
 */
dyn_accel(manip, tau_act, load)
Manipulator	*manip;
double		*tau_act;	/* actuator torques */
Load		*load;		/* tip load */
{
	Vect		Qd[N],
			Qdd[N];
	double		H[N][N],
			Hinv[N][N],
			tau_nett[N],
			tau[N];
	int		i, j, r;

	/*
	 * update all position dependent variables
	 */
	for (i = 0; i < manip->naxes; i++)
		rot_mat (&manip->dv[i].Rot, manip->q[i], &manip->link[i]);

	/*
	 * tau = M(theta)*qddot + C(theta_dot, theta) + G(theta) + tau_load
	 * first get lumped C & G terms by setting all joint accelerations 
	 * to 0.0.
	 */
	for (i = 0; i < manip->naxes; i++) {
		Qdd[i].x = Qdd[i].y = Qdd[i].z = 0.0;
		Qd[i].x = Qd[i].y = 0.0;
		Qd[i].z = QD(manip)[i];
	}

	dyn_newton_euler(manip, tau, Qd, Qdd, load, &(manip->gravity));

	/* subract G & C lumped values of torques from joint actuator torques */
		

	for (i = 0; i < manip->naxes; i++)
		tau_nett[i] = tau_act[i] - tau[i];

	/*
	 * Now evaluate newton-euler equations to obtain H(q) inertia matrix
	 * setting each joint acceleration equal to 1.0 in turn to find each
	 * column of H.
	 * Evaluation is done with zero velocities and external forces and 
	 * torques (external f & t not done yet)
	 * gravity forces set to zero
	 */
	for (i = 0; i < manip->naxes; i++)
		Qd[i].z = 0.0;	/* zero velocity */

	for (i = 0; i < manip->naxes; i++) {
		Qdd[i].z = 1.0;

		dyn_newton_euler(manip, tau, Qd, Qdd, load, &zerograv);

						      /* ^^ zero gravity */

		for (j = 0; j < manip->naxes; j++) 	/* M matrix columns are joint torques */
			H[j][i] = tau[j];
		Qdd[i].z = 0.0;		/* reset this acceleration component */
	}
	if (dyn_verbose) {
		printf("H:\n");
		for (i = 0; i < manip->naxes; i++) {
			for (j = 0; j < manip->naxes; j++)
				printf ("%12.3lf ", H[i][j]);
			printf ("\n");
		}
	}

	/*
	 * invert the H(q) matrix and solve for joint accelerations
	 */
	if ((r = inv(H, Hinv, manip->naxes)) != 0)		/* invert M */
		perr ("accel: not positive definite matrix\n");

#ifdef DEBUG
	printf("H^-1:\n");
	for (i = 0; i < manip->naxes; i++) {
		for (j = 0; j < manip->naxes; j++)
			printf ("%12.3lf ", Hinv[i][j]);
		printf ("\n");
	}
#endif

	/* get accelerations qddot = H^-1 * tau */

	gen_mat_vect_mult(QDD(manip), Hinv, tau_nett, manip->naxes);
	if (dyn_verbose) {
		printf("tau_n: ");
		for (i = 0; i < manip->naxes; i++)
			printf ("%12.3lf ", tau_nett[i]);
		printf ("\n");
		printf("qdd: ");
		for (i = 0; i < manip->naxes; i++)
			printf ("%12.3lf ", QDD(manip)[i]);
		printf ("\n");
	}
}

dyn_H(manip, H, load)
Manipulator	*manip;
double		H[N][N];
Load		*load;		/* tip load */
{
	Vect		Qd[N],
			Qdd[N];
	double		tau[N];
	int		i, j;

	/*
	 * update all position dependent variables
	 */
	for (i = 0; i < manip->naxes; i++)
		rot_mat (&manip->dv[i].Rot, manip->q[i], &manip->link[i]);


	/*
	 * Now evaluate newton-euler equations to obtain H(q) inertia matrix
	 * setting each joint acceleration equal to 1.0 in turn to find each
	 * column of H.
	 * Evaluation is done with zero velocities and external forces and 
	 * torques (external f & t not done yet)
	 * gravity forces set to zero
	 */
	for (i = 0; i < manip->naxes; i++) {
		Qd[i].x = Qd[i].y = Qd[i].z = 0.0;	/* zero velocity */
		Qdd[i].x = Qdd[i].y = Qdd[i].z = 0.0;	/* zero acceln */
	}

	for (i = 0; i < manip->naxes; i++) {
		Qdd[i].z = 1.0;

		dyn_newton_euler(manip, tau, Qd, Qdd, load, &zerograv);

						      /* ^^ zero gravity */

		for (j = 0; j < manip->naxes; j++) 	/* M matrix columns are joint torques */
			H[j][i] = tau[j];
		Qdd[i].z = 0.0;		/* reset this acceleration component */
	}
#ifdef DEBUG
	for (i = 0; i < manip->naxes; i++) {
		for (j = 0; j < manip->naxes; j++)
			printf ("\t% .3g ", H[i][j]);
		printf ("\n");
	}
#endif
}

dyn_gravity(manip, tau, load)
Manipulator	*manip;
double		*tau;
Load		*load;		/* tip load */
{
	Vect		Qd[N],
			Qdd[N];
	int		i, j;

	/*
	 * update all position dependent variables
	 */
	for (i = 0; i < manip->naxes; i++)
		rot_mat (&manip->dv[i].Rot, manip->q[i], &manip->link[i]);


	/*
	 * Now evaluate newton-euler equations to obtain gravity by setting
	 * acceleration and velocity to zero.
	 * Evaluation is done with zero velocities and external forces and 
	 * torques (external f & t not done yet)
	 * gravity forces set to zero
	 */
	for (i = 0; i < manip->naxes; i++) {
		Qd[i].x = Qd[i].y = Qd[i].z = 0.0;	/* zero velocity */
		Qdd[i].x = Qdd[i].y = Qdd[i].z = 0.0;	/* zero acceln */
	}

	dyn_newton_euler(manip, tau, Qd, Qdd, load, &(manip->gravity));
}

dyn_velocity(manip, tau, load)
Manipulator	*manip;
double		*tau;
Load		*load;		/* tip load */
{
	Vect		Qd[N],
			Qdd[N];
	int		i, j;

	/*
	 * update all position dependent variables
	 */
	for (i = 0; i < manip->naxes; i++)
		rot_mat (&manip->dv[i].Rot, manip->q[i], &manip->link[i]);


	/*
	 * Now evaluate newton-euler equations to obtain vel.terms by setting
	 * acceleration and gravity to zero.
	 */
	for (i = 0; i < manip->naxes; i++) {
		Qd[i].x = Qd[i].y = 0.0;
		Qd[i].z = QD(manip)[i];
		Qdd[i].x = Qdd[i].y = Qdd[i].z = 0.0;	/* zero acceln */
	}

	dyn_newton_euler(manip, tau, Qd, Qdd, load, &zerograv);
}

dyn_ne(manip, tau, load)
Manipulator	*manip;
double		*tau;
Load		*load;
{
	Vect		Qd[N],
			Qdd[N];
	int		i;

	/*
	 * update all position dependent variables
	 */
	for (i = 0; i < manip->naxes; i++)
		rot_mat (&manip->dv[i].Rot, manip->q[i], &manip->link[i]);

	/*
	 * tau = M(theta)*qddot + C(theta_dot, theta) + G(theta) + tau_load
	 * first get lumped C & G terms by setting all joint accelerations 
	 * to 0.0.
	 */
	for (i = 0; i < manip->naxes; i++) {
		Qdd[i].x = Qdd[i].y = Qdd[i].z = 0.0;
		Qd[i].x = Qd[i].y = 0.0;
		Qd[i].z = QD(manip)[i];
	}

	dyn_newton_euler(manip, tau, Qd, Qdd, load, &(manip->gravity));
}
