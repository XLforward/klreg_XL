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
#include	<stdio.h>
#include	<math.h>

#define	N	6
#define	N2	(2*N)
#define	GRAVITY	9.81
#define	TRUE	1
#define	FALSE	0

enum _axistype {
	REVOLUTE,
	PRISMATIC
};

typedef struct vector {
	double	x, y, z;
} Vect;

typedef struct matrix {
	Vect	a, b, c;
} Mat;

typedef struct homogeneous_matrix {
	Vect	n, o, a, p;
} DH_Mat;

typedef struct _load {
	int	load;	/* flag to indicate an external load */
	Vect	f,	/* external load on last link */
		n;	/* external moment referred to coords of last link */
	Mat	I;	/* inertial tensor of load */
} Load;

typedef struct _link {
	Vect	r,		/* distance of ith origin from i-1th wrt ith */
		rbar;		/* centre of mass of link i from ith origin */
	double	m;		/* mass of link i */
	double	alpha;		/* link twist */
	double	alpha_s,
		alpha_c;	/* sin, cos of alpha */
	double	A,		/* D&H parameters */
		D,
		theta;
	enum _axistype	axistype;	/* axis type; revolute or prismatic */
	Mat	I;		/* inertia tensor of link i */
} Link;

typedef struct actuator {
	double	J,		/* actuator inertia */
		n,		/* gear ratio */
		k,		/* motor torque constant */
		/* these parameters are motor referenced */
		B,		/* actuator friction damping coefficient */
		Cp,		/* actuator Coulomb friction coeffient */
		Cn;		/* actuator Coulomb friction coeffient */
} Motor;
		

typedef struct _dynvars {
	Vect	omega,		/* angular velocity */
		alpha,		/* angular acceleration */
		acc,		/* acceleration */
		abar,		/* acceleration of centre of mass */
		f,
		n;
	Mat	A;		/* a matrix to shorten calculations */
	Mat	Rot;		/* link rotation matrix */
} DynVars;

typedef struct manipulator {
	/* static properties */
	int		naxes;
	char		*name;
	char		*structure;
	Vect		gravity;
	Link		*link;	/* properties of manipulator	*/
	Motor		*motor;	/* properties of actuators	*/

	/* variables and state */
	double		*q,		/* joint postions	*/
			*qd,		/* joint velocities	*/
			*qdd;		/* joint accelerations	*/
	DynVars		*dv;
} Manipulator;

#define	Q(m)		((m)->q)
#define	QD(m)		((m)->qd)
#define	QDD(m)		((m)->qdd)

#define	DV(m)	((m)->dv)
#define	SC(m)	((m)->sc)

Manipulator	*dyn_readprops();

double	vect_dot();
