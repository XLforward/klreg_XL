/*
 *
 *  RNE.C	 MEX file version of RNE.M
 *
 *	TAU = RNE(DYN, Q, QD, QDD)
 *	TAU = RNE(DYN, [Q QD QDD])
 *
 *	Returns the joint torque required to achieve the specified joint position,
 *	velocity and acceleration state.
 *
 *	Gravity is assumed to be acting in the -Z direction with acceleration of
 *	9.81m/s/s, but this may be overriden by providing a gravity acceleration
 *	vector [gx gy gz].
 *
 *	TAU = RNE(DYN, Q, QD, QDD, GRAV)
 *	TAU = RNE(DYN, [Q QD QDD], GRAV)
 *
 *	An external force/moment acting on the end of the manipulator may also be
 *	specified by a 6-element vector [Fx Fy Fz Mx My Mz].
 *
 *	TAU = RNE(DYN, Q, QD, QDD, GRAV, FEXT)
 *	TAU = RNE(DYN, [Q QD QDD], GRAV, FEXT)
 *
 *	where	Q, QD and QDD are row vectors of the manipulator state; pos, vel, and accel.
 *
 * Updated 3/00 for Matlab 5 and object support.
 *
 *	(c) Peter Corke
 *	CSIRO Manufacturing Science & Technology
 *	Queensland Centre for Advanced Technology
 *	Pinjarra Hills, AUSTRALIA.
 */

#include "mex.h"
#include <math.h>
#include	"dynlib.h"

/* Input Arguments */
#define	ROBOT_IN	prhs[0]
#define	A1_IN	prhs[1]
#define	A2_IN	prhs[2]
#define	A3_IN	prhs[3]
#define	A4_IN	prhs[4]
#define	A5_IN	prhs[5]

/* Output Arguments */
#define	TAU_OUT	plhs[0]

/* Some useful things */
#define	NUMROWS(x)	mxGetM(x)
#define	NUMCOLS(x)	mxGetN(x)
#define	NUMELS(x)	(mxGetN(x)*mxGetM(x))
#define	POINTER(x)	mxGetPr(x)

/* forward defines */
void rne(Manipulator *manip, int nq, double *tau, double *q,
	double *qd, double *qdd, double *fext);
static char * mstruct_getstr(mxArray *m, int i, char *field);
static int mstruct_getint(mxArray *m, int i, char *field);
static double mstruct_getreal(mxArray *m, int i, char *field);
static double * mstruct_getrealvect(mxArray *m, int i, char *field);
static mxArray * mstruct_get_element(mxArray *m, int i, char *field);
void error(char *s, ...);


/* default values for gravity and external load */
const double	fext0[6];

void mexFunction(
	int		nlhs, mxArray	*plhs[],
	int		nrhs, const mxArray	*prhs[]
	)
{
	double	*p;
	double	*t,*y;
	double	*q, *qd, *qdd;
	unsigned int	m,n;
	int	j, naxes, nq;
	double	*fext = (double *)fext0;
	double *grav = NULL;
	Manipulator	*manip;
	mxArray		*mx_robot;
	mxArray		*mx_links;
	char		*name;
	static int	firstime = 0;

	if (  !mxIsStruct(ROBOT_IN) ||
	      (strcmp(mxGetClassName(ROBOT_IN), "robot") != 0)
	) {
		mexErrMsgTxt("first argument is not a robot structure\n");
	}

	mx_robot = ROBOT_IN;

	if (mstruct_getint(mx_robot, 0, "mdh") > 0) {
		mexErrMsgTxt("cannot handle modified D&H parameters\n");
	}
		
	naxes = mstruct_getint(mx_robot, 0, "n");

	/* Handle the different calling formats */
	switch (nrhs) {
	case 2:
/*	TAU = RNE(DYN, [Q QD QDD]) */
		if (NUMCOLS(A1_IN) != 3 * naxes)
			mexErrMsgTxt("RNE too few cols in [Q QD QDD]");
		q = POINTER(A1_IN);
		nq = NUMROWS(A1_IN);
		qd = &q[3*nq];
		qdd = &q[6*nq];
		break;
		
	case 3:
/*	TAU = RNE(DYN, [Q QD QDD], GRAV) */
		if (NUMCOLS(A1_IN) != (3 * naxes))
			mexErrMsgTxt("RNE too few cols in [Q QD QDD]");
		q = POINTER(A1_IN);
		nq = NUMROWS(A1_IN);
		qd = &q[3*nq];
		qdd = &q[6*nq];

		if (NUMELS(A2_IN) != 3)
			mexErrMsgTxt("RNE gravity vector expected");
		grav = POINTER(A2_IN);
		break;

	case 4:
/*	TAU = RNE(DYN, Q, QD, QDD)
	TAU = RNE(DYN, [Q QD QDD], GRAV, FEXT) */
		if (NUMCOLS(A1_IN) == (3 * naxes)) {
			q = POINTER(A1_IN);
			nq = NUMROWS(A1_IN);
			qd = &q[3*nq];
			qdd = &q[6*nq];

			if (NUMELS(A2_IN) != 3)
				mexErrMsgTxt("RNE gravity vector expected");
			grav = POINTER(A2_IN);
			if (NUMELS(A3_IN) != 6)
				mexErrMsgTxt("RNE Fext vector expected");
			fext = POINTER(A2_IN);
		} else {
			int	nqd = NUMROWS(A2_IN),
				nqdd = NUMROWS(A3_IN);

			nq = NUMROWS(A1_IN);
			if ((nq != nqd) || (nqd != nqdd))
				mexErrMsgTxt("RNE Q QD QDD must be same length");
			if ( (NUMCOLS(A1_IN) != naxes) ||
			     (NUMCOLS(A2_IN) != naxes) ||
			     (NUMCOLS(A3_IN) != naxes)
			) 
				mexErrMsgTxt("RNE Q must have Naxis columns");
			q = POINTER(A1_IN);
			qd = POINTER(A2_IN);
			qdd = POINTER(A3_IN);
		}
		break;

	case 5: {
/*	TAU = RNE(DYN, Q, QD, QDD, GRAV) */
		int	nqd = NUMROWS(A2_IN),
			nqdd = NUMROWS(A3_IN);

		nq = NUMROWS(A1_IN);
		if ((nq != nqd) || (nqd != nqdd))
			mexErrMsgTxt("RNE Q QD QDD must be same length");
		if ( (NUMCOLS(A1_IN) != naxes) ||
		     (NUMCOLS(A2_IN) != naxes) ||
		     (NUMCOLS(A3_IN) != naxes)
		) 
			mexErrMsgTxt("RNE Q must have Naxis columns");
		q = POINTER(A1_IN);
		qd = POINTER(A2_IN);
		qdd = POINTER(A3_IN);
		if (NUMELS(A4_IN) != 3)
			mexErrMsgTxt("RNE gravity vector expected");
		grav = POINTER(A2_IN);
		break;
	}

	case 6: {
/*	TAU = RNE(DYN, Q, QD, QDD, GRAV, FEXT) */
		int	nqd = NUMROWS(A2_IN),
			nqdd = NUMROWS(A3_IN);

		nq = NUMROWS(A1_IN);
		if ((nq != nqd) || (nqd != nqdd))
			mexErrMsgTxt("RNE Q QD QDD must be same length");
		if ( (NUMCOLS(A1_IN) != naxes) ||
		     (NUMCOLS(A2_IN) != naxes) ||
		     (NUMCOLS(A3_IN) != naxes)
		) 
			mexErrMsgTxt("RNE Q must have Naxis columns");
		q = POINTER(A1_IN);
		qd = POINTER(A2_IN);
		qdd = POINTER(A3_IN);
		if (NUMELS(A4_IN) != 3)
			mexErrMsgTxt("RNE gravity vector expected");
		grav = POINTER(A4_IN);
		if (NUMELS(A5_IN) != 6)
			mexErrMsgTxt("RNE Fext vector expected");
		fext = POINTER(A5_IN);
		break;
	}
	default:
		mexErrMsgTxt("RNE wrong number of arguments.");
	}


	/* Create a matrix for the return argument */
	TAU_OUT = mxCreateDoubleMatrix(nq, naxes, mxREAL);


	/* Allocate the manip structure needed by dynlib */
	if ((manip = (Manipulator *)mxCalloc(1, sizeof(Manipulator))) == NULL) {
		fprintf(stderr, "RNE: no mem\n");
	}

	/* and allocate space for motor, link etc */
	manip->link = (Link *)mxCalloc(naxes, sizeof(Link));
	manip->motor = (Motor *)mxCalloc(naxes, sizeof(Motor));
	manip->dv = (DynVars *)mxCalloc(naxes, sizeof(DynVars));
#ifdef	notdef
	/* not needed */
	manip->q = (double *)mxCalloc(naxes, sizeof(double));
	manip->qd = (double *)mxCalloc(naxes, sizeof(double));
	manip->qdd = (double *)mxCalloc(naxes, sizeof(double));
#endif
	/* now fill in the manip structure */
	manip->naxes = naxes;

	/* 
	 * elements of the robot structure are:
	 *	name:
	 *	manuf:
	 *	comment:
	 *	mdh:
	 *	link:
	 *	n:
	 *	gravity:
	 *	base:
	 */

	name = mstruct_getstr(mx_robot, 0, "name");
	manip->name = name;
	/*
	printf("Robot name: %s, %d axes\n", name, naxes);
	*/

	if (firstime == 0) {
		printf("Fast RNE: (c) Peter Corke 2000\n");
		firstime = 1;
	}

	grav = mstruct_getrealvect(mx_robot, 0, "gravity");
	manip->gravity.x = grav[0];
	manip->gravity.y = grav[1];
	manip->gravity.z = grav[2];

	/* get pointer to link structures */
	mx_links = mxGetField(mx_robot, 0, "link");
	if (mx_links == NULL)
		mexErrMsgTxt("couldnt find element link in robot structure");

	/* build joint structure string */
	manip->structure = mxCalloc(naxes+1, sizeof(char));
	for (j=0; j<naxes; j++) {
		manip->structure[j] = mstruct_getint(mx_links, j, "sigma") ? 'r' : 'p';
	}
	manip->structure[naxes] = '\0';


	/* Now unpack data from the Matlab link structures into
	 * dynlib Link and Motor structures.
	 *
	 * There is some overhead in all this, but when computing rne for
	 * a long trajectory we will be ahead.
	 *
	 * Elements of the link structure are:
	 *
	 *	alpha: 
	 *	A:
	 *	theta:
	 *	D:
	 *	sigma:
	 *	mdh:
	 *	m:
	 *	r:
	 *	I:
	 *	Jm:
	 *	G:
	 *	B:
	 *	Tc:
	 */


	for (j=0; j<naxes; j++) {
		Motor	*motor = &manip->motor[j];
		Link	*link = &manip->link[j];
		double	alpha, *p;
		mxArray	*m;
		int	length;

		link->axistype = mstruct_getint(mx_links, j, "sigma") == 0 ?
			REVOLUTE : PRISMATIC;
		link->D = mstruct_getreal(mx_links, j, "D");
		link->A = mstruct_getreal(mx_links, j, "A");
		link->theta = mstruct_getreal(mx_links, j, "theta");
		alpha = link->alpha = mstruct_getreal(mx_links, j, "alpha");
		link->alpha_s = sin(alpha);
		link->alpha_c = cos(alpha);

		link->m = mstruct_getreal(mx_links, j, "m");
		p = mstruct_getrealvect(mx_links, j, "r");
		link->rbar.x = p[0];
		link->rbar.y = p[1];
		link->rbar.z = p[2];
		/*
		printf("j%d: m=%f, rbar=%f, %f, %f\n", 
			j, link->m, p[0], p[1], p[2]);
		*/

		m = mstruct_get_element(mx_links, j, "I");
		length = mxGetM(m) * mxGetN(m);
		p = mxGetPr(m);

		switch (length) {
		case 6:
			/* the 6-vector case Ixx Iyy Izz Ixy Iyz Ixz */
			link->I.a.x = p[0];
			link->I.b.y = p[1];
			link->I.c.z = p[2];
			link->I.a.y =
			link->I.b.x = -p[3];
			link->I.b.z =
			link->I.c.y = -p[4];
			link->I.a.z =
			link->I.c.x = -p[5];
			break;
		case 9:
			/* the 3x3 matrix case, remember Matlab matrix
			 * ordering (column major)
			 */

			/* first column */
			link->I.a.x = p[0];
			link->I.a.y = p[1];
			link->I.a.z = p[2];
			/* second column */
			link->I.b.x = p[3];
			link->I.b.y = p[4];
			link->I.b.z = p[5];
			/* third column */
			link->I.c.x = p[6];
			link->I.c.y = p[7];
			link->I.c.z = p[8];
			break;
		default:
			mexErrMsgTxt("link inertia must have 6 or 9 elements");
		};

		motor->J = mstruct_getreal(mx_links, j, "Jm");
		motor->B = mstruct_getreal(mx_links, j, "B");
		motor->n = mstruct_getreal(mx_links, j, "G");

		m = mstruct_get_element(mx_links, j, "Tc");
		length = mxGetM(m) * mxGetN(m);
		p = mxGetPr(m);
		switch (length) {
		case 2:
			motor->Cp = p[0];
			motor->Cn = p[1];
			break;
		case 1:
			motor->Cp = 
			motor->Cn = p[0];
			break;
		default:
			mexErrMsgTxt("Coulomb friction must have 1 or 2 elements");
		};
	}

	rne(manip, nq, POINTER(TAU_OUT), q, qd, qdd, fext);

	/* free the datastructures */
	mxFree(manip->link);
	mxFree(manip->motor);
	mxFree(manip->dv);
	mxFree(manip);
}

void
rne(Manipulator *manip, int nq, double *tau, double *q, double *qd, double *qdd, 
	double *fext)
{
	Vect		Qd[N],
			Qdd[N];
	int		i, j, r;
	double		torq[N];

	for (i = 0; i < manip->naxes; i++)
		Qdd[i].x = Qdd[i].y  = Qd[i].x = Qd[i].y = 0.0;

#define	MEL(x,R,C)	(x[(R)+(C)*nq])

	/* for each point in the input trajectory */
	for (j=0; j<nq; j++) {
		/*
		 * update all position dependent variables
		 */
		for (i = 0; i < manip->naxes; i++) {
			switch (manip->link[i].axistype) {
			case REVOLUTE:
				manip->link[i].theta = MEL(q,j,i); break;
			case PRISMATIC:
				manip->link[i].D = MEL(q,j,i); break;
			}
			/* compute the link rotation matrix */
			rot_mat (&manip->dv[i].Rot, manip->link[i].theta,
				&manip->link[i]);

			/* compute the link translation vector */
			manip->link[i].r.x = manip->link[i].A;
			manip->link[i].r.y = 
				manip->link[i].D *  manip->link[i].alpha_s;
			manip->link[i].r.z = 
				manip->link[i].D *  manip->link[i].alpha_c;

			/* put the velocity and acceleration in vector form */
			Qd[i].z = MEL(qd,j,i);
			Qdd[i].z = MEL(qdd,j,i);
		}

		dyn_newton_euler(manip, torq, Qd, Qdd, fext, &(manip->gravity));

		/* stash the result */
		for (i = 0; i < manip->naxes; i++)
			MEL(tau,j,i) = torq[i];
	}
}

/*************************************************************************
 * Matlab structure access methods
 *************************************************************************/
static mxArray *
mstruct_get_element(mxArray *m, int i, char *field)
{
	mxArray	*e;

	if (mxIsCell(m)) {
#ifdef	DEBUG
		printf("%d x %d\n", mxGetM(m), mxGetN(m));
#endif
		/* get the i'th cell from the cell array */
		if ((e = mxGetCell(m, i)) == NULL)
			error("get_element: field %s: cant get cell element %d",field, i);
	} else
		e = m;

	if (!mxIsStruct(e))
		mexErrMsgTxt("get_element: expecting a structure");
	if ((e = mxGetField(e, 0, field)) != NULL)
		return e;
	else {
		error("No such field as %s", field);
	}
}

static char *
mstruct_getstr(mxArray *m, int i, char *field)
{
	mxArray	*e;
	int	status;
	int	buflen;
	char	*buf;

	e = mstruct_get_element(m, i, field);

	buflen = (mxGetM(e) * mxGetN(e)) + 1;

       /* Allocate enough memory to hold the converted string. */ 
	buf = mxCalloc(buflen, sizeof(char));
	 if (buf == NULL)
	    mexErrMsgTxt("Not enough heap space to hold converted string.");

       /* Copy the string data from matrix and place it into buf. */ 
         status = mxGetString(e, buf, buflen); 
         if (status == 0)
           return buf;
         else
           mexErrMsgTxt("Could not convert string data.");
}

static int
mstruct_getint(mxArray *m, int i, char *field)
{
	mxArray	*e;

	e = mstruct_get_element(m, i, field);

	return (int) mxGetScalar(e);
}

static double
mstruct_getreal(mxArray *m, int i, char *field)
{
	mxArray	*e;

	e = mstruct_get_element(m, i, field);

	return mxGetScalar(e);
}

static double *
mstruct_getrealvect(mxArray *m, int i, char *field)
{
	mxArray	*e;

	e = mstruct_get_element(m, i, field);

	return mxGetPr(e);
}

#include	<stdarg.h>

void error(char *s, ...);
void
error(char *s, ...)
{
	char	b[BUFSIZ];

	va_list	ap;

	va_start(ap, s);

	vsprintf(b, s, ap);

	mexErrMsgTxt(b);
}
