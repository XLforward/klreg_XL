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
 * constants defined for puma 560 robot
 * see Driels and Yang p355
 */
#define	A2	0.432
#define	A3	-.020
#define	D2	0.150
#define	D4	0.432
#define	D6	0.56

/* relative masses of links */
#define	M1	10.0
#define	M2	17.40
#define	M3	4.80
#define	M4	0.82
#define	M5	0.34
#define	M6	0.09

/* positions of centre of mass for links */
#define	XBAR1	0.0
#define	YBAR1	0.0
#define	ZBAR1	0.33
#define	XBAR2	-0.22
#define	YBAR2	0.0
#define	ZBAR2	0.1
#define	XBAR3	0.0
#define	YBAR3	0.0
#define	ZBAR3	0.21
#define	XBAR4	0.0
#define	YBAR4	0.0
#define	ZBAR4	0.0
#define	XBAR5	0.0
#define	YBAR5	0.0
#define	ZBAR5	0.03
#define	XBAR6	0.0
#define	YBAR6	0.0
#define	ZBAR6	0.03

/* moments of inertia about center of mass for links */
#define	IXX1	1.82
#define	IYY1	0.29
#define	IZZ1	1.81
#define	IXX2	1.76
#define	IYY2	2.91
#define	IZZ2	1.45
#define	IXX3	1.37
#define	IYY3	0.68
#define	IZZ3	0.82
#define	IXX4	0.39
#define	IYY4	0.39
#define	IZZ4	0.4
#define	IXX5	0.12
#define	IYY5	0.12
#define	IZZ5	0.1
#define	IXX6	0.03
#define	IYY6	0.03
#define	IZZ6	0.04

/*
 * gear ratios from Breaking away from VAL
 */
#define	RAT1	62.611
#define	RAT2	107.815
#define	RAT3	53.7063
#define	RAT4	76.03636
#define	RAT5	71.923
#define	RAT6	76.686

/*
 * other constants from unimation document
 */

/* motor inertias Kg-m^2 */
#define	J1INRT	0.0001979
#define	J2INRT	0.0001979
#define	J3INRT	0.0001979
#define	J4INRT	0.00001836
#define	J5INRT	0.00001836
#define	J6INRT	0.00001836

/*
 * guestimates for motor friction values
 */

/* motor static friction torque N-m */

#define	J1C_FRICT	0.05
#define	J2C_FRICT	0.05
#define	J3C_FRICT	0.05
#define	J4C_FRICT	0.005
#define	J5C_FRICT	0.005
#define	J6C_FRICT	0.005

/* motor damping coefficient N-m.sec/rad */

#define	J1D_COEFF	0.05	/* 0.02 */
#define	J2D_COEFF	0.002	/* 0.02 */
#define	J3D_COEFF	0.002	/* 0.02 */
#define	J4D_COEFF	0.002
#define	J5D_COEFF	0.002
#define	J6D_COEFF	0.002
