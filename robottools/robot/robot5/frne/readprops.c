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
#include	<string.h>

#include	"dynlib.h"

extern int	dyn_verbose;

/*
 *	allocate various constants to the Props structure
 *	eg	mass, centre of mass vectors, and moment of inertia tensor
 */

#define	AXMAX	10

enum key_type {
	D, A, alpha_d, alpha, theta, theta_d, m, gravity,
	Xbar, Ybar, Zbar, Ixx, Iyy, Izz, Ixy, Iyz, Ixz,
	G, Bm, Jm, Cp, Cm, Km
};

#define	KEY(key)	{"key", key}

struct keys {
	char		*key;
	enum key_type	type;
} key_list[] = {
	KEY(D),
	KEY(A),
	KEY(alpha_d),
	KEY(alpha),
	KEY(theta),
	KEY(theta_d),
	KEY(m),
	KEY(gravity),
	KEY(Xbar),
	KEY(Ybar),
	KEY(Zbar),
	KEY(Ixx),
	KEY(Iyy),
	KEY(Izz),
	KEY(Ixy),
	KEY(Iyz),
	KEY(Ixz),
	KEY(G),
	KEY(Bm),
	KEY(Jm),
	KEY(Cp),
	KEY(Cm),
	KEY(Km),
};

#define	LENGTH_LIST	(sizeof(key_list)/sizeof(struct keys))
	

Manipulator *
dyn_readprops(fname)
char	*fname;
{
	Manipulator	*dyn_readprops2(), *manip;

	if ((manip = (Manipulator *)calloc(1, sizeof(Manipulator))) == NULL) {
		fprintf(stderr, "init_props: no mem\n");
		return NULL;
	}
	return dyn_readprops2(manip, fname);
}

Manipulator *
dyn_readprops2(manip, fname)
char		*fname;
Manipulator	*manip;
{
	char	b[200], key[200], arg[200];
	char	*type = NULL, *structure = NULL;
	char	fmt[200];
	int	i, n, gravflg = 0;
	double	value[AXMAX];
	enum key_type	kt;
	FILE	*fp;


	if ((fp = fopen(fname, "r")) == NULL)
		return NULL;

	/*
	 * get the manipulator type (name) and joint structure keys
	 */
	while (fgets(b, 200, fp) != NULL) {
		if (b[0] == '%')
			continue;
		if (sscanf(b, "%s %s", key, arg) == 2)
			if (strcmp(key, "type") == 0)
				type = strdup(arg);
			else if (strcmp(key, "structure") == 0)
				structure = strdup(arg);
			else if (strcmp(key, "gravity") == 0)
				if (sscanf(b, "%*s %lf %lf %lf",
					&(manip->gravity.x),
					&(manip->gravity.y),
					&(manip->gravity.z)
				  ) != 3) {
					fprintf(stderr, "init_props: bad gravity vector\n");
					return NULL;
					
				}
				else {
					manip->gravity.x *= -1.0;
					manip->gravity.y *= -1.0;
					manip->gravity.z *= -1.0;
					gravflg++;
				}
			if (type && structure && gravflg)
				break;
	}

	/*
	 * from joint structure compute number of axes and init props struct
	 */
	n = strlen(structure);
	if (n > AXMAX) {
		fprintf(stderr, "init_props: too many axes\n");
		return NULL;
	}
	if(dyn_verbose)
		fprintf(stderr, "Data file for %s, structure %s, %d joints\n",
		type, structure, n);

	manip->naxes = n;
	manip->name = type;
	manip->structure = structure;

	manip->link = (Link *)calloc(n, sizeof(Link));
	manip->motor = (Motor *)calloc(n, sizeof(Motor));
	manip->dv = (DynVars *)calloc(n, sizeof(DynVars));
	manip->q = (double *)calloc(n, sizeof(double));
	manip->qd = (double *)calloc(n, sizeof(double));
	manip->qdd = (double *)calloc(n, sizeof(double));

	for (i=0; i<n; i++) {
		manip->link[i].axistype = structure[i] == 'r' ? REVOLUTE :
							     PRISMATIC;
		if (structure[i] != 'r')
			fprintf(stderr, "init_props: prismatic axes not yet supported\n");
	}


	/*
	 * deal with other key word, each has njoint values
	 */
	while (fgets(b, 200, fp) != NULL) {
		char	*ky, *p;

		/*
		 * ditch coment lines
		 */
		if (b[0] == '%' || strlen(b) == 1)
			continue;

		/*
		 * get the keyword token
		 */
		ky = strtok(b, " \t");
		if (ky == NULL)
			continue;
		if (dyn_verbose)
			fprintf(stderr, "%s\n", ky);

		/*
		 * find it in the list
		 */
		for (i=0; i<LENGTH_LIST; i++)
			if (strcmp(key_list[i].key, ky) == 0) {
				kt = key_list[i].type;
				break;
			}
		if (i == LENGTH_LIST) {
			printf("unknown keyword \"%s\"\n", ky);
			continue;
		}

		/*
		 * parse the value tokens
		 */
		for (i=0; i<n; i++)
			if ((p = strtok(NULL, " \t")) == NULL)
				fprintf(stderr, "Error\n");
			else
				value[i] = atof(p);

		/*
		 * store the results
		 */
		for (i=0; i<n; i++)
			switch (kt) {
			case D:
				manip->link[i].D = value[i]; break;
			case A:
				manip->link[i].A = value[i]; break;
			case alpha_d:
				value[i] *= M_PI/180.0;
				/* fall through */
			case alpha:
				manip->link[i].alpha = value[i];
				sincos(value[i], &manip->link[i].alpha_s,
					&manip->link[i].alpha_c);
				break;
			case theta:
			case theta_d:
				break;
			case m:
				manip->link[i].m = value[i]; break;
			case Xbar:
				manip->link[i].rbar.x = value[i]; break;
			case Ybar:
				manip->link[i].rbar.y = value[i]; break;
			case Zbar:
				manip->link[i].rbar.z = value[i]; break;

			case Ixx:
				manip->link[i].I.a.x = value[i]; break;
			case Iyy:
				manip->link[i].I.b.y = value[i]; break;
			case Izz:
				manip->link[i].I.c.z = value[i]; break;
			case Ixy:
				manip->link[i].I.a.y =
				manip->link[i].I.b.x = -value[i]; break;
			case Iyz:
				manip->link[i].I.b.z =
				manip->link[i].I.c.y = -value[i]; break;
			case Ixz:
				manip->link[i].I.a.z =
				manip->link[i].I.c.x = -value[i]; break;
			case Bm:
				manip->motor[i].B = value[i]; break;
			case Jm:
				manip->motor[i].J = value[i]; break;
			case Cp:
				manip->motor[i].Cp = value[i]; break;
			case Cm:
				manip->motor[i].Cn = value[i]; break;
			case G:
				manip->motor[i].n = value[i]; break;
			case Km:
				manip->motor[i].k = value[i]; break;
			}
	}

	/*
	 * compute the COG link i wrt link i from link (i-1), Rpi* in 
	 * notation of Gonzalez et.al.  As per page 117.
	 */
	for (i=0; i<n; i++) {
		manip->link[i].r.x = manip->link[i].A;
		manip->link[i].r.y = manip->link[i].D *  manip->link[i].alpha_s;
		manip->link[i].r.z = manip->link[i].D *  manip->link[i].alpha_c;
	}

	fclose(fp);
	return manip;
}
