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

vect_cross (r, a, b)
register Vect	*r, *a, *b;
{
	r->x = a->y*b->z - a->z*b->y;
	r->y = a->z*b->x - a->x*b->z;
	r->z = a->x*b->y - a->y*b->x;
}

double
vect_dot (a, b)
register Vect	*a, *b;
{
	return a->x * b->x + a->y * b->y + a->z * b->z;
}

vect_add (r, a, b)
register Vect	*r, *a, *b;
{
	r->x = a->x + b->x;
	r->y = a->y + b->y;
	r->z = a->z + b->z;
}

scal_mult (r, a, s)
register Vect	*r, *a;
double		s;
{
	r->x = s*a->x;
	r->y = s*a->y;
	r->z = s*a->z;
}

mat_vect_mult (r, m, v)
register Mat	*m;
register Vect	*v, *r;
{
	r->x = m->a.x*v->x + m->b.x*v->y + m->c.x*v->z;
	r->y = m->a.y*v->x + m->b.y*v->y + m->c.y*v->z;
	r->z = m->a.z*v->x + m->b.z*v->y + m->c.z*v->z;
}

/* multiplies by the transpose of the matrix */
matt_vect_mult (r, m, v)
register Mat	*m;
register Vect	*v, *r;
{
	r->x = m->a.x*v->x + m->a.y*v->y + m->a.z*v->z;
	r->y = m->b.x*v->x + m->b.y*v->y + m->b.z*v->z;
	r->z = m->c.x*v->x + m->c.y*v->y + m->c.z*v->z;
}

/* does a real matrix and vector multiplication */
gen_mat_vect_mult (c, a, b, n)
register double	*b, *c;
double	a[N][N];
int	n;
{
	int	i, j;
	double	t;

	for (i = 0; i < n; i++) {
		t = 0;
		for (j = 0; j < n; j++)
			t += a[i][j] * b[j];
		*c++ = t;
	}
}

/* forms the r vectors given the homogeneous matrices */
#ifdef	notdef
dh_to_r (t, r)
DH_Mat	t[];
Link	*r;
{
	int	i;

	for (i = 0; i < N; i++) {
		r->r[i].x = t[i].n.x*t[i].p.x + t[i].o.x*t[i].p.y + 
				t[i].a.x*t[i].p.z;
		r->r[i].y = t[i].n.y*t[i].p.x + t[i].o.y*t[i].p.y + 
				t[i].a.y*t[i].p.z;
		r->r[i].z = t[i].n.z*t[i].p.x + t[i].o.z*t[i].p.y + 
				t[i].a.z*t[i].p.z;
	}
}
#endif

perr (s)
char	*s;
{
	fprintf (stderr, "%s", s);
	exit (1);
}
