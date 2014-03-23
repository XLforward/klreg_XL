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

/*
 * build a link rotation matrix
 */
rot_mat (r, q, l)
Mat			*r;
double			q;
register Link		*l;
{
	register Mat	*mp;
	double		s, c;

#ifdef	sun
	sincos(q, &s, &c);
#else
	s = sin(q);
	c = cos(q);
#endif

	r->a.x = c;	r->b.x = -l->alpha_c*s;		r->c.x = l->alpha_s*s;
	r->a.y = s;	r->b.y = l->alpha_c*c;		r->c.y = -l->alpha_s*c;
	r->a.z = 0.0;	r->b.z = l->alpha_s;		r->c.z = l->alpha_c;
}
