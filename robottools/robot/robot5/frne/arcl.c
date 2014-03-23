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
 * arcl.c
 *
 * Read/write manipulator trajectory data from an ARCL format file/stream.
 */

#ifdef	ARCL
#include	<stdio.h>
#include	"arcl.h"
#include	"stream.h"
#include	"string.h"

static int	njoints_r,
		njoints_w;

dyn_initstream_read(fp, manip_type, manip_structure, app_name)
FILE	*fp;
char	**manip_type,           /* returned manipulator type */
	**manip_structure,      /* returned structure */
	**app_name;             /* returned application name */
{
	magicrec		*mr;
	void			*p;

	if (stream_read(fp, STREAM_FWD, &p, NULL) != STREAM_MAGIC)
		return ERROR;
	
	mr = (magicrec *)p;

	if (mr->magic != STREAM_MAGICNUM)
		return ERROR;

	if (manip_type)
		*manip_type = strdup(mr->type);
	if (manip_structure)
		*manip_structure = strdup(mr->structure);
	if (app_name)
		*app_name = strdup(mr->application);

	njoints_r = strlen(mr->structure);
	return OK;
}

dyn_readjoints(fp, j, time)
FILE    *fp;
double  *j;
double  *time;
{
	void		*p;
	register int	i;
	register float	*d;
	int		itime;

	for (;;)
		switch ( stream_read(fp, STREAM_FWD, &p, &itime) ) {
		case STREAM_JN:
			d = (float *)p;
			for (i=0; i<njoints_r; i++)
				*j++ = *d++;
			*time = itime / 1000.0;
			return OK;
		case ERROR:
			return ERROR;
		default:
			break;
		}

}

FILE *
dyn_initstream_write(fp, manip_type, manip_structure, app_name)
FILE	*fp;
char    *manip_type,            /* manipulator type */
        *manip_structure,       /* structure */
        *app_name;              /* application name */
{
	magicrec		mr;

	strncpy(mr.type, manip_type, STREAM_STRLEN);
	strncpy(mr.structure, manip_structure, STREAM_STRLEN);
	strncpy(mr.application, app_name, STREAM_STRLEN);
	njoints_w = mr.njoints = strlen(manip_structure);
	mr.magic = STREAM_MAGIC;

	stream_write(fp, STREAM_MAGIC, &mr, sizeof(mr), 0);

	return OK;
}

dyn_writejoints(fp, j, time)
FILE    *fp;
double  *j;
double  time;
{
	float		jf[16];
	register int	i;
	int		itime;

	itime = (int) (time * 1000.0);

	for (i=0; i<njoints_w; i++)
		jf[i] = *j++;
	stream_write(fp, STREAM_JN, jf, njoints_w * sizeof(float), itime);
}
#endif
