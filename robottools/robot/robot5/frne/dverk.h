#define	MAX_FN	2048
#define	TOO_MANY_FN_CALLS_ERR	-1
#define	INITIAL_ERR		-2
#define	STEP_SIZE_TOO_SMALL_ERR	-3
#define	MEM_ERR			-4

#define	SIGN(a, b)      (b < 0.0)? (-a) : (a)
#define	MAX(a, b)       (a < b) ? (b) : (a)
#define	MIN(a, b)       (a < b) ? (a) : (b)
 
#define	SIXTH	.1666666666666667
#define	TRUE	1
#define	FALSE	0
#define	R1	.1666666666666667
#define	R2	.05333333333333333
#define	R3	.2133333333333333
#define	R4	.8333333333333333
#define	R5	-2.666666666666667
#define	R6	2.5
#define	R7	-2.578125
#define	R8	9.166666666666667
#define	R9	-6.640625
#define	R10	.8854166666666667
#define	R11	2.4
#define	R12	-8.0
#define	R13	6.560457516339869
#define	R14	-.3055555555555556
#define	R15	.3450980392156863
#define	R16	-.5508666666666667
#define	R17	1.653333333333333
#define	R18	-.9455882352941176
#define	R19	-.324
#define	R20	.2337882352941176
#define	R21	2.035465116279070
#define	R22	-6.976744186046512
#define	R23	5.648179814561484
#define	R24	-.1373815676141258
#define	R25	.2863022661036103
#define	R26	.1441785567164738
#define	R27	.075
#define	R28	.3899286987522282
#define	R29	.3194444444444444
#define	R30	.1350383631713555
#define	R31	.01078329882677709
#define	R32	.06980519480519481
#define	R33	.00625
#define	R34	.006963012477718360
#define	R35	-.006944444444444444
#define	R36	.006138107416879795
#define	R37	.06818181818181818
#define	R38	-.01078329882677709
#define	R39	-.06980519480519481
#define	C1	.1666666666666667
#define	C2	.2666666666666667
#define	C3	.6666666666666667
#define	C4	.8333333333333333
#define	C5	.06666666666666667
#define	C6	120.4272910821709
#define	RREB	2.775557562e-17
#define	DWARF	1.058791184e-22

struct com_link {
	int	fns,			/* c(24) */
		fn_calls,		/* c(6) */
		scale,			/* c(15) */
		succ,			/* no. of successful steps c(22) */
		fails;			/* no. of successive failures c(23) */
	double	wgt_norm,		/* weighted norm y c(12) */
		hmin,			/* c(13) */
		hmag,			/* c(14) */
		hmax,			/* c(16) */
		xtrial,			/* c(17) */
		htrial,			/* c(18) */
		hstart,			/* c(4) */
		est;			/* c(19) */
};

