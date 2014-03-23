#
#  Modify the following line to suit your site.
#
#	-DNUM_RECIPES	if you wish to use numerical integration routine
#			from Numerical Recipes in C.  You must have NRC
#			package installed.
#
#	-DARCL		if you wish to utilize ARCL format trajectory files.
#			You must have the ARCL package installed.
#			
#OPTIONS =	-DNUM_RECIPES -DARCL
#OPTIONS =	-DARCL -g
OPTIONS =
CC =		i486-linuxlibc5-gcc

#
# if any of the above options are enabled, set the path to the package
#
NRCDIR	= ../../src/nrc
ARCLDIR = /usr/users/robot/arcl/h

OBJ 	= \
	integrate.o \
	accel.o \
	inv.o \
	rotate.o \
	ne.o \
	math.o \
	dverk.o \
	readprops.o \
	arcl.o
INCS = -I$(ARCLDIR) -I$(NRCDIR)
CFLAGS	= -O $(INCS) $(OPTIONS)

LIB	= dynsim.a

all:	frne.mex

frne.mex:	frne.c $(LIB)
	cmex frne.c $(LIB)

$(LIB):	$(OBJ)
	ar cr $(LIB) $(OBJ)
	ranlib $(LIB)

clean:
	-rm -f *.o core mon.out *.a *.mex*
	-(cd eg; make clean);

print:
	tgrind $(src)

$(OBJ):	dynlib.h
dverk.o:	dverk.c dverk.h

shar:
	shar -o dl -L 64 -M *

tar:
	tar cf ../dl.tar .
	compress ../dl.tar
