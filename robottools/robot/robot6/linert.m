%LINERT	Return the manipulator link inertia in matrix form
%
%	LINERT(DYN, L)
%
%	Extract the manipulator link inertia for link L from DYN and return
%	it in matrix form

%	Copyright (C) 1994 Peter Corke

function J = linert(dyn, l)
	J = [	dh_dyn(l,10) dh_dyn(l,13) dh_dyn(l,15); ...
		dh_dyn(l,13) dh_dyn(l,11) dh_dyn(l,14); ...
		dh_dyn(l,15) dh_dyn(l,14) dh_dyn(l,12)	];
