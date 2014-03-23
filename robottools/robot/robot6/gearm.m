%GEARM	Return the manipulator gearing in matrix form
%
%	GEARM(DYN)
%
%	Extract the manipulator gearing information from DYN and return
%	it as a diagonal matrix.

%	Copyright (C) 1994 Peter Corke

function G = gearm(dyn)
	G = diag(dyn(:,17));
