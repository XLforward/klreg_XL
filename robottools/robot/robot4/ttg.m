%TTG	Return the i'th homogeneous transform from a trajectory
%
%	TI = TTG(T, I) returns the I'th homogeneous transform from a trajectory
%			matrix T.
%
%	See also  CTRAJ

%	Copright (C) Peter Corke 1995
function T = ttg(TG, i)
	T = reshape(TG(i,:), 4,4);
