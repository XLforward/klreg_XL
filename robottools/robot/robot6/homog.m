function h = homog(r)
%
% HOMOG(r)
%	return a homogeneous equivalent of a rotation matrix
%
	h = [  r   [0 0 0]'
	     0 0 0   1];
