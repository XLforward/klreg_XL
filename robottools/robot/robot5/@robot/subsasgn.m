%SUBSASGN	assignment methods on a ROBOT object
%
%	ROBOT.gravity = [gx gy gz]
%	ROBOT.base = 4x4 homog xform
%	ROBOT.name = 'name'
%	ROBOT.manuf = 'who built it'
%	ROBOT.comment = 'general comment'

%	Copyright (C) Peter Corke 1999

function r = subsasgn(r, s, v)

	if s(1).type  ~= '.'
		error('only .field supported')
	end
	switch s(1).subs,
	case 'gravity',
		r.gravity = v;
	case 'base',
		if ~ishomog(v)
			error('base must be a homogeneous transform');
		end
		r.base = v;
	case 'name',
		r.name = v;
	case 'manuf',
		r.manuf = v;
	case 'comment',
		r.comment = v;
	otherwise, error('Unknown method')
	end
