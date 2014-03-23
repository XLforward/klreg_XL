%SUBSREF	subscript reference methods on a ROBOT object
%
%	ROBOT.n			return number of links
%	ROBOT.link		return cell array of link objects
%	ROBOT.gravity 		return gravity vector
%	ROBOT.base 		return homog xform of robot base
%
%	ROBOT.name 		return name
%	ROBOT.manuf		return who built it
%	ROBOT.comment		return general comment
%
%	ROBOT.dh		return legacy DH matrix
%	ROBOT.dyn		return legacy DYN matrix

%	Copyright (C) Peter Corke 1999

function v = subsref(r, s)

	if s(1).type  ~= '.'
		%error('only .field supported')
	end

	% NOTE WELL:  the following code can't use getfield() since
	% getfield()  uses this, and Matlab will crash!!

	el = char(s(1).subs);
	switch el,
	case 'link',
		if length(s) == 1,
			v = r.link;
		elseif s(2).type == '{}'
			j = s(2).subs;
			j = j{1};
			if (j < 1) | (j > r.n)
				error('link index out of bounds')
			end
			v = r.link{j};
		end
	case 'n',
		v = r.n;
	case 'name',
		v = r.name;
	case 'dh',
		v = rdh(r);
	case 'dyn'
		v = rdyn(r);
	case 'gravity'
		v = r.gravity;
	case 'base'
		v = r.base;
	case 'mdh',
		v = r.mdh;
	case 'show',
		v = r.handle';
	otherwise, error('Unknown method')
	end
