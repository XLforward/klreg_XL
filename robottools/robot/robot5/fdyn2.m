
% private function called by fdyn()
%
% MOD HISTORY
% 4/99 add object support

%	Copyright (C) 1999 Peter Corke
function xd = fdyn2(t, x, flag, robot, torqfun)

	n = robot.n;

	q = x(1:n);
	qd = x(n+1:2*n);
	if isstr(torqfun)
		tau = feval(torqfun, t, q, qd);
	else
		tau = zeros(n,1);
	end
	
	qdd = accel(robot, x(1:n,1), x(n+1:2*n,1), tau);
	xd = [x(n+1:2*n,1); qdd];
