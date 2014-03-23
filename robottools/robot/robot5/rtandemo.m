%**************************animation********************************************
echo on
%
% The trajectory demonstration has shown how a joint coordinate trajectory
% may be generated
	t = [0:.056:2]';		% generate a time vector
	q = jtraj(qz, qr, t);	% generate joint coordinate trajectory
%
% the function plotbot() animates a stick figure robot moving along a trajectory.

	plot(p560, q);
% The drawn line segments do not necessarily correspond to robot links, but join 
% the origins of sequential link coordinate frames.
%
% A small right-angle coordinate frame is drawn on the end of the robot to show
% the wrist orientation.  The axes X, Y and Z are represented by colors red, green
% and blue respectively.

pause	% any key to continue

% There are a number of options, one of which is to inhibit erasure, which leaves
% a trail of robots behind
	plot(p560, q, 'l')
pause	% any key to continue
echo off
