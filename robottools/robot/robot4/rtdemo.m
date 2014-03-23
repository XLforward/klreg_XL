echo off
%RTDEMO		Robot toolbox demonstrations
%
%

%	Copyright (C) Peter Corke 1993
puma560
while 1,
	clg
	clc
	which = menu('Robot Toolbox demonstrations', ...
		'Transformations', ...
		'Trajectory', ...
		'Forward kinematics', ...
		'Animation', ...
		'Inverse kinematics', ...
		'Jacobians', ...
		'Inverse dynamics', ...
		'Forward dynamics', ...
		'Exit');

	if which == 1,
		rttrdemo
	elseif which == 2,
		rttgdemo
	elseif which == 3,
		rtfkdemo
	elseif which == 4,
		rtandemo
	elseif which == 5,
		rtikdemo
	elseif which == 6,
		rtjademo
	elseif which == 7,
		rtdydemo
	elseif which == 8,
		rtfddemo
		% chaotic 2 link
	elseif which == 9,
		break;
	end
end
