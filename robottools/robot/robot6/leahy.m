%LEAHY	setup Leahy's standard paths for Puma 560 test
% 
puma560
%
% load as per Leahy89.  Experimental analysis of robot control: a performance
%  standard for the Puma-560.  M.B. Leahy. Proc. IEEE Symp.Intelligent Control
%
q0=[-50 -135 135 0 0 0]*pi/180;
q1=[50 -85 30 0 0 0]*pi/180;

%
% convert to joint angles as per Paul&Zhang
q0=q0-[pi -pi 0 0 0 0];
q1=q1-[pi -pi 0 0 0 0];
%
% path lasts for 1.5s
%
t=[0:.01:1.5]';
%
% make joint interp trajectory
%
[q,qd,qdd]=jtraj(q0,q1,t);

