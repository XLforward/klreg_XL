t=[0:.056:2]';
q = jtraj(qz, qr, t);
Ttg = fkine(p560, q);

figure(gcf)     % bring the figure to the top
axis([-256 256 -256 256])       % create 512 x 512 pixel image
grid
xlabel('X (pixels)')
ylabel('Y (pixels)')
set(gca, 'drawmode', 'fast');
h = line('XData', 0, 'YData', 0, ...
         'LineStyle', '*', 'EraseMode', 'xor');

f = 12e-3;              % camera focal length in m
alphax = 79.2e3         % pix/m for Pulnix TM-6 + Digimax
alphay = 120.5e3        % pix/m for Pulnix TM-6 + Digimax
% camera calibration matrix
C = [ alphax 0 0 0; 0 alphay 0 0; 0 0 1 0] * ...
    [ 1 0 0 0; 0 1 0 0; 0 0 -1/f 1; 0 0 0 1];

Tobj = transl(.5, 3, .5);       % location of object in world
xx = [];
yy = [];
for  tt = Ttg'
	T6 = reshape(tt', 4, 4);        % a trajectory point T6
        Tcam = T6 * rotx(-pi/2);        % camera transform

        Tobj_cam = inv(Tcam) * Tobj;    % object wrt camera

        x = C * Tobj_cam(:,4);          % camera transform
        X = x(1)/x(3); Y = x(2)/x(3);   % homogeneous coords

        set(h, 'xdata', X, 'ydata', Y); % move the marker
	xx = [xx X];
	yy = [yy Y];
        drawnow
end
