% JAC560
%
% Jacobian of Puma560
%
function jac = jac560(theta)

  if length(theta) < 6,
	error('theta vector too short')
  end

%
% D&H params
%
  lscale = 1000.0;	
  A2 = lscale * 0.43180;
  D3 = lscale * 0.128778;
  A3 = lscale * 0.020320;
  D4 = lscale * 0.433070;

%
% trig values
%
  s1 = sin(theta(1)); c1 = cos(theta(1));
  s2 = sin(theta(2)); c2 = cos(theta(2));
  s3 = sin(theta(3)); c3 = cos(theta(3));
  s4 = sin(theta(4)); c4 = cos(theta(4));
  s5 = sin(theta(5)); c5 = cos(theta(5));
  s6 = sin(theta(6)); c6 = cos(theta(6));
  s23 = sin(theta(2)+theta(3)); c23 = cos(theta(2)+theta(3));

  u512 = -c5 * s6;
  u522 = -s5 * s6;
  u412 = c4 * u512 - s4 * c6;
  u422 = s4 * u512 + c4 * c6;
  u413 = -c4 * s5;
  u423 = -s4 * s5;
  u212 = c23 * u412 - s23 * u522;
  u222 = s23 * u412 + c23 * u522;
  u213 = c23 * u413 - s23 * c5;
  u223 = s23 * u413 + c23 * c5;

  u521 = s5 * c6;	
  u511 = c5 * c6;	
  u421 = s4 * u511 + c4 * s6;
  u411 = c4 * u511 - s4 * s6;
  u323 = s3 * u413 + c3 * c5;
  u322 = s3 * u412 + c3 * u522;
  u321 = s3 * u411 + c3 * u521;
  u211 = c23 * u411 - s23 * u521;

  u214 = -s23 * D4 + A3 * c23 + A2 * c2;

  jac = zeros(6,6);
% row 1
  jac(1,1) = u211 * D3 + u421 * u214;
  jac(1,2) = u321 * A2;
  jac(1,3) = - u411 * D4 + u521 * A3;
% row 2
  jac(2,1) = u212 * D3 + u422 * u214;
  jac(2,2) = u322 * A2;
  jac(2,3) = - u412 * D4 + u522 * A3;
% row 3
  jac(3,1) = u213 * D3 + u423 * u214;
  jac(3,2) = u323 * A2;
  jac(3,3) = - u413 * D4 + c5 * A3;
% row 4
  jac(4,1) = s23 * u411 + c23 * u521;
  jac(4,2) = 0.0;
  jac(4,3) = - u421;
  jac(4,4) = u521;
  jac(4,5) = - s6;
  jac(4,6) = 0.0;
% row 5
  jac(5,1) = u222; 
  jac(5,2) = 0.0;
  jac(5,3) = - u422;
  jac(5,4) = u522;
  jac(5,5) = - c6;
  jac(5,6) = 0.0;
% row 6
  jac(6,1) = u223; 
  jac(6,2) = 0.0;
  jac(6,3) = - u423;
  jac(6,4) = c5;
  jac(6,5) = 0.0;
  jac(6,6) = 1.0;

%
% now add column 3 to column 2, since Lou's solution is for th23, not th3
%
 jac(:,2) = jac(:,2) + jac(:,3);
