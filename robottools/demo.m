% Example::
% Create a 3D point cloud
%          p1 = randn(3,200);      
%          
% % Transform it by an arbitrary amount
% t=[1.0,0.0,0]';
%  R_=eye(3);
% R_(2,2)=0.7071;
% R_(1,2)=-0.7071;
% R_(2,1)=0.7071;
% R_(1,1)=0.7071;
%          p2 = (R_)*p1+repmat(t,[1,200]);
% % Perform ICP to determine the transformation that maps p1 to p2
%example 3d scanned faceclear;
load 3Dface;
[N,D]=size(X);
[M,D]=size(Y);
% R_=eye(D);
% R_(2,2)=cos(45);
% R_(2,3)=-sin(45);
% R_(3,2)=sin(45);
% R_(3,3)=cos(45);
% X=X*(R_)';
OX=X;
OY=Y;

tic;
[A,B,T,d]=icp(X', Y');
 plot3(A(1,:),A(2,:),A(3,:),'bx');
grid
hold on
plot3(B(1,:),B(2,:),B(3,:),'ro');
% fprintf('dis=%.6f\n',d);
dif=(A')-(B');
dnorm=0;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N;
disp(dnorm);
toc;
