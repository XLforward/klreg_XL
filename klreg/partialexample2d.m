clear;
load 2Dfishmissing25;
OX=X;
OY=Y;
%  figure,plot_iter(Y, Y); title('init');
%  sumdis=0;
%  [N,D]=size(X);
%  [M,D]=size(Y);
%  t=[0.1,0.0];
%  R_=eye(D);
% R_(2,2)=0.7071;
% R_(1,2)=-0.7071;
% R_(2,1)=0.7071;
% R_(1,1)=0.7071;
%  OX=OX*(R_)';
% X=OX+repmat(t,[N,1]);
 figure,plot_iter(OX,OY); title('Before registering');
 tic;
[T,R,t,X,Y]=kl_register(X,Y,80,0.2);
toc;

