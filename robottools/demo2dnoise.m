clear;
load 2Dfishnoise7;
[N,D]=size(X);
 figure,plot_iter(X,Y); title('Before registering');
 opt.maxiter=50;
 opt.dplot=0;
tic;
[A,B,T,d]=icp(X', Y',opt);
toc;
figure,plot_iter(A' ,B',opt); title('after registering');
% fprintf('dis=%.6f\n',d);
dif=(A')-(B');
dnorm=0;
N=91;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N

