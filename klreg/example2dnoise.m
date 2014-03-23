clear;
load 2Dfishmissnoise;
[N,D]=size(X);
OX=X;
OY=Y;
 figure,plot_iter(X,Y); title('Before registering');
 
 tic;
[T,R,t,X,Y]=kl_register(X,Y,30,0.9);
toc;

OY=OY*(T.R)+repmat(T.t,[N,1]);
figure,plot_iter(OX,OY); title('after registering');
dif=X-Y;
dnorm=0;
N=91;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N