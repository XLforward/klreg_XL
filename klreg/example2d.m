clear;
load 2Dfish;
[N,D]=size(X);
OX=X;
OY=Y;
figure1 = figure('Color',[1 1 1]);
plot_iter(X,Y); title('Before registering');
axis off;
 tic;
[T,R,t,X,Y]=kl_register(X,Y,80,0.0);
toc;
OY=OY*(T.R)+repmat(T.t,[N,1]);
figure1 = figure('Color',[1 1 1]);
plot_iter(OX,OY); title('our method');
axis off;
dif=X-Y;
dnorm=0;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N
% E_R=norm((R_)-R)
