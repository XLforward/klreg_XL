%example 3d scanned faceclear;
load 3Dface;
[N,D]=size(X);
[M,D]=size(Y);

OX=X;
OY=Y;
figure,plot_iter(OX,OY); title('Before registering');
sumdis=0;
 for i=1:1:N
    for j=1:1:M
        sumdis=sumdis+(X(i,:)-Y(j,:))*(X(i,:)-Y(j,:))';
    end
 end
 sigma2=sumdis/(N*M*D);
 fprintf('before iter sigma2 = %.5f \n',sigma2);
 tic;
[T,R,t,X_,Y]=kl_register(X,Y,20,0);
toc;
OY=OY*(T.R)+repmat(T.t,[M,1]);
figure,plot_iter(OX,OY); title('after registering');
dif=OX-OY;
dnorm=0;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N;
fprintf('dis=%.9f\n',dnorm);
disp(dnorm);
clear;
