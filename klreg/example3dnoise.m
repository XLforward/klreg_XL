%example 3d bun with 360 noise;
load 3Dbuncor;
OX=X;
OY=Y;
figure,plot_iter(OX,OY); title('Before registering');
sumdis=0;
[N,D]=size(OX);
[M,D]=size(OY);
OM=M;
k=5000;
idx=randsample(N,k);
idy=randsample(M,k);
X=zeros(k,3);
Y=zeros(k,3);
for i=1:1:k
    X(i,:)=OX(idx(i),:);
    Y(i,:)=OY(idy(i),:);
end

[N,D]=size(X);
[M,D]=size(Y);

 for i=1:1:N
    for j=1:1:M
        sumdis=sumdis+(X(i,:)-Y(j,:))*(X(i,:)-Y(j,:))';
    end
 end
 sigma2=sumdis/(N*M*D);
 fprintf('before iter sigma2 = %.5f \n',sigma2);
 tic;
[T,R,t,X_,Y]=kl_register(X,Y,sigma2);
toc;
OY=OY*(T.R)+repmat(T.t,[OM,1]);
figure,plot_iter(OX,OY); title('after registering');
dif=OX-OY;
dnorm=0;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N;
fprintf('dis=%.9f\n',dnorm);
clear;
