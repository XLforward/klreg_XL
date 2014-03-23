clear;
load 2Dfish;
[N,D]=size(X);
t=[1.5,1.5];
X=X+repmat(t,[N,1]);
Y=X;
OX=X;
OY=Y;
%  figure,plot_iter(Y, Y); title('init');
 sumdis=0;
 [N,D]=size(X);
 [M,D]=size(Y);
 t=[0.1,0.0];
 R_=eye(D);
R_(2,2)=0.7071;
R_(1,2)=-0.7071;
R_(2,1)=0.7071;
R_(1,1)=0.7071;
 OX=OX*(R_)';
X=OX+repmat(t,[N,1]);
 figure,plot_iter(OX,OY); title('Before registering');
 for i=1:1:N
    for j=1:1:M
        sumdis=sumdis+(X(i,:)-Y(j,:))*(X(i,:)-Y(j,:))';
    end
 end
 sigma2=sumdis/(N*M*D);
 fprintf('before iter sigma2 = %.5f \n',sigma2);
 tic;
[T,R,t,X,Y]=kl_register(X,Y,sigma2,15,0.0);
toc;
dif=X-Y;
dnorm=0;
for i=1:1:N
    dnorm=dnorm+norm(dif(i,:),2);
end
dnorm=dnorm/N;
disp(dnorm);

