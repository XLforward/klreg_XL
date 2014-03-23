%example standford bunny
clear;
load bun045.txt;
load bun090.txt;
X=bun045';
Y=bun090';
[N,D]=size(X);
[M,D]=size(Y);
OX=X;
OY=Y;
k=5000;
idx=randsample(N,k);
idy=randsample(M,k);
X=zeros(k,3);
Y=zeros(k,3);
for i=1:1:k
    X(i,:)=OX(idx(i),:);
    Y(i,:)=OY(idy(i),:);
end
sumdis=0;
 
figure,plot_iter(OX,OY); title('Before registering');

 for i=1:1:k
    for j=1:1:k
        sumdis=sumdis+(X(i,:)-Y(j,:))*(X(i,:)-Y(j,:))';
    end
 end
 sigma2=sumdis/(k*k*D);
 fprintf('before iter sigma2 = %.5f \n',sigma2);
[T,R,t,X_,Y]=kl_register(X,Y,30,0.0);

OY=OY*(T.R)+repmat(T.t,[M,1]);
 figure,plot_iter(OX,OY); title('After registering');
 dlmwrite('after045X.txt',OX, 'delimiter', ' ', 'precision', '%.6f');
 dlmwrite('afterO90Y.txt',OY, 'delimiter', ' ', 'precision', '%.6f');
 disp(R);
 disp(t);
 clear;
