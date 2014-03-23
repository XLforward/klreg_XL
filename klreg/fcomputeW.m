function [W] = fcomputeW( X, Y,sigma2,outliers )
%
%this function is to compute the Weights matriX_
%   X is the fixed points,Y is the move points;
%   N is the points number of X_
%   M is the points number of Y
%   sigma2 is the covariance of sigma2
%   W is the Weights matriX_ of X_ and Y
%   W[i,j] = (-1/(2*sigma2))(eX_p{ -(X_[i]*X_[i]-y[j]*y[j])/(2*sigma2)}/sum£¨eX_p{ -(X_[i]*X_[i]-y[j]*y[j])£©/(2*sigma2)}£¬j = 1,...,M)

[N,D] = size(X);[M,D] = size(Y);
Wn=zeros(N,1);
W=zeros(M,N);
nos=(outliers*M*(2*3.1415926*sigma2)^(D/2))/((1-outliers)*N);
for n=1:1:N
    for m=1:1:M
        Wn(n)=Wn(n)+exp(-((X(n,:)-Y(m,:))*(X(n,:)-Y(m,:))')/(2*sigma2));
    end
    Wn(n)=(1-outliers)*Wn(n)+nos/M;
end

for n=1:1:N
    for m=1:1:M
        if(Wn(n)==0)
            W(m,n)=0;
        else
            i=exp(-((X(n,:)-Y(m,:))*(X(n,:)-Y(m,:))')/(2*sigma2));
            W(m,n)=i/(Wn(n));
        end
    end
end


end

