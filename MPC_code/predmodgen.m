function predmod=predmodgen(sys,dim,para)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

%Prediction matrix from initial state
T=zeros(dim.nx*(dim.N+1),dim.nx);
for k=0:dim.N
    T(k*dim.nx+1:(k+1)*dim.nx,:)=sys.A^k;
end


S=zeros(dim.nx*(dim.N+1),dim.nu*(dim.N));
for k=1:dim.N
    for i=0:k-1
        S(k*dim.nx+1:(k+1)*dim.nx,i*dim.nu+1:(i+1)*dim.nu)=sys.A^(k-1-i)*sys.B;
    end
end

predmod.T=T;
predmod.S=S;

end