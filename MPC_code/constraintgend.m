function constraints=constraintgend(dime,con)

A=[];
for i=1:(dime.nu*dime.N-2)
    A=[A;zeros(1,i-1) 1 0 -1 zeros(1,dime.nu*dime.N-2-i)];
end
A=[A;-1.*A];
constraints.A=A;
constraints.b= -con.dumax*ones(size(A,1),1);

end