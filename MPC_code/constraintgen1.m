function constraints=constraintgen1(dim,con,predmod,sys)


Mxy=kron(eye(dim.N+1),[1 0 0;0 1 0]);
constraints.A1 = [eye(dim.N*dim.nu);-1*eye(dim.N*dim.nu)];
constraints.b1= [con.umin*ones(dim.N*dim.nu,1);-con.umax*ones(dim.N*dim.nu,1)];
constraints.A2 = Mxy*predmod.S;
constraints.b2= con.imin*ones(2*dim.N+2,1)-Mxy*predmod.T*sys.x;
constraints.A3= -Mxy*predmod.S;
constraints.b3= -con.imax*ones(2*dim.N+2,1)+Mxy*predmod.T*sys.x;
end