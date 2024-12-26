function constraints=constraintgen(dim,con)

constraints.A = [eye(dim.N*dim.nu);-1*eye(dim.N*dim.nu)];
constraints.b= [con.umin*ones(dim.N*dim.nu,1);-con.umax*ones(dim.N*dim.nu,1)];

end