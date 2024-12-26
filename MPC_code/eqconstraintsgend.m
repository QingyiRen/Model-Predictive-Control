function eqconstraints=eqconstraintsgend(sys,dim,dhat)

eqconstraints.A=[eye(dim.nx)-sys.A -sys.B; sys.C zeros(dim.ny,dim.nu)];
eqconstraints.b=[sys.Bd*dhat; sys.yrefd-sys.Cd*dhat];

end