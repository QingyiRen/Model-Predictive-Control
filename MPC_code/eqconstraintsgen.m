function eqconstraints=eqconstraintsgen(sys,dim,para)

eqconstraints.A=[eye(dim.nx)-sys.A -sys.B; sys.C zeros(dim.ny,dim.nu)];
eqconstraints.b=[zeros(dim.nx,1); sys.yref];

end