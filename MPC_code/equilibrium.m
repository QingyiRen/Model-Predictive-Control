function equilibrium=equilibrium(para,ucon)

Ld=para.Ld ;
Lq=para.Lq;
R=para.R;
p=para.p;
B=para.B;
mg=para.mg;
Kt=para.Kt;

syms x1 x2 x3
eqns = [ucon(1)-R*x1+x3*Lq*x2==0, ucon(2)-R*x2-x3*Ld*x1-x3*mg==0, Kt*x2-B/p*x3==0];
sol = solve(eqns, [x1, x2,x3]);
solx1 = double(sol.x1);
solx2 = double(sol.x2);
solx3 = double(sol.x3);

equilibrium=[solx1(solx1 > 0);solx2(solx2 > 0);solx3(solx3 > 0)];

end