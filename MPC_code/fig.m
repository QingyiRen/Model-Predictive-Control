
%different Q
figure()
stairs(x_rec1(3,:));
hold on;
stairs(x_rec2(3,:));
hold on;
stairs(x_rec3(3,:));
legend('\alpha=0.1','\alpha=1','\alpha=10')
title('angular speed')

figure()
stairs(u_rec1(1,:));
hold on;
stairs(u_rec2(1,:));
hold on;
stairs(u_rec3(1,:));
legend('\alpha=0.1','\alpha=1','\alpha=10')
title('input(u_1)')

figure()
stairs(u_rec1(2,:));
hold on;
stairs(u_rec2(2,:));
hold on;
stairs(u_rec3(2,:));
legend('\alpha=0.1','\alpha=1','\alpha=10')
title('input(u_2)')

% %different N
% figure()
% stairs(x_rec4(3,:));
% hold on;
% stairs(x_rec5(3,:));
% hold on;
% stairs(x_rec6(3,:));
% legend('N=3','N=5','N=10')
% title('angular speed')
% 
% figure()
% stairs(u_rec4(1,:));
% hold on;
% stairs(u_rec5(1,:));
% hold on;
% stairs(u_rec6(1,:));
% legend('N=3','N=5','N=10')
% title('input(u_1)')
% 
% figure()
% stairs(u_rec4(2,:));
% hold on;
% stairs(u_rec5(2,:));
% hold on;
% stairs(u_rec6(2,:));
% legend('N=3','N=5','N=10')
% title('input(u_2)')

% %different starting point
% figure()
% stairs(x_rec7(3,:));
% hold on;
% stairs(x_rec8(3,:));
% hold on;
% stairs(x_rec9(3,:));
% legend('initial point 1','initial point 2','initial point 3')
% title('angular speed')
% 
% figure()
% stairs(u_rec7(1,:));
% hold on;
% stairs(u_rec8(1,:));
% hold on;
% stairs(u_rec9(1,:));
% legend('initial point 1','initial point 2','initial point 3')
% title('input(u_1)')
% 
% figure()
% stairs(u_rec7(2,:));
% hold on;
% stairs(u_rec8(2,:));
% hold on;
% stairs(u_rec9(2,:));
% legend('initial point 1','initial point 2','initial point 3')
% title('input(u_2)')
