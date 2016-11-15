
n = 11;
xl = [3458 3446 3424 3356 3201 2866 1944 1461 1053 844 584];
xr = [3408 3403 3355 3195 1792 740  515  343  242  140 111];
n_div_l = 4;
n_div_r = 4;
x = xl;
n_div = n_div_l;
y = 0:10:(n-1)*10;

plot(x,y,'o--');
hold on

xx1 = linspace(x(1),x(n_div),20);
yy1 = spline(x,y,xx1);
p1 = polyfit(xx1,yy1,3)

xx2 = linspace(x(n_div-1),x(n),20);
yy2 = spline(x,y,xx2);
p2 = polyfit(xx2,yy2,3)

xx1 = linspace(x(1),x(n_div));
xx2 = linspace(x(n_div),x(n));

plot(xx1,polyval(p1,xx1),xx2,polyval(p2,xx2));
legend('pomiar','p1', 'p2')

