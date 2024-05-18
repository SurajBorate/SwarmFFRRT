waypoints = [0,0;2,1;3,2;3.5,5;2,4.5;-1,8];
t0 = 0;
tf = 5;
V = 1;
dt = 0.01;
nn = size(waypoints);
n = nn(1);
x = waypoints(:,1);
delx = diff(x);
y = waypoints(:,2);
dely = diff(y);
plot(x,y,'o');
O = zeros(n-1,1);
D = zeros(n-1,1);
delT = zeros(n-1,1);
T = zeros(n-1,1);
X0 = zeros(n-1,1);
Xf = zeros(n-1,1);
Y0 = zeros(n-1,1);
Yf = zeros(n-1,1);
Vx0 = zeros(n-1,1);
Vxf = zeros(n-1,1);
Px = zeros(n-1,4);
Py = zeros(n-1,4);
for i = 1:(n-1)
o=atan2(dely(i),delx(i));
O(i) = o;
D(i) = delx(i)^2+dely(i)^2;
end
for i = 1:(n-1)
delT(i) = (tf-t0)*(D(i)/sum(D));
T(i) = t0+sum(delT(1:i));
end
T = [0;T];
Vx = V*cos(O);
Vx0 = Vx;
Vxf = [Vx(2:n-1);0];
Vy = V*sin(O);
Vy0 = Vy;
Vyf = [Vy(2:n-1);0];
T0 = T;
Tf = [T(2:n-1);tf];
X0 = x(1:n-1);
Xf = x(2:n);
Y0 = y(1:n-1);
Yf = y(2:n);
for i = 1:n-1
    px = cubic(X0(i),Xf(i),Vx0(i),Vxf(i),T0(i),Tf(i));
    Px(i,:) = px;
    py = cubic(Y0(i),Yf(i),Vy0(i),Vyf(i),T0(i),Tf(i));
    Py(i,:) = py;
end
for i=1:n-1
    t=T0(i):dt:Tf(i);
    Xtraj = Px(i,1)+Px(i,2)*t+Px(i,3)*t.^2+Px(i,4)*t.^3;
    Ytraj = Py(i,1)+Py(i,2)*t+Py(i,3)*t.^2+Py(i,4)*t.^3;
    hold on
    plot(Xtraj,Ytraj)
    
end

function p = cubic(x0,x1,v0,v1,t0,tf)
A = [1 t0 t0^2 t0^3;0 1 2*t0 3*t0^2;1 tf tf^2 tf^3;0 1 2*tf 3*tf^2];
p = A\[x0;v0;x1;v1];
end

