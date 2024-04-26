%% *Dynamics of a double rotary Pendulum.*
% *Calculate all positions.*

syms theta1(t) theta2(t) theta3(t);
syms L1 L2 L3 M1 M2 M3 % Symbollic for now.
syms g;
% L1 = 2;
% L2 = 1;
% L3 = 1;
% M1 = 1;
% M2 = 1;
% M3 = 1;
x1 = L1*cos(theta1(t))/2;
y1 = L1*sin(theta1(t))/2;
z1 = 0;
x2 = L1*cos(theta1(t)) + L2*sin(theta1(t))*sin(theta2(t))/2;
y2 = L1*sin(theta1(t)) + L2*-cos(theta1(t))*sin(theta2(t))/2;
z2 = L2*cos(theta2(t))/2;
x3 = L1*cos(theta1(t)) + L2*sin(theta1(t))*sin(theta2(t)) + L3*sin(theta1(t))*cos(theta2(t))*sin(theta3(t))/2 + L3*sin(theta1(t))*sin(theta2(t))*cos(theta3(t))/2;
y3 = L1*sin(theta1(t)) + L2*-cos(theta1(t))*sin(theta2(t)) + L3*-cos(theta1(t))*cos(theta2(t))*sin(theta3(t))/2 + L3*-cos(theta1(t))*sin(theta2(t))*cos(theta3(t))/2;
z3 = L2*cos(theta2(t)) - L3*sin(theta2(t))*sin(theta3(t))/2 + L3*cos(theta2(t))*cos(theta3(t))/2;
% *Validation of position system.*

% syms t;
% Testx1(t) = subs(x1,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% Testy1(t) = subs(y1,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% Testz1(t) = subs(z1,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% 
% 
% Testx2(t) = subs(x2,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% Testy2(t) = subs(y2,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% Testz2(t) = subs(z2,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% 
% Testx3(t) = subs(x3,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% Testy3(t) = subs(y3,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% Testz3(t) = subs(z3,[L1,L2,L3,theta1,theta2,theta3],[1,1,1,t,t,t]);
% 
% fplot3(Testx3,Testy3,Testz3,[-3*pi,3*pi]);
% xlim([-2,2]);
% ylim([-2,2]);
% hold on;
% fplot3(Testx1,Testy1,Testz1,[-3*pi,3*pi]);
% fplot3(Testx2,Testy2,Testz2,[-3*pi,3*pi]);
% hold off;
% Take Derivates.

%syms J1 J2 J3
J1 = (M1*L1^2)/12;
J2 = (M2*L2^2)/12;
J3 = (M3*L3^2)/12;
dx1dt = simplify(diff(x1));
dy1dt = simplify(diff(y1));
dz1dt = 0;
dx2dt = simplify(diff(x2));
dy2dt = simplify(diff(y2));
dz2dt = simplify(diff(z2));
dx3dt = simplify(diff(x3));
dy3dt = simplify(diff(y3));
dz3dt = simplify(diff(z3));
% *Compute Lagrangian.*



LT = simplify((M1*(dx1dt^2+dy1dt^2) + M2*(dx2dt^2+dy2dt^2+dz2dt^2) + M3*(dx3dt^2+dy3dt^2+dz3dt^2) +  J1*diff(theta1)^2 +  J2*diff(theta2)^2 +  J3*diff(theta3)^2)/2,"Steps",100);
LV = M1*g*z1 + M2*g*z2 + M3*g*z3;
L = simplify(LT - LV);
% *Compute Euler Equations of Motion*

syms Tau1(t);
InputTorque(t) = 2*(heaviside(t) - heaviside(t-0.5));
syms C1 C2 C3;
D1 = C1 * diff(theta1); % Damping 1 Co-effecient
D2 = C2 * diff(theta2); % Damping 2 Co-effecient
D3 = C3 * diff(theta3); % Damping 3 Co-effecient
EOM1 = simplify(diff(diff(L,diff(theta1)),t)-diff(L,theta1),"Steps",100)==Tau1;
EOM2 = simplify(diff(diff(L,diff(theta2)),t)-diff(L,theta2),"Steps",100)==0*t;
EOM3 = simplify(diff(diff(L,diff(theta3)),t)-diff(L,theta3),"Steps",100)==0*t;
% *ODE from EoM*

PDE1 = subs(EOM1,[L1,L2,L3,M1,M2,M3,g,],[1,1,1,1,1,1,9.80665]);
PDE2 = subs(EOM2,[L1,L2,L3,M1,M2,M3,g,],[1,1,1,1,1,1,9.80665]);
PDE3 = subs(EOM3,[L1,L2,L3,M1,M2,M3,g,],[1,1,1,1,1,1,9.80665]);
[V,S] = odeToVectorField(PDE1,PDE2,PDE3);
% *MatlabFunction-Ify ODE's*

M = matlabFunction(V,'Vars',{'t','Y'});
initCond = [0 0 0 0 0 0];
options = odeset('RelTol',1e-9,'AbsTol',1e-9);
sols = ode45(M,[0 20],initCond,options);
% z3 = -L2*cos(theta2)-(L3*cos(theta3)/2);

% S =
%  theta2
% Dtheta2
%  theta1
% Dtheta1
%  theta3
% Dtheta3

Delta = 1/30;
Time = 0:Delta:20;

SimTheta1 = deval(sols,Time,3);
SimTheta2 = deval(sols,Time,1);
SimTheta3 = deval(sols,Time,5);

Simx1 =-cos(SimTheta1);
Simy1 = sin(SimTheta1);
Simz1 = 0.*(SimTheta1);

Simx2 = cos(SimTheta1)+(sin(SimTheta1).*sin(SimTheta2));
Simy2 = sin(SimTheta1)+(cos(SimTheta1).*sin(SimTheta2));
Simz2 = cos(SimTheta2);

Simx3 = cos(SimTheta1)+(sin(SimTheta1).*(sin(SimTheta2) + sin(SimTheta1).*cos(SimTheta2).*sin(SimTheta3) + sin(SimTheta1).*sin(SimTheta2).*cos(SimTheta3)));
Simy3 = sin(SimTheta1)+(cos(SimTheta1).*(sin(SimTheta2) + -cos(SimTheta1).*cos(SimTheta2).*sin(SimTheta3) + -cos(SimTheta1).*sin(SimTheta2).*cos(SimTheta3)));
Simz3 = cos(SimTheta2)-(sin(SimTheta2).*cos(SimTheta3)) + cos(SimTheta2).*cos(SimTheta3);


x1 = L1*cos(theta1(t))/2;
y1 = L1*sin(theta1(t))/2;
z1 = 0;
x2 = L1*cos(theta1(t)) + L2*sin(theta1(t))*sin(theta2(t))/2;
y2 = L1*sin(theta1(t)) + L2*-cos(theta1(t))*sin(theta2(t))/2;
z2 = L2*cos(theta2(t))/2;
x3 = L1*cos(theta1(t)) + L2*sin(theta1(t))*sin(theta2(t)) + L3*sin(theta1(t))*cos(theta2(t))*sin(theta3(t))/2 + L3*sin(theta1(t))*sin(theta2(t))*cos(theta3(t))/2;
y3 = L1*sin(theta1(t)) + L2*-cos(theta1(t))*sin(theta2(t)) + L3*-cos(theta1(t))*cos(theta2(t))*sin(theta3(t))/2 + L3*-cos(theta1(t))*sin(theta2(t))*cos(theta3(t))/2;
z3 = L2*cos(theta2(t)) - L3*sin(theta2(t))*sin(theta3(t))/2 + L3*cos(theta2(t))*cos(theta3(t))/2;

v=VideoWriter("Week22Animation1");
open(v);


formatSpec = '%.3f';

Fig = figure(2);
set(Fig,'position',[0,0,1080,540]);

for x=1:length(Time)


    % Link 3
    scatter3(Simx3(x),Simy3(x),Simz3(x));
    xlim([-2,2])
    ylim([-2,2])
    zlim([-2,2])
    hold on
   
    % Link 3 to 2 line
    plot3([Simx3(x),Simx2(x)],[Simy3(x),Simy2(x)],[Simz3(x),Simz2(x)])
   
    % Link 2
    scatter3(Simx2(x),Simy2(x),Simz2(x));

    % Link 2 to 1 line
    plot3([Simx1(x),Simx2(x)],[Simy1(x),Simy2(x)],[Simz1(x),Simz2(x)])

    % Link 1
    scatter3(Simx1(x),Simy1(x),Simz1(x));

    % Link 1 to 0 line
    plot3([Simx1(x),0],[Simy1(x),0],[Simz1(x),0])
    
    % Link 0 (origin)
    scatter3(0,0,0);

    set(gcf,"color",[0,0,0.1])

    title("Time: "+num2str(Time(x),formatSpec),"Torque: "+num2str(double(InputTorque(Time(x))),formatSpec));

    grid on
    hold off
            
    frame = getframe(gcf);
    writeVideo(v,frame);

    

end

close(v);
