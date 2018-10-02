%% Zero offset geometric controller applied to the offset model  
function[dx, xLd, Rd, qd, f, M] = odefun_control2(t,x,data)

%% Constants
mL = data.params.mL;
g = data.params.g;
mQ = data.params.mQ;
J = data.params.J;
e1 = data.params.e1;
e2 = data.params.e2;
e3 = data.params.e3;
l = data.params.l;
r = data.params.r;

%% Desired States
%---------------%

% Case 1: Testing
[xLd,vLd,aLd,~,dqd,d2qd,~,Omegad,dOmegad] = get_nom_traj(data.params, get_load_traj3(t));

%% Extracting States
xL = x(1:3);
vL = x(4:6);
q = x(7:9);
omega = x(10:12);
dq = vec_cross(omega, q);
R = reshape(x(13:21), 3,3);
Omega = x(22:24);
b3 = R(:,3);
b1 = R(:,1);

% LOAD POSITION TRACKING

% Position errors
err_x = xL - xLd;
err_v = vL - vLd;

epsilon_bar = 0.8;
kp_xy = 0.3/epsilon_bar^2; kd_xy = 0.6/epsilon_bar;
k1 = diag([kp_xy kp_xy 2]); k2 = diag([kd_xy kd_xy 1.5]);

% PD force to track trajectory for Load with
% feedforward
A = (-k1*err_x - k2*err_v + (mQ+mL)*(aLd+g*e3) + mQ*l*vec_dot(dq,dq)*q);
qd = -A/norm(A);

epsilon_q = 0.5;
kp = -1.5/epsilon_q^2; kom = -0.8/epsilon_q;
err_q = hat_map(q)^2*qd;
err_om = dq - vec_cross(vec_cross(qd, dqd), q);

F_pd = -kp*err_q-kom*err_om;
F_ff = (mQ*l)*vec_dot(q, vec_cross(qd,dqd))*vec_cross(q,dq)+...
    (mQ*l)*vec_cross( vec_cross(qd, d2qd), q);
F_n = vec_dot(A,q)*q;

F = F_pd - F_ff + F_n;

b3c = F/norm(F);

f = vec_dot(F, R(:,3));

% Load position
xL_dot = vL;

if(abs(norm(qd)-1) > 1e-2)
    disp('Error in pd'); keyboard;
end

% Load Attitude
q_dot = dq;

% DESIRED YAW DIRECTION
b1d = e1;
b1c = -vec_cross(b3c,vec_cross(b3c,b1d))/norm(vec_cross(b3c,b1d));
Rc = [b1c, vec_cross(b3c,b1c),b3c];
Rd = Rc;
if(norm(Rd'*Rd-eye(3)) > 1e-2)
    disp('Error in R'); keyboard;
end
kR = 4 ; kOm = 4 ;
epsilon = 0.1 ; %.5 ; %0.01 ;

err_R = 1/2 * vee_map(Rd'*R - R'*Rd);
err_Om = Omega - R'*Rd*Omegad;
M = -kR/epsilon^2*err_R - kOm/epsilon*err_Om + vec_cross(Omega, J*Omega)...
    - J*(hat_map(Omega)*R'*Rd*Omegad - R'*Rd*dOmegad);

% Quadrotor Attitude
R_dot = R*hat_map(Omega);

%% Apply zero offset geometric controller to the offset dynamical model
u = vec_dot(f, R(:,3))/(mQ*l);
u_para = q*(q'*u);
u_perp = -hat(q)*(hat(q)*u);

qb = R'*q;
A11 = (mQ+mL)*eye(3);
A12 = mQ*q*qb'*hat(r);
A21 = -mQ*hat(r)*qb*q';
A22 = J + mQ*(hat(r)*qb)*(hat(r)*qb)';
A = [A11,A12;A21,A22];

G1 = mQ*l*(q*q');
G2 = -mQ*l*hat(r)*qb*q';
G = [G1;G2];

d1 = mQ*(qb'*(hat(Omega))^2*r - l*vec_dot(omega,omega))*q;
d2 = -mQ*(qb'*(hat(Omega))^2*r - l*vec_dot(omega,omega))*(hat(r)*qb);
d = [d1;d2];

temp = A\(G*u_para+d+[zeros(3,1);M-vec_cross(Omega, J*Omega)]);
vL_dot = temp(1:3) - g*e3;
Omega_dot = temp(4:6);
omega_dot = -hat(q)*u_perp+vec_cross(q,(1/l)*...
    (vL_dot+g*e3+R*(hat(Omega)^2+hat(Omega_dot))*r));

%% Output
dx = [xL_dot; vL_dot; q_dot; omega_dot; reshape(R_dot, 9,1); Omega_dot];
if nargout <= 1
   fprintf('Simulation time %0.4f seconds \n',t);
end

end