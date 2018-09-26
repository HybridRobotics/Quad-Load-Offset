%% Offset Model: slow model control
function[dx, xLd, Rd, qd, f, M] = odefun_control(t,x,data)

global R_;
global Omega_;
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

%% Get Desired States
[xLd,vLd,aLd,~,dqd,d2qd,Rd,omegad,domegad,Omegad,dOmegad] = ...
get_nom_traj2(data.params, get_load_traj3(t));

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

%% Load Position Control

% Position errors
err_x = xL - xLd;
err_v = vL - vLd;

epsilon_bar = 0.8; % 0.8
kp_xy = 0.3/epsilon_bar^2; kd_xy = 0.6/epsilon_bar;
kx = diag([kp_xy kp_xy 2]); kv = diag([kd_xy kd_xy 1.5]);

%% Quadrotor Coupled Dynamics
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

err_R = 1/2 * vee_map(Rd'*R - R'*Rd);
err_Om = Omega - R'*Rd*Omegad;
kR = 4; kOm = 4;
epsilon = 0.5 ; %.5 ; %0.01 ;
kR = 4/epsilon^2; kOm = 4/epsilon;

Wd1 = aLd+g*e3-kx*err_x-kv*err_v;
Wd2 = -hat_map(Omega)*R'*Rd*Omegad+R'*Rd*dOmegad-kR*err_R-kOm*err_Om;

%% PD force to track trajectory for Load with Feedforward
u_para = vec_dot((A11*Wd1+A12*Wd2-d1)/norm(G1),q)*q;
qd = -(A11*Wd1+A12*Wd2-d1)/norm(A11*Wd1+A12*Wd2-d1);

epsilon_q = 0.05;
kq = -1.5/epsilon_q^2; kom = -0.8/epsilon_q;
err_q = hat_map(q)^2*qd;
err_om = dq - vec_cross(vec_cross(qd, dqd), q);
u_perp = -kq*err_q-kom*err_om - vec_dot(q, vec_cross(qd,dqd))*vec_cross(q,dq) -...
    vec_cross( vec_cross(qd, d2qd), q)+(1/l)*hat(q)*hat(q)*(R*(hat(Omega)*hat(Omega)+dOmegad)*r);

%% Input f
v = u_para+u_perp;
f = vec_dot(v, R(:,3))*mQ*l;
b3c = v/norm(v);

% DESIRED YAW DIRECTION
b1d = e1;
b1c = -vec_cross(b3c,vec_cross(b3c,b1d))/norm(vec_cross(b3c,b1d));
Rc = [b1c, vec_cross(b3c,b1c),b3c];
Rd = Rc;

if(norm(Rd'*Rd-eye(3)) > 1e-2)
    disp('Error in R'); keyboard;
end
err_R = 1/2 * vee_map(Rd'*R - R'*Rd);
err_Om = Omega - R'*Rd*Omegad;
kR = 4; kOm = 4;
epsilon = 0.1 ; %.5 ; %0.01 ;
kR = 4/epsilon^2; kOm = 4/epsilon;


%% Update Dynamics
Wd1 = aLd+g*e3-kx*err_x-kv*err_v;
Wd2 = -hat_map(Omega)*R'*Rd*Omegad+R'*Rd*dOmegad-kR*err_R-kOm*err_Om;

%% Input M
M = vec_cross(Omega, J*Omega)+A21*Wd1+A22*Wd2-G2*u_para-d2;

%% Dynamics
temp = A\(G*u_para+d+[zeros(3,1);M-vec_cross(Omega, J*Omega)]);
xL_dot = vL;
vL_dot = temp(1:3) - g*e3;
q_dot = dq;
R_dot = R*hat_map(Omega);
Omega_dot = temp(4:6);
omega_dot = -hat(q)*u_perp+vec_cross(q,(1/l)*...
    (vL_dot+g*e3+R*(hat(Omega)*hat(Omega)+hat(Omega_dot))*r));

R_ = R;
Omega_ = Omega;
%% Output
dx = [xL_dot; vL_dot; q_dot; omega_dot; reshape(R_dot, 9,1); Omega_dot];

if nargout <= 1
   fprintf('Simulation time %0.4f seconds \n',t);
end

end