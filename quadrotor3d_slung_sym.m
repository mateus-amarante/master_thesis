
syms m M g L;
syms x y z alpha beta;
syms xdot ydot zdot alphadot betadot;
syms xcdot ycdot zcdot;
syms ux uy uz u1;

qdot = [xdot; ydot; zdot; alphadot; betadot];
qcdot = [xcdot; ycdot; zcdot; alphadot; betadot];

% U = [u1; u2; u3; u4];

% DYNAMICS: Mq*q_ddot + Cq*q_dot + Gq = bq*u

% Linear Dynamicsn
salpha = sin(alpha);
calpha = cos(alpha);
sbeta = sin(beta);
cbeta = cos(beta);

% m14 =  m*L*calpha*cbeta; m41 = m14;
% m15 = -m*L*salpha*sbeta; m51 = m15;
% m24 =  m*L*calpha*sbeta; m42 = m24;
% m25 =  m*L*salpha*cbeta; m52 = m25;
% m34 =  m*L*salpha; m43 = m34;
% 
% Mq = [
%     M+m  0    0    m14    m15;
%     0    M+m  0    m24    m25;
%     0    0    M+m  m34    0;
%     m41  m42  m43  m*L^2  0;
%     m51  m52  0    0,     m*L^2*salpha^2];
% 
% c14 = -m*L*(calpha*sbeta*betadot  + salpha*cbeta*alphadot);
% c15 = -m*L*(calpha*sbeta*alphadot + salpha*cbeta*betadot);
% c24 =  m*L*(calpha*cbeta*betadot  - salpha*sbeta*alphadot);
% c25 =  m*L*(calpha*cbeta*alphadot - salpha*sbeta*betadot);
% c34 =  m*L*calpha*alphadot;
% c45 = -m*L^2*salpha*calpha*betadot;
% c54 = -c45;
% c55 = m*L^2*salpha*calpha*alphadot;
% 
% Cq = [
%     0 0 0 c14 c15;
%     0 0 0 c24 c25;
%     0 0 0 c34 0;
%     0 0 0 0   c45;
%     0 0 0 c54 c55];
% 
% Gq = [0; 0; (M+m)*g; m*g*L*salpha; 0];
% 
% bq = [ux; uy; uz; 0; 0];
% 
% % Output
% qddot = collect(simplify(Mq\(-Cq*qdot -Gq + bq*u1)),u1);

%% Definição 2
m14 = -m*L*calpha*cbeta; m41 = m14;
m15 =  m*L*salpha*sbeta; m51 = m15;
% m24 =  m*L*calpha*sbeta; m42 = m24;
m25 =  m*L*cbeta; m52 = m25;
m34 =  m*L*salpha*cbeta; m43 = m34;
m35 =  m*L*calpha*sbeta; m53 = m35;

Mq = [
    M+m  0    0    m14            m15;
    0    M+m  0    0              m25;
    0    0    M+m  m34            0;
    m41  0    m43  m*L^2*cbeta^2  0;
    m51  m52  0    0,             m*L^2];

c14 =  m*L*(calpha*sbeta*betadot  + salpha*cbeta*alphadot);
c15 =  m*L*(calpha*sbeta*alphadot + salpha*cbeta*betadot);
% c24 =  m*L*(calpha*cbeta*betadot  - salpha*sbeta*alphadot);
c25 = -m*L*sbeta*betadot;
c34 =  m*L*(kcalpha*cbeta*alphadot  - salpha*sbeta*betadot);
c35 =  m*L*(calpha*cbeta*betadot   - salpha*sbeta*alphadot);
aux =  m*L^2*sbeta*cbeta;
c44 = -aux*betadot;
c45 = -aux*alphadot;
c54 = -c45;

Cq = [
    0 0 0 c14 c15;
    0 0 0 0   c25;
    0 0 0 c34 c35;
    0 0 0 c44 c45;
    0 0 0 c54 0];

Gq = [0; 0; (M+m)*g; m*g*L*salpha*cbeta; m*g*L*calpha*sbeta];

bq = [ux; uy; uz; 0; 0];

% Output
qddot = Mq\(-Cq*qdot -Gq + bq*u1);

