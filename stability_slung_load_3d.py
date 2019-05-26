# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:light
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.4'
#       jupytext_version: 1.1.3
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# # Stability Analysis of the SMC for the Slung Load System 3D
# ## Common Setup

# +
from sympy import *
from sympy.physics.mechanics import dynamicsymbols, init_vprinting

init_vprinting()

# Time and system variables
t, y1, y2, y3, y4= symbols('t y_1 y_2 y_3 y_4')

# Control parameters
lambda1, lambda2, lambda3, lambda4= symbols('\tlambda_1 \tlambda_2 \tlambda_3 \tlambda_4')
lambda_z, lambda_psi= symbols('\tlambda_z \lambda_\psi')


# Physical parameters
m, M, l, g, Ix, Iy, Iz = symbols('m M l g I_x I_y I_z')

# +
# STATE VARIABLES

# Robot position
x, y, z = dynamicsymbols('x y z')
x_d, y_d, z_d = dynamicsymbols('x_d y_d z_d')

xdot = diff(x,t)
xddot = diff(xdot,t)
xdot_d = diff(x_d,t)
xddot_d = diff(xdot_d,t)

ydot = diff(y,t)
yddot = diff(ydot,t)
ydot_d = diff(y_d,t)
yddot_d = diff(ydot_d,t)

zdot = diff(z,t)
zddot = diff(zdot,t)
zdot_d = diff(z_d,t)
zddot_d = diff(zdot_d,t)

# Robot orientation
phi, theta, psi = dynamicsymbols('\tphi \ttheta \tpsi')
phi_d, theta_d, psi_d = dynamicsymbols('\tphi_d \ttheta_d \tpsi_d')

phidot = diff(phi, t)
phiddot = diff(phidot, t)
phidot_d = diff(phi_d, t)
phiddot_d = diff(phidot_d, t)

thetadot = diff(theta, t)
thetaddot = diff(thetadot, t)
thetadot_d = diff(theta_d, t)
thetaddot_d = diff(thetadot_d, t)

psidot = diff(psi, t)
psiddot = diff(psidot, t)
psidot_d = diff(psi_d, t)
psiddot_d = diff(psidot_d, t)

# Load angles
phiL, thetaL = dynamicsymbols('\tphi_L \ttheta_L')
phiL_d, thetaL_d =  dynamicsymbols('\tphi_L^d \ttheta_L^d')

phiLdot = diff(phiL, t)
phiLddot = diff(phiLdot, t)
phiLdot_d = diff(phiL_d, t)
phiLddot_d = diff(phiLdot_d, t)

thetaLdot= diff(thetaL, t)
thetaLddot = diff(thetaLdot, t)
thetaLdot_d= diff(thetaL_d, t)
thetaLddot_d = diff(thetaLdot_d, t)

# +
# TRANSLATION DYNAMICS

# Auxiliary variables
sphiL = sin(phiL); sthetaL = sin(thetaL);
cphiL = cos(phiL); cthetaL = cos(thetaL);

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
uz = cos(phi)*cos(theta);

# Auxiliary constants
# Cf = m*l/(M+m)*(cthetaL**2*phiLdot**2 + thetaLdot**2);
# Cb = m/(M*(M + m));
Cf, Cx, Cy, Cz = symbols('C_f C_x C_y C_z')
# Cx = (M/m + cthetaL**2)
# Cy = (M/m + 1 - sphiL**2*cthetaL**2)
# Cz = (M/m + 1 - cphiL**2*cthetaL**2)

# System functions
fx = -Cf*sthetaL;
# bx = Cb*(sthetaL*cthetaL*(uy*sphiL - uz*cphiL) + ux*(M/m + cthetaL**2));
bx = (sthetaL*cthetaL*(uy*sphiL - uz*cphiL) + ux*Cx);


fy = Cf*sphiL*cthetaL;
# by = Cb*(sphiL*cthetaL*(ux*sthetaL + uz*cphiL*cthetaL) + uy*(M/m + 1 - sphiL**2*cthetaL**2));
by = (sphiL*cthetaL*(ux*sthetaL + uz*cphiL*cthetaL) + uy*Cy);

fz = -Cf*cphiL*cthetaL - g;
# bz = Cb*(cphiL*cthetaL*(-ux*sthetaL + uy*sphiL*cthetaL) + uz*(M/m + 1 - cphiL**2*cthetaL**2));
bz = (cphiL*cthetaL*(-ux*sthetaL + uy*sphiL*cthetaL) + uz*Cz);

fphiL = 2*tan(thetaL)*phiLdot*thetaLdot;
bphiL = -(uy*cphiL + uz*sphiL)/(M*l*cthetaL);

fthetaL = -sthetaL*cthetaL*phiLdot**2;
bthetaL = (ux*cthetaL + sthetaL*(uy*sphiL - uz*cphiL))/(M*l);

# +
# ROTATION DYNAMICS
fphi = thetadot*psidot*(Iy - Iz)/Ix
bphi = 1/Ix

ftheta = phidot*psidot*(Iz - Ix)/Iy
btheta = 1/Iy

fpsi = phidot*thetadot*(Ix - Iy)/Iz
bpsi = 1/Iz
# -

# FULLY ACTUATED SUBSYSTEM CONTROL
u1n = symbols('u_1_n')
# u1 = u1n/bz
u1 = u1n
# u1 = (zddot_d - fz + lambda_z*(zdot_d - zdot))/bz
# u4 = (psiddot_d - fpsi + lambda_psi*(psidot_d - psidot))/bpsi
u1

# ## $x$ and $\theta$

# +
bx = bx.subs({theta: (theta_d-y1), thetadot: (thetadot_d-y2)})
by = by.subs({theta: (theta_d-y1), thetadot: (thetadot_d-y2)})

fx, fy = symbols('f_x f_y')

u1 = u1.subs({theta: (theta_d-y1), thetadot: (thetadot_d-y2)})

xddot = fx + bx*u1
yddot = fy + by*u1

exddot_b = cos(psi)*(xddot_d - xddot) + sin(psi)*(yddot_d - yddot)

y1dot = y2
y3dot = -(lambda2*y3 + lambda3*y2 + lambda4*y1)/lambda1
y2dot = -(lambda1*exddot_b + lambda2*y3dot + lambda4*y2)/lambda3

# +
# Linearization
F = Matrix([y1dot,y2dot,y3dot])
y =  Matrix([y1, y2, y3])

jac = F.jacobian(y)
jac

# +
lin = jac.subs({y1:0, y2:0, y3:0}).evalf()
ux_s, uy_s, uz_s = symbols('u_x u_y u_z')
bx_s, by_s, bz_s = symbols('b_x b_y b_z')

ux_theta = cos(phi)*sin(theta_d)*cos(psi) + sin(phi)*sin(psi)
uy_theta = cos(phi)*sin(theta_d)*sin(psi) - sin(phi)*cos(psi)
uz_theta = cos(phi)*cos(theta_d)

bx_theta = bx.subs({theta: theta_d})
by_theta = by.subs({theta: theta_d})
bz_theta = bz.subs({theta: theta_d})

# lin = lin.subs({theta_d: 0}).evalf()
# lin = lin.subs({bx_theta: bx_s, by_theta: by_s, bz_theta: bz_s}).evalf()
# lin = lin.subs({ux_theta: ux_s, uy_theta: uy_s, uz_theta: uz_s}).evalf()

lin

# +
# gamma = symbols('\tgamma')
# poly = lin.charpoly(gamma)
# coefs = poly.coeffs()
# coefs

# +
A21 = lin[1,0]
A22 = lin[1,1]
A23 = lin[1,2]
a = lin[2,0]
b = lin[2,1]
c = lin[2,2]

c2 = -(A22 + c)
c1 = c*A22-A21-b*A23
c0 = c*A21 - a*A23
# -

simplify(c2)

simplify(c1/c0)

# simplify(c0)
c0 = simplify(c0)
# cL = (sin(phiL)*cos(psi)*cos(thetaL) + sin(psi)*sin(thetaL))
# cL = cL*sin(phi)*cos(phiL)*cos(thetaL)
# cL_s = symbols('C_L')
# c0 = c0.subs({cL: cL_s}).evalf()
c0
# collect(c0, Cx)

# +
# Cx = (M/m + cthetaL**2)
# Cy = (M/m + 1 - sphiL**2*cthetaL**2)
# Cz = (M/m + 1 - cphiL**2*cthetaL**2)
# octave_code(c0)
# -

simplify(c0.subs({phi: 0, phiL: 0, thetaL: 0}).evalf())

c0.subs({theta_d: 0.}).evalf()

simplify(c0)

simplify(c0.subs({phi: 0., theta: 0., psi: 0., phi_d: 0., theta_d: 0., psi_d: 0., phiL: 0., thetaL: 0.}).evalf())

m1 = Matrix([[cos(psi),sin(psi)],[-sin(psi),cos(psi)]])
m2 = Matrix([[cos(psi),sin(psi)],[sin(psi),-cos(psi)]])
simplify(m1*m2)
# m2

# +
F = symbols('F')

m22 = 1/lambda3*(-lambda4 + lambda2*lambda3/lambda1)
m23 = lambda2**2/(lambda1*lambda3)
m31 = -lambda4/lambda1
m32 = -lambda3/lambda1
m33 = -lambda2/lambda1

M = Matrix([[0,1,0],[F, m22, m23],[m31, m32, m33]])
M
# -

gamma = symbols('\tgamma')
poly = M.charpoly(gamma)
coefs = poly.coeffs()
simplify(coefs)

# c2 = -(A22 + c)
# c1 = c*A22-A21-b*A23
# c0 = c*A21 - a*A23
simplify(coefs[2]/coefs[3])

# +
F = diff(y2dot, y1).subs({y1:0, y2:0, y3:0}).evalf()
F = simplify(-lambda3/lambda1*F + lambda2*lambda4/lambda1**2)
F
# ux_s, uy_s, uz_s = symbols('u_x u_y u_z')
# bx_s, by_s, bz_s = symbols('b_x b_y b_z')

# ux_theta = cos(phi)*sin(theta_d)*cos(psi) + sin(phi)*sin(psi)
# uy_theta = cos(phi)*sin(theta_d)*sin(psi) - sin(phi)*cos(psi)
# uz_theta = cos(phi)*cos(theta_d)

# bx_theta = bx.subs({theta: theta_d})
# by_theta = by.subs({theta: theta_d})
# bz_theta = bz.subs({theta: theta_d})

# F = F.subs({bx_theta: bx_s, by_theta: by_s, bz_theta: bz_s}).evalf()
# F = F.subs({ux_theta: ux_s, uy_theta: uy_s, uz_theta: uz_s}).evalf()
# simplify(F.subs({phi: 0., theta: 0., psi: 0., phi_d: 0., theta_d: 0., psi_d: 0., phiL: 0., thetaL: 0.}).evalf())
# -

# ## $y$ and $\phi$

# +
bx = bx.subs({phi: (phi_d-y1), phidot: (phidot_d-y2)})
by = by.subs({phi: (phi_d-y1), phidot: (phidot_d-y2)})

fx, fy = symbols('f_x f_y')

u1 = u1.subs({phi: (phi_d-y1), phidot: (phidot_d-y2)})

xddot = fx + bx*u1
yddot = fy + by*u1

eyddot_b = -sin(psi)*(xddot_d - xddot) + cos(psi)*(yddot_d - yddot)

y1dot = y2
y3dot = -(lambda2*y3 + lambda3*y2 + lambda4*y1)/lambda1
y2dot = -(lambda1*eyddot_b + lambda2*y3dot + lambda4*y2)/lambda3

# +
# Linearization
F = Matrix([y1dot,y2dot,y3dot])
y =  Matrix([y1, y2, y3])

jac = F.jacobian(y)
jac

# +
lin = jac.subs({y1:0, y2:0, y3:0}).evalf()
ux_s, uy_s, uz_s = symbols('u_x u_y u_z')
bx_s, by_s, bz_s = symbols('b_x b_y b_z')

ux_theta = cos(phi)*sin(theta_d)*cos(psi) + sin(phi)*sin(psi)
uy_theta = cos(phi)*sin(theta_d)*sin(psi) - sin(phi)*cos(psi)
uz_theta = cos(phi)*cos(theta_d)

bx_theta = bx.subs({theta: theta_d})
by_theta = by.subs({theta: theta_d})
bz_theta = bz.subs({theta: theta_d})

# lin = lin.subs({theta_d: 0}).evalf()
# lin = lin.subs({bx_theta: bx_s, by_theta: by_s, bz_theta: bz_s}).evalf()
# lin = lin.subs({ux_theta: ux_s, uy_theta: uy_s, uz_theta: uz_s}).evalf()

lin

# +
A21 = lin[1,0]
A22 = lin[1,1]
A23 = lin[1,2]
a = lin[2,0]
b = lin[2,1]
c = lin[2,2]

c2 = -(A22 + c)
c1 = c*A22-A21-b*A23
c0 = c*A21 - a*A23
# -

simplify(c2)

simplify(c1/c0)

# simplify(c0)
c0 = simplify(c0)
# cL = (sin(phiL)*cos(psi)*cos(thetaL) + sin(psi)*sin(thetaL))
# cL = cL*sin(phi)*cos(phiL)*cos(thetaL)
# cL_s = symbols('C_L')
# c0 = c0.subs({cL: cL_s}).evalf()
c0
# collect(c0, Cx)

simplify(c0.subs({theta: 0, phiL: 0, thetaL: 0}).evalf())


