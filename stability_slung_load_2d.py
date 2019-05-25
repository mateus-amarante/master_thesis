# ---
# jupyter:
#   jupytext:
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

# # Stability Analysis of the SMC for the Slung Load System 2D
#
#

# +
from sympy import *
from sympy.physics.mechanics import dynamicsymbols, init_vprinting

init_vprinting()

# Time and system variables
t, y1, y2, y3, y4, y5 = symbols('t y_1 y_2 y_3 y_4 y_5')

# Control parameters
lambda1, lambda2, lambda3, lambda4, lambda5 = symbols('\tlambda_1 \tlambda_2 \tlambda_3 \tlambda_4 \tlambda_5')

# Physical parameters
m, M, l, g = symbols('m M l g')
# -

# # System Transformation (z, $\alpha$)

# +
# Original System Variables
theta, z_d, alpha_d = dynamicsymbols('\ttheta z_d \talpha_d')

zddot_d = diff(z_d,t,2)
alphadot_d = diff(alpha_d,t,1)
alphaddot_d = diff(alpha_d,t,2)

fz = -m*l*cos(alpha_d - y1)*(alphadot_d - y2)**2/(M+m) - g;
bz = (-m*sin(alpha_d - y1)*sin(theta - alpha_d +y1) + M*cos(theta))/(M*(M+m));

falpha = 0;
balpha = sin(theta - alpha_d + y1)/(M*l);

y1dot = y2
y3dot = -(lambda2*y3 + lambda3*y2 + lambda4*y1)/lambda1

u1_num = (lambda1*(zddot_d-fz) + lambda2*y3dot + lambda3*(alphaddot_d - falpha) + lambda4*y2);
u1_den = lambda1*bz + lambda3*balpha;

u1 = u1_num/u1_den

y2dot = alphaddot_d - balpha*u1;

# +
# Linearization
F = Matrix([y1dot,y2dot,y3dot])
y =  Matrix([y1, y2, y3])

# F = F.subs({alpha_d: 0., alphadot_d: 0., alphaddot_d: 0.}).evalf()
jac = F.jacobian(y)
jac
# -

lin = jac.subs({y1:0, y2:0, y3:0}).evalf()
lin2 = lin.subs({alpha_d: 0., alphadot_d: 0., alphaddot_d: 0.}).evalf()
lin2
# F.subs({y1:0, y2:0, y3:0,alpha_d: 0., alphadot_d: 0., alphaddot_d: 0.}).evalf()

gamma = symbols('\tgamma')
poly = lin2.charpoly(gamma)
coefs = poly.coeffs()
coefs

A21, A22, A23, a, b, c = symbols('A_21 A_22 A_23 a b c')
MM = Matrix([[0,1,1],[A21,A22,A23],[a,b,c]])
MM.charpoly(gamma)

# +
A21 = lin2[1,0]
A22 = lin2[1,1]
A23 = lin2[1,2]
a = lin2[2,0]
b = lin2[2,1]
c = lin2[2,2]

c2 = -(A22 + c)
c1 = c*A22-A21-b*A23
c0 = c*A21 - a*A23

# -

c2 = simplify(c2)
c2

c1 = simplify(c1)
c1

c0 = simplify(c0)
c0

# # System Transformation (x, $\theta$)

# +
# Original System Variables
theta, theta_d, x, x_d, alpha, alpha_d = dynamicsymbols('\ttheta \ttheta_d x x_d \talpha \talpha_d')
z, z_d, lambda_z = dynamicsymbols('z z_d \tlambda_z')

alphadot = diff(alpha,t)
alphadot_d = diff(alpha_d, t)

zdot = diff(z,t)
zdot_d = diff(z_d, t)
zddot_d = diff(z_d, t, 2)

xddot_d = diff(x_d,t,2)

fx, u1n = symbols('f_x u_1_n')


u1d = (-m*sin(alpha)*sin(theta_d-y1-alpha) + M*cos(theta_d-y1))

bx = (m*cos(alpha)*sin(theta_d-y1-alpha) + M*sin(theta_d-y1))
xddot = fx + bx*u1n/u1d
# bx = bx/(M*(M+m))
# xddot = fx + bx*u1

alphaddot = sin(theta_d - y1 - alpha_d + y3)*u1/(M*l)

y1dot = y2
y3dot = -(lambda2*y3 + lambda3*y2 + lambda4*y1)/lambda1
y2dot = -(lambda1*(xddot_d - xddot) + lambda2*y3dot + lambda4*y2)/lambda3
# y4dot = alphaddot

# +
# Linearization
F = Matrix([y1dot,y2dot,y3dot])
y =  Matrix([y1, y2, y3])

# F = F.subs({alpha_d: 0., alphadot_d: 0., alphaddot_d: 0.}).evalf()
jac = F.jacobian(y)
jac
# -

lin = jac.subs({y1:0, y2:0, y3:0}).evalf()
lin
# F.subs({y1:0, y2:0, y3:0,alpha_d: 0., alphadot_d: 0., alphaddot_d: 0.}).evalf()

gamma = symbols('\tgamma')
poly = lin.charpoly(gamma)
coefs = poly.coeffs()
coefs

A21, A22, A23, a, b, c = symbols('A_21 A_22 A_23 a b c')
MM = Matrix([[0,1,0],[A21,A22,A23],[a,b,c]])
poly = MM.charpoly(gamma)
factor(poly)

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
# c2

simplify(c1/c0)

simplify(c1)


simplify(c1.subs({theta_d: 0.}).evalf())

simplify(c0)

simplify(c0.subs({theta_d: 0.}).evalf())

# +
fz = -m*l*cos(alpha)*(alphadot_d - y4)**2/(M+m)
bz = (m*sin(alpha)*sin(alpha + y1 - theta_d) + M*cos(theta_d - y1))/(M*(M+m))
u1 = (zddot_d - fz + lambda_z*(zdot_d - zdot))/bz

alphaddot = sin(theta_d - y1 - alpha)/(M*l)*u1

y1dot = y2
y3dot = -(lambda2*y3 + lambda3*y2 + lambda4*y1 + lambda5*y4)/lambda1
y2dot = -(lambda1*(xddot_d - xddot) + lambda2*y3dot + lambda4*y2 + lambda5*(alphaddot_d - alphaddot))/lambda3
y4dot = alphaddot

# Linearization
F = Matrix([y1dot,y2dot,y3dot,y4dot])
y =  Matrix([y1, y2, y3, y4])

# F = F.subs({alpha_d: 0., alphadot_d: 0., alphaddot_d: 0.}).evalf()
jac = F.jacobian(y)
lin = jac.subs({y1:0, y2:0, y3:0, y4:0, alphadot_d: 0, alphaddot_d:0, theta_d: 0}).evalf()

gamma = symbols('\tgamma')
poly = lin.charpoly(gamma)
coefs = poly.coeffs()
coefs


# -

coefs[2]/coefs[1]

# # System Transformation ($\alpha$, $\theta$)

# +
# Original System Variables
theta, theta_d, alpha_d, alpha = dynamicsymbols('\ttheta \ttheta_d \talpha_d \talpha')

alphaddot_d = diff(alpha_d,t,2)

u1 = symbols('u_1')

alphaddot = sin(theta_d - y1 - alpha_d + y3)*u1/(M*l)


# u1d = (-m*sin(alpha_d-)*sin(theta_d-y1-alpha_d+y3) + M*cos(theta_d-y1))*cos(theta_d-y1)
# bx = (m*cos(alpha)*sin(theta_d-y1-alpha) + M*sin(theta_d-y1))*sin(theta_d-y1)

# xddot = fx + bx*u1n/u1d

# bx = bx/(M*(M+m))
# xddot = fx + bx*u1n

y1dot = y2
y3dot = -(lambda2*y3 + lambda3*y2 + lambda4*y1)/lambda1
y2dot = -(lambda1*(alphaddot_d - alphaddot) + lambda2*y3dot + lambda4*y2)/lambda3

# +
# Linearization
F = Matrix([y1dot,y2dot,y3dot])
y =  Matrix([y1, y2, y3])

# F = F.subs({alpha_d: 0., alphadot_d: 0., alphaddot_d: 0.}).evalf()
jac = F.jacobian(y)
jac
# -

lin = jac.subs({y1:0, y2:0, y3:0}).evalf()
lin.subs({theta_d: 0., alpha_d: 0}).evalf()

gamma = symbols('\tgamma')
poly = lin.charpoly(gamma)
coefs = poly.coeffs()
simplify(coefs)

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

simplify(c1)

simplify(c0)

simplify(c1/c0)

Poly((gamma+1)*(gamma+2)*(gamma+100))


