# -*- coding: utf-8 -*-
# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py
#     text_representation:
#       extension: .py
#       format_name: light
#       format_version: '1.4'
#       jupytext_version: 1.1.7
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# +
from sympy import *
from sympy.physics.mechanics import dynamicsymbols, init_vprinting

init_vprinting()

# Ângulos de Euler
phi, theta, psi = dynamicsymbols('\phi \ttheta \psi')

# Angular Rate in the Body Frame
p, q, r = dynamicsymbols('p q r')

# Matrizes de rotação q = R*p (q -> inertial frame; p -> body frame)
Rx = rot_axis1(phi).T
Ry = rot_axis2(theta).T
Rz = rot_axis3(psi).T

R = Rz*Ry*Rx

Rb2i = R
Ri2b = R.T

# q_dot = R'*p (p constante p/ braço fixo)
# q_dot = R'*(RT*q) (map q to q_dot - Inertial Frame)
# RT*q_dot = p_dot = RT*R'*p (map p to p_dot)
# RT*R' and R'*RT are both skew-symmetric (definido por 3 termos)
RT_Rdot = simplify(R.T*diff(R)) # (Body Frame)
Rdot_RT = simplify(diff(R)*R.T) # (Inertial Frame)

# pqr (rpy,rpy_dot)
pqr = Matrix([diff(phi),0,0])+Rx.T*Matrix([0,diff(theta),0])+Rx.T*Ry.T*Matrix([0,0,diff(psi)])

# Transformação de velocidades angulares de I --> B
Ti2b = Matrix([[1,0,-sin(theta)],[0,cos(phi),cos(theta)*sin(phi)],[0,-sin(phi),cos(theta)*cos(phi)]])

# Transformação de velocidades angulares de B --> I
Tb2i = simplify(Ti2b**(-1))

# rpy_dot (rpy,pqr)
rpy_dot = simplify(Tb2i*Matrix([p,q,r]))

# Tb2i_dot
Tb2i_dot = simplify(diff(Tb2i))

# pqr_dot (rpy,rpy_dot)
pqr_dot = simplify(diff(pqr))

# rpy_ddot (rpy,rpy_dot)
rpy_ddot = simplify(Tb2i_dot*pqr + Tb2i*diff(Matrix([p,q,4])))

# -

Rx # B->I

Ry # B->I

Rz # B->I

R # Rb2i

R.T # Ri2b

RT_Rdot # map p to p_dot (Body Frame)

Rdot_RT # map q to q_dot (Inertial Frame)

pqr # pqr (rpy_dot)

Ti2b # Transformação de velocidades angulares I->B

rpy_dot # rpy_dot (pqr)

Tb2i # Transformação de velocidades angulares B->I

Tb2i_dot # Tb2i_dot

pqr_dot # pqr_dot (rpy,rpy_dot)

rpy_ddot # rpy_ddot (rpy,rpy_dot)


