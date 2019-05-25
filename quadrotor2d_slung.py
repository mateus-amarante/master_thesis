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

# # Dynamic Model of Quadrotor with Suspended Load (2D)
#
# OBS:
# * This document does not cover the rotational dynamics, since it's the same of the quadrotor
# * There is a summary at the end
#
# ## Common Setup

# +
from sympy import *
from sympy.physics.mechanics import dynamicsymbols, init_vprinting

init_vprinting()

# Time and input variables
t, u1, u2, u1x, u1z = symbols('t u1 u2 u1_x u1_z')

# Physical parameters
m, M, l, g, I = symbols('m M l g I')

# Pitch and load angles
theta, alpha = dynamicsymbols('\ttheta \talpha')

# Angular Velocities
thetadot = diff(theta,t)
alphadot = diff(alpha,t)

# Angular Accelerations
thetaddot = diff(thetadot,t)
alphaddot = diff(alphadot,t)

# Auxiliary variables
ux = sin(theta)
uz = cos(theta)
# -

# ## Coordinates $(x,z,\alpha)$

# +
# Position
x,z = dynamicsymbols('x z')
xL = x - l*sin(alpha) # xL as function of x
zL = z - l*cos(alpha) # zL as function of z

# Velocities
xdot = diff(x,t)
zdot = diff(z,t)

xLdot = diff(xL,t)
zLdot = diff(zL,t)

thetadot = diff(theta,t)
alphadot = diff(alpha,t)

# Accelerations
xddot = diff(xdot,t)
zddot = diff(zdot,t)

xLddot = diff(xLdot,t)
zLddot = diff(zLdot,t)

print('LOAD VELOCITIES:')
display(simplify(xLdot),simplify(zLdot))

print('LOAD ACCELERATIONS:')
display(simplify(xLddot),simplify(zLddot))
# -

L = M/2*(xdot**2+zdot**2) + m/2*(xLdot**2+zLdot**2) - M*g*z - m*g*zL
L = simplify(expand(L))
L = collect(collect(L,zdot**2),xdot**2)
L = collect(L,m*g)
L = collect(L,m*l*alphadot)
L

# $$ L = \left(\frac{M+m}{2}\right)\left(\dot{x}^2 + \dot{z}^2\right) + \frac{ml^2}{2}\dot{\alpha}^2 + ml\dot{\alpha}\left(- \dot{x}\cos{\alpha} + \dot{z}\sin{\alpha}\right) + g\left[ml\cos\alpha-(M+m)z\right]$$

eqX = simplify(diff(diff(L,xdot),t) - diff(L,x)-u1x)
eqX

eqZ = simplify(diff(diff(L,zdot),t) - diff(L,z)-u1z)
eqZ 

eqAlpha = simplify(diff(diff(L,alphadot),t) - diff(L,alpha))
eqAlpha

# ### Dynamic Model $(x,z,\alpha)$
#
# $$
# \begin{align}
# (M+m)\,\ddot{x} - ml\cos{\alpha}\,\ddot{\alpha} + ml\sin{\alpha}\,\dot{\alpha}^2 &= \sin{\theta}\,u_1 \\
# (M+m)\,\ddot{z} + ml\sin{\alpha}\,\ddot{\alpha} + ml\cos{\alpha}\,\dot{\alpha}^2 + (M+m)g &= \cos{\theta}\,u_1 \\
# ml^2\,\ddot{\alpha} - ml\cos{\alpha}\,\ddot{x} + ml\sin{\alpha}\,\ddot{z} + mgl\sin{\alpha} &= 0
# \end{align}
# $$ 

res = solve([eqX,eqZ,eqAlpha],[xddot,zddot,alphaddot])

res

fx = res[xddot]
fx = collect(collect(fx,m*u1x),u1x)
fx

# **Simplification of $u_1m$ term:**

aux = expand(expand_trig(-sin(alpha)*cos(alpha-theta)+sin(theta)))
aux = collect(aux,sin(theta))
aux

# $$
# \cos^2{\alpha} \sin{\theta} - \cos{\alpha} \sin{\alpha} \cos{\theta} \\
# \cos{\alpha}(\cos{\alpha} \sin{\theta} - \sin{\alpha} \cos{\theta}) \\
# -\cos{\alpha}\sin{(\alpha-\theta)}
# $$

fz = res[zddot]
fz = collect(fz,u1z)
fz = collect(fz,g)
fz

falpha = res[alphaddot]
falpha

# ### Accelerations $(x,z,\alpha)$
# $$
# \begin{align}
# \ddot{x} &= \left[\frac{-ml\sin{\alpha}\,\dot{\alpha}^2}{M+m}\right] + \left[\frac{-m\cos\alpha\sin(\alpha-\theta) +M\sin{\theta}}{M(M+m)}\right]u_1 \\
# \ddot{z} &= \left[-\frac{ml\cos{\alpha}\,\dot{\alpha}^2}{M+m}-g\right] + \left[\frac{m\sin\alpha\sin(\alpha-\theta) +M\cos{\theta}}{M(M+m)}\right]u_1 \\
# \ddot{\alpha} &= \left[\frac{\sin(\theta-\alpha)}{Ml}\right]u_1
# \end{align}
# $$

# ## Coordinates $(x_L,z_L,\alpha)$

# +
# Position
xL,zL = dynamicsymbols('x_L z_L')
x = xL + l*sin(alpha) # x as function of xL
z = zL + l*cos(alpha) # z as function of zL

# Velocities
xdot = diff(x,t)
zdot = diff(z,t)

xLdot = diff(xL,t)
zLdot = diff(zL,t)

thetadot = diff(theta,t)
alphadot = diff(alpha,t)

# Accelerations
xddot = diff(xdot,t)
zddot = diff(zdot,t)

xLddot = diff(xLdot,t)
zLddot = diff(zLdot,t)
# -

L = M/2*(xdot**2+zdot**2) + m/2*(xLdot**2+zLdot**2) - M*g*z - m*g*zL
L = simplify(expand(L))
L = collect(collect(L,zLdot**2),xLdot**2)
L = collect(L,g)
L = collect(L,M*l*alphadot)
L

# $$ L = \left(\frac{M+m}{2}\right)\left(\dot{x}_L^2 + \dot{z}_L^2\right) + \frac{Ml^2}{2}\dot{\alpha}^2 + Ml\dot{\alpha}\left(\dot{x}_L\cos{\alpha}-\dot{z}_L\sin{\alpha}\right)  + g\left[-Ml\cos\alpha-(M+m)z_L\right]$$

eqX = simplify(diff(diff(L,xLdot),t) - diff(L,xL)-ux*u1)
eqX

eqZ = simplify(diff(diff(L,zLdot),t) - diff(L,zL)-uz*u1)
eqZ = collect(eqZ,g)
eqZ 

eqAlpha = simplify(diff(diff(L,alphadot),t) - diff(L,alpha))
eqAlpha

# ### Dynamic Model $(x_L,z_L,\alpha)$
#
# $$
# \begin{align}
# (M+m)\,\ddot{x}_L + Ml\cos{\alpha}\,\ddot{\alpha} - Ml\sin{\alpha}\,\dot{\alpha}^2 &= \sin{\theta}\,u_1 \\
# (M+m)\,\ddot{z}_L - Ml\sin{\alpha}\,\ddot{\alpha} - Ml\cos{\alpha}\,\dot{\alpha}^2 + (M+m)g &= \cos{\theta}\,u_1 \\
# Ml^2\,\ddot{\alpha} + Ml\cos{\alpha}\,\ddot{x}_L - Ml\sin{\alpha}\,\ddot{z}_L - Mgl\sin{\alpha} &= 0
# \end{align}
# $$ 

res = solve([eqX,eqZ,eqAlpha],[xLddot,zLddot,alphaddot])

res

fxL = res[xLddot]
fxL = collect(collect(fxL,M*u1),u1)
fxL

fzL = res[zLddot]
fzL = collect(fzL,u1)
fzL = collect(fzL,g)
fzL

falpha = res[alphaddot]
falpha

# ### Accelerations $(x_L,z_L,\alpha)$
# $$
# \begin{align}
# \ddot{x}_L &= \left[\frac{Ml\sin{\alpha}\,\dot{\alpha}^2}{M+m}\right] + \left[\frac{-M\cos\alpha\sin(\alpha-\theta) +m\sin{\theta}}{m(M+m)}\right]u_1 \\
# \ddot{z}_L &= \left[\frac{Ml\cos{\alpha}\,\dot{\alpha}^2}{M+m}-g\right] + \left[\frac{M\sin\alpha\sin(\alpha-\theta) +m\cos{\theta}}{m(M+m)}\right]u_1 \\
# \ddot{\alpha} &= \left[\frac{\sin(\alpha-\theta)}{ml}\right]u_1
# \end{align}
# $$

# ## Summary
#
# ### Dynamic Model
#
# ####  $(x,z,\alpha)$
#
# $$
# \begin{align}
# (M+m)\,\ddot{x} - ml\cos{\alpha}\,\ddot{\alpha} + ml\sin{\alpha}\,\dot{\alpha}^2 &= \sin{\theta}\,u_1 \\
# (M+m)\,\ddot{z} + ml\sin{\alpha}\,\ddot{\alpha} + ml\cos{\alpha}\,\dot{\alpha}^2 + (M+m)g &= \cos{\theta}\,u_1 \\
# ml^2\,\ddot{\alpha} - ml\cos{\alpha}\,\ddot{x} + ml\sin{\alpha}\,\ddot{z} + mgl\sin{\alpha} &= 0
# \end{align}
# $$ 
#
# #### $(x_L,z_L,\alpha)$
#
# $$
# \begin{align}
# (M+m)\,\ddot{x}_L + Ml\cos{\alpha}\,\ddot{\alpha} - Ml\sin{\alpha}\,\dot{\alpha}^2 &= \sin{\theta}\,u_1 \\
# (M+m)\,\ddot{z}_L - Ml\sin{\alpha}\,\ddot{\alpha} - Ml\cos{\alpha}\,\dot{\alpha}^2 + (M+m)g &= \cos{\theta}\,u_1 \\
# Ml^2\,\ddot{\alpha} + Ml\cos{\alpha}\,\ddot{x}_L - Ml\sin{\alpha}\,\ddot{z}_L - Mgl\sin{\alpha} &= 0
# \end{align}
# $$ 
#
# ### Accelerations
#
# #### $(x,z,\alpha)$
# $$
# \begin{align}
# \ddot{x} &= \left[\frac{-ml\sin{\alpha}\,\dot{\alpha}^2}{M+m}\right] + \left[\frac{-m\cos\alpha\sin(\alpha-\theta) +M\sin{\theta}}{M(M+m)}\right]u_1 \\
# \ddot{z} &= \left[-\frac{ml\cos{\alpha}\,\dot{\alpha}^2}{M+m}-g\right] + \left[\frac{m\sin\alpha\sin(\alpha-\theta) +M\cos{\theta}}{M(M+m)}\right]u_1 \\
# \ddot{\alpha} &= \left[\frac{-\sin(\alpha-\theta)}{Ml}\right]u_1
# \end{align}
# $$
#
# ### Accelerations
#
# #### $(x_L,z_L,\alpha)$
# $$
# \begin{align}
# \ddot{x}_L &= \left[\frac{Ml\sin{\alpha}\,\dot{\alpha}^2}{M+m}\right] + \left[\frac{-M\cos\alpha\sin(\alpha-\theta) +m\sin{\theta}}{m(M+m)}\right]u_1 \\
# \ddot{z}_L &= \left[\frac{Ml\cos{\alpha}\,\dot{\alpha}^2}{M+m}-g\right] + \left[\frac{M\sin\alpha\sin(\alpha-\theta) +m\cos{\theta}}{m(M+m)}\right]u_1 \\
# \ddot{\alpha} &= \left[\frac{\sin(\alpha-\theta)}{ml}\right]u_1
# \end{align}
# $$


