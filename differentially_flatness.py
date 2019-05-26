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
#     display_name: Python 2
#     language: python
#     name: python2
# ---

# # Differentially Flatness of Slung-Load System
#
# Dynamic Model (Newton-Euler):
#
# $$
# M \ddot{\vec{x}} = f R \vec{e}_z - m g \vec{e}_z + T\vec{p} \\
# m \ddot{\vec{x}}_L = -T\vec{p} - m g \vec{e}_z \\
# I \dot{\vec{\omega}} + \vec{\omega} \times I \vec{\omega} = \vec{\tau}
# $$
#
# Geometric relationship:
#
# $$
# \vec{x} = \vec{x}_L - L\vec{p}
# $$
#
# ## Step 1: $\vec{x}_L \rightarrow \vec{x}$
#
# **Lemma 1:** From $\vec{x}_L$ and its higher-order derivatives, we can determine $\vec{x}$ and its higher-order derivatives. The $nth$ derivative of $\vec{x}$ is determined from the $(n+2)th$ derivative of $\vec{x}_L$ and the lower order derivatives
#
# **Proof:**
#
# First, determine the tension force:
#
# $$
# T\vec{p} = - m \ddot{\vec{x}}_L - m g \vec{e}_z \\
# \vec{p} = \frac{T\vec{p}}{\|{T\vec{p}}\|} \\
# T = T\vec{p} \cdot \vec{p}
# $$
#
# Then $\vec{x}$ is determined from the geometric relationship and its derivatives:
#
# $$
# \vec{x} = \vec{x}_L - L\vec{p} \\
# \dot{\vec{x}} = \dot{\vec{x}}_L - L\dot{\vec{p}} \\
# \ddot{\vec{x}} = \ddot{\vec{x}}_L - L\ddot{\vec{p}} \\
# \vdots \\
# \vec{x}^{(n)} = {\vec{x}_L}^{(n)} - L\vec{p}^{(n)}
# $$
#
# $\dot{\vec{p}}$ is found by derivating the second equation of the dynamic model:
#
# $$
# \dot{\vec{p}} = -\frac{(m\dddot{\vec{x}}_L + \dot{T}\vec{p})}{T}
# $$
#
# $\dot{T}$ can be determined by derivating $T$ obtained directly from the second equation of the dynamic model:
#
# $$
# T = \|-m\ddot{\vec{x}}_L -mg\vec{e}_z \| = \sqrt{m^2\left[{\ddot{\vec{x}}_L}^2+{\ddot{\vec{y}}_L}^2 \left( {\ddot{\vec{z}}_L}^2 + g\right) \right]}
# $$
#
# Note that, by derivating the two equations above, $\vec{p}^{(n)}$ will depend on the $(n+2)th$ derivative of $\vec{x}_L$ and the lower order derivatives.
#
# It follows the expressions for $\vec{p}^{(n)}$ and $T^{(n)}$ up to the fourth order.
#

# +
from sympy import *
from sympy.physics.mechanics import *

# from math import *

init_vprinting()

t = symbols('t')

xL, yL, zL, T, p = dynamicsymbols('x_L y_L z_L T p')

# Physical parameters
m, M, l, g, I = symbols('m M l g I')

xLddot = diff(xL, t, 2)
yLddot = diff(yL, t, 2)
zLddot = diff(zL, t, 2)

T_exp = sqrt((-m*xLddot)**2 + (-m*yLddot)**2 + (-m*zLddot-m*g)**2)
Tdot_exp = diff(T_exp, t).subs(T_exp, T)
Tddot_exp = diff(Tdot_exp, t).subs(Tdot_exp, diff(T))
T3dot_exp = diff(Tddot_exp, t).subs(Tddot_exp, diff(T, t, 2))
T4dot_exp = diff(T3dot_exp, t).subs(T3dot_exp, diff(T, t, 3))

display(simplify(T_exp))
display(simplify(Tdot_exp))
vprint(simplify(Tddot_exp))
print('--------------------------------')
vprint(simplify(T3dot_exp))
print('--------------------------------')
vprint(simplify(T4dot_exp))

# +
# In the following expressions, xL is a vector

pdot_exp  = (-m*diff(xL,t,3) - diff(T,t)*p)/T
pddot_exp = diff(pdot_exp,t).subs(pdot_exp, diff(p))
p3dot_exp = diff(pddot_exp,t).subs(pddot_exp, diff(p,t,2))
p4dot_exp = diff(p3dot_exp,t).subs(p3dot_exp, diff(p,t,3))

vprint(simplify(pdot_exp))
vprint(simplify(pddot_exp))
vprint(simplify(p3dot_exp))
vprint(simplify(p4dot_exp))
# -

# ## Step2: $[\psi, \vec{x}] \rightarrow [R, f, \vec{\omega}, \vec{\alpha}, \vec{\tau}]$
#
# **Lemma 1:** From $[\psi, \vec{x}]$ and its higher-order derivatives, we can determine $[R, f, \vec{\omega}, \vec{\tau}]$. To define $[\vec{\alpha}, \vec{\tau}]$ we need up to the fourth derivative of $\vec{x}$.
#
# **Proof:**
#
# ###  $\rightarrow[R, f]$
#
# Equation of motion can be re-written as:
#
# $$
# M \ddot{\vec{x}} - T\vec{p} + M g \vec{e}_z = f \vec{e}_{z_b} \qquad \text{where} \quad \vec{e}_{z_b} = R \vec{e}_z
# $$
#
# Then:
#
# $$
# \vec{e}_{z_b} = \frac{\vec{t}}{\| \vec{t} \|} \qquad \text{where} \quad \vec{t} = \left[ \ddot{x} - \frac{T_x}{M}, \ddot{y} - \frac{T_y}{M}, \ddot{z} + g - \frac{T_z}{M} \right]^T
# $$
#
# Defining $\vec{e}_{x_c}$ as the projection of the body frame to XY plane, given the yaw angle:
#
# $$
# \vec{e}_{x_c} = {[\cos \psi, \sin \psi, 0]^T}
# $$
#
# We can completely determine the body frame unit vectors:
#
# $$
# \vec{e}_{y_b} = \frac{e_{z_b} \times e_{x_c}}{\| e_{z_b} \times e_{x_c} \|} \\
# \vec{e}_{x_b} = \vec{e}_{y_b} \times \vec{e}_{z_b}
# $$
#
# Then we can determine the rotation matrix:
#
# $$
# R = [\vec{e}_{x_b}, \vec{e}_{y_b}, \vec{e}_{z_b}]
# $$
#
# Observe that, with R we can obtain $f$ from the dynamic model equation.
#
# ### $\rightarrow[\vec{\omega}]$
#
# Derivating the first equation of motion, we get:
#
# $$
# M\dddot{\vec{x}} = \dot{f}\vec{e}_{z_b} + f(\vec{\omega} \times \vec{e}_{z_b})
# $$
#
# Projecting this expression along $e_{z_b}$ and noting that $\dot{f} = M\dddot{\vec{x}}\cdot\vec{e}_{z_b}$, substituting $\dot{f}$ we define the vector $\vec{h}$:
#
# $$
# \vec{h}_\omega = \vec{\omega} \times \vec{e}_{z_b} = \frac{M}{f}\left[  \dddot{\vec{x}} - (\dddot{\vec{x}} \cdot \vec{e}_{z_b})\vec{e}_{z_b} \right]
# $$
#
# $\vec{h}_\omega$ is the projection of $\frac{M}{f}\dddot{\vec{x}}$ onto $x_b -- y_b$ plane, which has the same module of and perpendicular to the projection of $\vec{\omega}$ onto this plane. Then p, q can be determined:
#
# $$
# p = -\vec{h}_\omega \cdot \vec{e}_{y_b} \\
# q = \vec{h}_\omega \cdot \vec{e}_{x_b} \\
# $$
#
# The third component of the angular velocity is determined by simply writing $\vec{\omega}_{bw} = \vec{\omega}_{bc} + \vec{\omega}_{cw}$ and observing that $\vec{\omega}_{bc}$ has no component along $\vec{e}_{z_b}$. So this component is the projection of the yaw velocity from the inertial frame onto the body frame:
#
# $$
# r = \dot{\psi}\vec{e}_{z} \cdot \vec{e}_{z_b}
# $$
#
# ### $\rightarrow[\dot{\vec{\omega}}, \vec{\tau}]$
#
# Derivating the first dynamic equation twice:
#
# $$
# M \ddddot{\vec{x}} = \ddot{f}\vec{e}_{z_b} + 2 \vec{\omega} \times \dot{f} \vec{e}_{z_b} + \vec{\alpha} \times f \vec{e}_{z_b} + \vec{\omega} \times \left( \vec{\omega} \times f \vec{e}_{z_b} \right)
# $$
#
# Projecting this expression along $e_{z_b}$ and noting that $\ddot{f} = M\ddddot{\vec{x}}\cdot\vec{e}_{z_b} - \left[ \vec{\omega} \times \left( \vec{\omega} \times f \vec{e}_{z_b} \right) \right] \cdot \vec{e}_{z_b} $, substituting $\dot{f}$ we define the vector $\vec{h}_\alpha$:
#
# $$
# \vec{h}_\alpha = \vec{\alpha} \times \vec{e}_{z_b}
# $$
#
# $\vec{h}_\alpha$ is on the plane $x_b -- y_b$ and is perpendicular to the projection of $\vec{\alpha}$ onto this plane. Then $\dot{p}$ and $\dot{q}$ can be determined:
#
# $$
# \dot{p} = -\vec{h}_\alpha \cdot \vec{e}_{y_b} \\
# \dot{q} = \vec{h}_\alpha \cdot \vec{e}_{x_b} \\
# $$
#
# By writing the angular accelerations a sum of two components:
#
# $$
# \vec{\alpha}_{bw} = \vec{\alpha}_{bc} + \vec{\omega}_{cw} \times \vec{\omega}_{bc} + \vec{\alpha}_{cw}
# $$
#
# Projecting this expression along $\vec{e}{z_b}$, we get: $\vec{\alpha}_{bc} \cdot \vec{e}_{z_b} = 0,\, (\vec{\omega}_{cw} \times \vec{\omega}_{bc}) \cdot \vec{e}_{z_b} = 0$. So $\dot{r}$ is the projection of the yaw acceleration from the inertial frame onto the body frame:
#
# $$
# r = \ddot{\psi}\vec{e}_{z} \cdot \vec{e}_{z_b}
# $$
#
# Finally, $\vec{\tau}$ is determined directly from the dynamic model third equation.


