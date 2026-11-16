# Ideal three-phase inverter

These equations describe [inverter.py](inverter.py). The model converts three PWM duty cycles into average motor phase voltages over one PWM period.

## Signals

| Symbol | Code name | Meaning |
| --- | --- | --- |
| $d_a,d_b,d_c$ | `u` | Input duty cycles, in phase order a, b, c |
| $V_{bus}$ | `V_bus` | Positive DC supply voltage (V) |
| $v_{a,pole},v_{b,pole},v_{c,pole}$ | `v_pole` | Average inverter output voltages relative to the negative DC rail (V) |
| $v_n$ | `v_pole.mean()` | Floating motor neutral voltage relative to the negative DC rail (V) |
| $V_a,V_b,V_c$ | Return value | Phase voltages relative to the motor neutral (V) |

A duty cycle of 0 means the output is connected to the negative rail for the whole PWM period. A duty cycle of 1 means it is connected to the positive rail for the whole period. A value of 0.5 means half the period at each rail.

## 1. Limit duty cycles

For each phase $k\in\{a,b,c\}$:

$$
\bar d_k=\min(1,\max(0,d_k))
$$

Values below 0 become 0, and values above 1 become 1. The code requires exactly three finite input values; otherwise it raises `ValueError`.

## 2. Average pole voltages

$$
v_{k,pole}=V_{bus}\bar d_k
$$

Each pole voltage is the supply voltage multiplied by the fraction of time spent at the positive rail.

## 3. Floating neutral voltage

$$
v_n=\frac{v_{a,pole}+v_{b,pole}+v_{c,pole}}{3}
$$

The model represents a floating star-connected motor neutral by the average of the three pole voltages.

## 4. Motor phase voltages

$$
V_a=v_{a,pole}-v_n,\qquad
V_b=v_{b,pole}-v_n,\qquad
V_c=v_{c,pole}-v_n
$$

Equivalently, with $\bar d_{avg}=(\bar d_a+\bar d_b+\bar d_c)/3$:

$$
\boxed{V_k=V_{bus}(\bar d_k-\bar d_{avg})}
$$

Subtracting the neutral removes the voltage shared by all phases (common-mode voltage). Therefore $V_a+V_b+V_c=0$. The line-to-line voltage is simply $V_a-V_b=V_{bus}(\bar d_a-\bar d_b)$, and similarly for the other phase pairs.

## Example

For $V_{bus}=48\,\mathrm{V}$ and duty cycles $[0.75,0.50,0.25]$:

$$
\mathbf v_{pole}=[36,24,12]\,\mathrm{V},\qquad v_n=24\,\mathrm{V}
$$

$$
[V_a,V_b,V_c]=[12,0,-12]\,\mathrm{V}.
$$

## Connection to the current controller

The current controller produces $[v_d,v_q]$. The existing `transforms.svpwm(theta_e, vd, vq, V_bus)` function converts these to three **pole-voltage references**, in volts. Divide those references by $V_{bus}$ before passing them to this inverter as duty cycles:

$$
\mathbf d=\frac{\operatorname{svpwm}(\theta_e,v_d,v_q,V_{bus})}{V_{bus}}.
$$

The inverter has no dynamic states; its `t` and `x` arguments do not affect the output. It models average voltage only, with no switching ripple, dead time, semiconductor voltage drops, or losses.
