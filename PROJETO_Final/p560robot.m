function statedot = PUMA560(state, u, p560)
q = state(1, 1:6);
qd = state(1, 7:12);
t = u;
qdd = p560.accel(q, qd, t)
statedot = [qd; qdd];