function tau = PDControler(bot, t, q, qd, qref, Kp, Kd)
    tau = (-Kp*(q-qref)' - Kd*qd')';
end