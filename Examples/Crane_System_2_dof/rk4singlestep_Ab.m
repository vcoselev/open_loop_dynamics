function xout = rk4singlestep_Ab(A, d_A, tau, in, v, dt, tk, xk, external_xin, vars)
    A_1 = A(tk,xk);
    d_A_1 = d_A(tk,xk);
    tau_1 = tau(tk,xk);
    input_1 = in(tk,xk,external_xin);
    v_1 = v(tk,xk);
    b_1 = -d_A_1*v_1+tau_1+input_1;
    f1 = [A_1\b_1; v_1];

    A_2 = A(tk+dt/2,xk+(dt/2)*f1);
    d_A_2 = d_A(tk+dt/2,xk+(dt/2)*f1);
    tau_2 = tau(tk+dt/2,xk+(dt/2)*f1);
    input_2 = in(tk+dt/2,xk+(dt/2)*f1,external_xin);
    v_2 = v(tk+dt/2,xk+(dt/2)*f1);
    b_2 = -d_A_2*v_2+tau_2+input_2;
    f2 = [A_2\b_2; v_2];

    A_3 = A(tk+dt/2,xk+(dt/2)*f2);
    d_A_3 = d_A(tk+dt/2,xk+(dt/2)*f2);
    tau_3 = tau(tk+dt/2,xk+(dt/2)*f2);
    input_4 = in(tk+dt/2,xk+(dt/2)*f2, external_xin);
    v_3 = v(tk+dt/2,xk+(dt/2)*f2);
    b_3 = -d_A_3*v_3+tau_3+input_4;
    f3 = [A_3\b_3 ; v_3];

    A_4 = A(tk+dt, xk+dt*f3);
    d_A_4 = d_A(tk+dt, xk+dt*f3);
    tau_4 = tau(tk+dt, xk+dt*f3);
    input_4 = in(tk+dt, xk+dt*f3, external_xin);
    v_4 = v(tk+dt, xk+dt*f3);
    b_4 = -d_A_4*v_4+tau_4+input_4;
    f4 = [A_4\b_4; v_4];

    xout = xk +(dt/6)*(f1+2*f2+2*f3+f4);
end