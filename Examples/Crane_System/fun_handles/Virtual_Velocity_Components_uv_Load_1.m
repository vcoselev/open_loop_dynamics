function expr = Virtual_Velocity_Components_uv_Load_1(t,in2)
%Virtual_Velocity_Components_uv_Load_1
%    EXPR = Virtual_Velocity_Components_uv_Load_1(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    27-Aug-2024 11:50:46

%Virtual Velocity Components: Rigid Body: Load theta_1(t)
u_beta_1 = in2(9,:);
u_beta_2 = in2(8,:);
u_r = in2(7,:);
u_theta_1 = in2(6,:);
t2 = cos(u_beta_1);
t3 = cos(u_theta_1);
t4 = sin(u_beta_1);
t5 = sin(u_beta_2);
t6 = sin(u_theta_1);
t7 = u_r+2.91647e+1;
expr = [0.0;0.0;1.0;t3.*(-4.46e-2)-t3.*t4.*(7.39e+2./2.5e+2)-t6.*t7+t2.*t5.*t6.*(7.39e+2./2.5e+2);t6.*(-4.46e-2)+t3.*t7-t4.*t6.*(7.39e+2./2.5e+2)-t2.*t3.*t5.*(7.39e+2./2.5e+2);0.0];
end
