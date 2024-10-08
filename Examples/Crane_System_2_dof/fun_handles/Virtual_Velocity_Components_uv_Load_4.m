function expr = Virtual_Velocity_Components_uv_Load_4(t,in2)
%Virtual_Velocity_Components_uv_Load_4
%    EXPR = Virtual_Velocity_Components_uv_Load_4(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    26-Aug-2024 17:54:18

%Virtual Velocity Components: Rigid Body: Load beta_1(t)
u_beta_1 = in2(9,:);
u_beta_2 = in2(8,:);
u_theta_1 = in2(6,:);
t2 = cos(u_beta_1);
t3 = cos(u_beta_2);
t4 = cos(u_theta_1);
t5 = sin(u_beta_1);
t6 = sin(u_beta_2);
t7 = sin(u_theta_1);
expr = [t3.*t4;t3.*t7;-t6;t2.*t7.*(-7.39e+2./2.5e+2)+t4.*t5.*t6.*(7.39e+2./2.5e+2);t2.*t4.*(7.39e+2./2.5e+2)+t5.*t6.*t7.*(7.39e+2./2.5e+2);t3.*t5.*(7.39e+2./2.5e+2)];
end
