function expr = Virtual_Velocity_Components_uv_Load_3(t,in2)
%Virtual_Velocity_Components_uv_Load_3
%    EXPR = Virtual_Velocity_Components_uv_Load_3(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    26-Aug-2024 17:54:17

%Virtual Velocity Components: Rigid Body: Load beta_2(t)
u_beta_1 = in2(9,:);
u_beta_2 = in2(8,:);
u_theta_1 = in2(6,:);
t2 = cos(u_beta_1);
t3 = cos(u_beta_2);
t4 = cos(u_theta_1);
t5 = sin(u_theta_1);
expr = [-t5;t4;0.0;t2.*t3.*t4.*(-7.39e+2./2.5e+2);t2.*t3.*t5.*(-7.39e+2./2.5e+2);t2.*sin(u_beta_2).*(7.39e+2./2.5e+2)];
end
