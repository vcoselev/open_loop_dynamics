function expr = Virtual_Velocity_Components_uv_Load_5(t,in2)
%Virtual_Velocity_Components_uv_Load_5
%    EXPR = Virtual_Velocity_Components_uv_Load_5(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    27-Aug-2024 11:50:49

%Virtual Velocity Components: Rigid Body: Load Gamma(t)
u_beta_1 = in2(9,:);
u_beta_2 = in2(8,:);
u_theta_1 = in2(6,:);
t2 = cos(u_beta_1);
t3 = cos(u_theta_1);
t4 = sin(u_beta_1);
t5 = sin(u_beta_2);
t6 = sin(u_theta_1);
expr = [t4.*t6+t2.*t3.*t5;-t3.*t4+t2.*t5.*t6;t2.*cos(u_beta_2);0.0;0.0;0.0];
end