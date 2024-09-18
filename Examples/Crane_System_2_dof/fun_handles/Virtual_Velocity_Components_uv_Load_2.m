function expr = Virtual_Velocity_Components_uv_Load_2(t,in2)
%Virtual_Velocity_Components_uv_Load_2
%    EXPR = Virtual_Velocity_Components_uv_Load_2(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    26-Aug-2024 17:54:16

%Virtual Velocity Components: Rigid Body: Load r(t)
u_theta_1 = in2(6,:);
expr = [0.0;0.0;0.0;cos(u_theta_1);sin(u_theta_1);0.0];
end
