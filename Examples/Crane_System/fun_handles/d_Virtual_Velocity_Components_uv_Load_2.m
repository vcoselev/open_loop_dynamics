function expr = d_Virtual_Velocity_Components_uv_Load_2(t,in2)
%d_Virtual_Velocity_Components_uv_Load_2
%    EXPR = d_Virtual_Velocity_Components_uv_Load_2(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    27-Aug-2024 11:50:47

%Virtual Velocity Components: Rigid Body: Load r(t)
u_theta_1 = in2(6,:);
v_theta_1 = in2(1,:);
expr = [0.0;0.0;0.0;-v_theta_1.*sin(u_theta_1);v_theta_1.*cos(u_theta_1);0.0];
end
