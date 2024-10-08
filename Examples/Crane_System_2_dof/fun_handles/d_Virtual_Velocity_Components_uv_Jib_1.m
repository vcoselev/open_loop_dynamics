function expr = d_Virtual_Velocity_Components_uv_Jib_1(t,in2)
%d_Virtual_Velocity_Components_uv_Jib_1
%    EXPR = d_Virtual_Velocity_Components_uv_Jib_1(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    26-Aug-2024 17:54:12

%Virtual Velocity Components: Rigid Body: Jib theta_1(t)
u_theta_1 = in2(6,:);
v_theta_1 = in2(1,:);
t2 = atan(3.49117199391172e+1);
t4 = 2.294640751838945e+4;
t3 = t2+u_theta_1;
expr = [0.0;0.0;0.0;(t4.*v_theta_1.*sin(t3))./1.0e+4;t4.*v_theta_1.*cos(t3).*(-1.0e-4);0.0];
end
