function expr = d_Virtual_Velocity_Components_uv_Load_4(t,in2)
%d_Virtual_Velocity_Components_uv_Load_4
%    EXPR = d_Virtual_Velocity_Components_uv_Load_4(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    26-Aug-2024 17:54:18

%Virtual Velocity Components: Rigid Body: Load beta_1(t)
u_beta_1 = in2(9,:);
u_beta_2 = in2(8,:);
u_theta_1 = in2(6,:);
v_beta_1 = in2(4,:);
v_beta_2 = in2(3,:);
v_theta_1 = in2(1,:);
t2 = cos(u_beta_1);
t3 = cos(u_beta_2);
t4 = cos(u_theta_1);
t5 = sin(u_beta_1);
t6 = sin(u_beta_2);
t7 = sin(u_theta_1);
expr = [-t4.*t6.*v_beta_2-t3.*t7.*v_theta_1;-t6.*t7.*v_beta_2+t3.*t4.*v_theta_1;-t3.*v_beta_2;t5.*t7.*v_beta_1.*(7.39e+2./2.5e+2)-t2.*t4.*v_theta_1.*(7.39e+2./2.5e+2)+t2.*t4.*t6.*v_beta_1.*(7.39e+2./2.5e+2)+t3.*t4.*t5.*v_beta_2.*(7.39e+2./2.5e+2)-t5.*t6.*t7.*v_theta_1.*(7.39e+2./2.5e+2);t4.*t5.*v_beta_1.*(-7.39e+2./2.5e+2)-t2.*t7.*v_theta_1.*(7.39e+2./2.5e+2)+t2.*t6.*t7.*v_beta_1.*(7.39e+2./2.5e+2)+t3.*t5.*t7.*v_beta_2.*(7.39e+2./2.5e+2)+t4.*t5.*t6.*v_theta_1.*(7.39e+2./2.5e+2);t2.*t3.*v_beta_1.*(7.39e+2./2.5e+2)-t5.*t6.*v_beta_2.*(7.39e+2./2.5e+2)];
end
