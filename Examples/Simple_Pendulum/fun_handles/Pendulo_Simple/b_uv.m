function expr = b_uv(t,in2)
%B_UV
%    EXPR = B_UV(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    26-Aug-2024 17:44:28

%b vector: Pendulo_Simple. Variables structure: [v_theta(t); u_theta(t)]
u_theta = in2(2,:);
expr = [sin(u_theta).*-4.9;0.0];
end
