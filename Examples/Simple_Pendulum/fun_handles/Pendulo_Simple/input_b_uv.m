function expr = input_b_uv(t,in2)
%INPUT_B_UV
%    EXPR = INPUT_B_UV(T,IN2)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    26-Aug-2024 17:44:29

%input_b vector: Pendulo_Simple. Variables structure: [v_theta(t); u_theta(t); tau_in(t)]
tau_in = in2(3,:);
expr = [tau_in;0.0];
end
