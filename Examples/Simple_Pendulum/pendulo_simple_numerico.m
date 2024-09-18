clear
%% Con Valores
addpath fun_handles/
g = 9.81
sys_pendulo_numeric = Dynamic_Library.System("Pendulo_Simple");

%Creamos el origen de nuestro sistema.
sys_pendulo_numeric.Create_New_Point("N","Canonical",sym([0;0;0]));
sys_pendulo_numeric.Create_New_Base("B_0",'axis_labels',[str2sym("x_0");str2sym("y_0");str2sym("z_0")]);
sys_pendulo_numeric.Create_New_Coordinate_System("C_0","N","B_0");

sys_pendulo_numeric.Create_New_Generalized_Coordinate("theta",str2sym("theta(t)"));
sys_pendulo_numeric.Create_New_Base("Bar", ...
                            'axis_labels',[str2sym("B_1");str2sym("B_2");str2sym("B_3")], ...
                              'father_base',"B_0", ...
                                'angle',str2sym("theta(t)"), ...
                                'axis_rotation',"1");
sys_pendulo_numeric.Create_New_Coordinate_System("Bar_CS_N","N","Bar");
l_1 = 1;
sys_pendulo_numeric.Create_New_Point("G","Bar_CS_N",[0;0;str2sym(string(-l_1/2))]);

Mass = 1;
sys_pendulo_numeric.Create_New_Rigid_Body("Bar", ...
                                'G_Point',"G",...
                                'Base',"Bar",...
                                'Mass',Mass, ...
                                'Inertial_Tensor',[1/12*Mass*(l_1^2) 0 0; ...
                                                    0 0 0 ; ...
                                                     0 0 0]);

sys_pendulo_numeric.Create_New_Action("Gravity",...
                                'Point',"G", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Bar", ...
                                'Vector',sym([0;0;0;0;0;-Mass*9.8]));

sys_pendulo_numeric.Create_New_Input("Moment_In",...
                                 'Point',"N", ...
                                 'Base',"B_0", ...
                                 'Rigid_Body',"Bar", ...
                                 'Vector',sym([str2sym("tau_in(t)");0;0;0;0;0]), ...
                                 'External_Variables',[str2sym("tau_in(t)")]);


sys_pendulo_numeric.unify_system

sys_pendulo_numeric.unify_symbolic_system 
[A,B] = sys_pendulo_numeric.Linealized_Model([0;0],[0])

A_uv_ode = @sys_pendulo_numeric.Get_A_tk_xk;
d_A_uv_ode =@sys_pendulo_numeric.Get_d_A_tk_xk;
tau_uv_ode = @sys_pendulo_numeric.Get_tau_tk_xk;
input_uv_ode = @sys_pendulo_numeric.Get_input_tk_xk;

vars = [sys_pendulo_numeric.v;sys_pendulo_numeric.u];
v_ode = odeFunction(sys_pendulo_numeric.v,vars,"File","v_function");

%%
%Compute trajectory
frames_per_second = 10;
dt = 1/frames_per_second;
sim_time = 10;
tspan = [0:dt:sim_time];
x0 = [0;pi/12];
X=[];
X(:,1)=x0;
xin = x0;
external_xin = [];
sz = size(tspan);
M_in_x_in = [zeros(sz)+0;zeros(sz)+0];

external_xin = [M_in_x_in];

for i=1:tspan(end)/dt
    time = i*dt;
    xout = rk4singlestep_Ab(A_uv_ode, ...
                            d_A_uv_ode, ...
                            tau_uv_ode, ...
                            [input_uv_ode], ...
                            v_ode, ...
                            dt, ...
                            time, ...
                            xin, ...
                            external_xin(:,i), ...
                            vars);
    X = [X xout];
    xin = xout;
    %if mod(i,10) == 0
        disp(i);
        disp(xout.');
        disp(string(datetime));
    %end
end
wo = sqrt(Mass*g*(l_1/2)/(1/3*Mass*(l_1^2)));
A = x0(2);
B = x0(1)/wo;
X_teorica = double([-wo*A*sin(wo.*tspan)+wo*B*cos(wo.*tspan);A*cos(wo.*tspan)+B*sin(wo.*tspan)]);
plot(tspan, X, tspan, X_teorica)
legend("V","U","V_teorica","U_teorica")