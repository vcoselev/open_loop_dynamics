clear
%syms g
g = 9.81;
Crane_System = Dynamic_Library.System("Crane");
%Creamos el origen de nuestro sistema.
Crane_System.Create_New_Point("N","Canonical",sym([0;0;0]));
Crane_System.Create_New_Base("B_0",'axis_labels',[str2sym("x_0");str2sym("y_0");str2sym("z_0")]);
Crane_System.Create_New_Coordinate_System("C_0","N","B_0");

% Introducimos la coordenada generalizada theta_1(t)
Crane_System.Create_New_Generalized_Coordinate("theta_1",str2sym("theta_1(t)"));
% Posicionamos el centro de masas de la base de la torre
OG_Base = sym([0.3513; 1.38; 24.9501]);
Crane_System.Create_New_Point("G_Base","C_0",OG_Base);
Crane_System.Create_New_Coordinate_System("G_Base_CS","G_Base","B_0");
G_Base_Join_Point_Base_Jib = sym([0; 0; 18.9995]);
Crane_System.Create_New_Point("Join_Point_Base_Jib","G_Base_CS",G_Base_Join_Point_Base_Jib);
% Cremos la base de la pluma y el referencial que va con la pluma.
Crane_System.Create_New_Base("Jib", ...
                            'axis_labels',[str2sym("J_1");str2sym("J_2");str2sym("J_3")], ...
                              'father_base',"B_0", ...
                                'angle',str2sym("theta_1(t)"), ...
                                'axis_rotation',"3");
Crane_System.Create_New_Coordinate_System("Referencial_Rotation_Axis_Jib","Join_Point_Base_Jib","Jib");


%Creamos el referencial del carrito
Crane_System.Create_New_Generalized_Coordinate("r",str2sym("r(t)"));
Join_Point_Base_Jib_G_Trolley = sym([29.1647+str2sym("r(t)");0.0446;-0.3451]);
Crane_System.Create_New_Point("A_ast","Referencial_Rotation_Axis_Jib",Join_Point_Base_Jib_G_Trolley);
Crane_System.Create_New_Coordinate_System("Referencial_Trolley","A_ast","Jib");

Crane_System.Create_New_Generalized_Coordinate("beta_2",str2sym("beta_2(t)"));
Crane_System.Create_New_Base("Bar_beta_2", ...
                            'axis_labels',[str2sym("P_1_2");str2sym("P_2_2");str2sym("P_3_2")], ...
                              'father_base',"Jib", ...
                                'angle',str2sym("beta_2(t)"), ...
                                'axis_rotation',"2");

Crane_System.Create_New_Generalized_Coordinate("beta_1",str2sym("beta_1(t)"));
Crane_System.Create_New_Base("Bar_beta_1", ...
                            'axis_labels',[str2sym("P_1_1");str2sym("P_2_1");str2sym("P_3_1")], ...
                              'father_base',"Bar_beta_2", ...
                                'angle',str2sym("beta_1(t)"), ...
                                'axis_rotation',"1");

Crane_System.Create_New_Generalized_Coordinate("Gamma",str2sym("Gamma(t)"));
Crane_System.Create_New_Base("Bar", ...
                            'axis_labels',[str2sym("P_1");str2sym("P_2");str2sym("P_3")], ...
                              'father_base',"Bar_beta_1", ...
                                'angle',str2sym("Gamma(t)"), ...
                                'axis_rotation',"3");

Crane_System.Create_New_Coordinate_System("Referencial_Trolley_Bar","A_ast","Bar");
%Crane_System.Create_New_Coordinate_System("Referencial_Trolley_Bar","A_ast","Bar_beta_1");
%Crane_System.Create_New_Generalized_Coordinate("ly",str2sym("ly(t)"));
%Crane_System.Create_New_Point("P_ast","Referencial_Trolley_Bar",sym([0;0;-str2sym("ly(t)")]));
ly = 1;
G_Trolley_G_Load = sym([0;0;ly-3.9560]);
%G_Trolley_G_Load = sym([0;0;str2sym("ly(t)")-3.9560]);
Crane_System.Create_New_Point("P_ast","Referencial_Trolley_Bar",G_Trolley_G_Load);
%Creamos los solidos rigidos del sistema.

%Crane_System.Create_New_Point("G_Jib","Referencial_N_Jib",sym([str2sym("x_CG_Jib");0;0]));
Join_Point_Base_Jib_G_Jib = sym([-2.2937;0.0657;0]);
Crane_System.Create_New_Point("G_Jib","Referencial_Rotation_Axis_Jib",Join_Point_Base_Jib_G_Jib)
I_Jib_Concrete = 1.0e+04 *[1.7719   0   0;
                           0    2.1950    0;
                           0    0    1.5159];
Mass_Jib_Concrete = 2.1872e+04;

I_Jib_Steel = [0.1513    0    0;
                0   3.1988     0;
                0    0.1824    3.0953]*1.0e+07;

Mass_Jib_Steel = 1.7551e+05;

I_Jib = I_Jib_Concrete+I_Jib_Steel;

Mass_Jib = Mass_Jib_Concrete + Mass_Jib_Steel;

% I_Jib = sym([]);
% for i = 1:3
%     for j = 1:3
%             if i <= j
%                 str = "I_Jib_"+string(i)+string(j);
%                 I_Jib(i,j)=str2sym(str);
%                 Crane_System.Create_New_Parameter(str,str2sym(str));
% 
%             else
%                 str = "I_Jib_"+string(j)+string(i);
%                 I_Jib(i,j)=str2sym(str);
%             end
%     end
% end
% Mass_Jib = str2sym("m_Jib");
Crane_System.Create_New_Rigid_Body("Jib", ...
                                'G_Point',"G_Jib",...
                                'Base',"Jib",...
                                'Mass',Mass_Jib, ...
                                'Inertial_Tensor',I_Jib);

Crane_System.Create_New_Action("Gravity_Jib",...
                                'Point',"G_Jib", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Jib", ...
                                'Vector',sym([0;0;0;0;0;-Mass_Jib*g]));
Vec_Angular_Velocity_Jib = Crane_System.Angular_Velocity("B_0","Jib","B_0");
cin_vis = 200E-4; % m^2/s
density = 850; % Kg/m^3
mu = cin_vis * density;
r = 1.35;
h= 0.4;
e = 0.05*r;
c = mu/e;
A = 2*pi*r*h;
c = 100000;
Coulomb_Friction_Moment = -h*pi*r^2*(r+1)*c*A*Vec_Angular_Velocity_Jib;
% Crane_System.Create_New_Action("Coulomb_Friction_Moment",...
%                                 'Point',"Join_Point_Base_Jib", ...
%                                 'Base',"B_0", ...
%                                 'Rigid_Body',"Jib", ...
%                                 'Vector',[Coulomb_Friction_Moment;0;0;0]);
Crane_System.Create_New_Input("Moment_In_Jib",...
                                 'Point',"Join_Point_Base_Jib", ...
                                 'Base',"B_0", ...
                                 'Rigid_Body',"Jib", ...
                                 'Vector',sym([0;0;str2sym("tau_in(t)");0;0;0]), ...
                                 'External_Variables',[str2sym("tau_in(t)")]);

I_Trolley = 1.0e+03 *[2.5231   0    0;
   0    1.5859   0;
    0   0    2.0106];

Mass_Trolley = 2.6562e+03;

% I_Trolley = sym([]);
% for i = 1:3
%     for j = 1:3
%             if i <= j
%                 str = "I_Trolley_"+string(i)+string(j);
%                 I_Trolley(i,j)=str2sym(str);
%                 Crane_System.Create_New_Parameter(str,str2sym(str));
% 
%             else
%                 str = "I_Trolley_"+string(j)+string(i);
%                 I_Trolley(i,j)=str2sym(str);
%             end
%     end
% end
% Mass_Trolley = str2sym("m_Trolley");
Crane_System.Create_New_Rigid_Body("Trolley", ...
                                'G_Point',"A_ast",...
                                'Base',"Jib",...
                                'Mass',Mass_Trolley, ...
                                'Inertial_Tensor',I_Trolley);

Crane_System.Create_New_Action("Gravity_Trolley",...
                                'Point',"A_ast", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Trolley", ...
                                'Vector',sym([0;0;0;0;0;-Mass_Trolley*g]));

% Crane_System.Create_New_Input("Force_In_Trolley",...
%                                  'Point',"A_ast", ...
%                                  'Base',"Jib", ...
%                                  'Rigid_Body',"Trolley", ...
%                                  'Vector',sym([0;0;0;str2sym("F_in(t)");0;0]), ...
%                                  'External_Variables',[str2sym("F_in(t)")]);

%Crane_System.Create_New_Point("Centroid_Surface_Trolley","Canonical",sym([29.486520767211914; 1.4035983085632324; 43.916645936279295]))
Crane_System.Create_New_Point("Centroid_Surface_Trolley","Referencial_Rotation_Axis_Jib",sym([29.1352+str2sym("r(t)");0.0236;-0.0330]))
a = 1.38;
b = 2.38;
A = a*b;
c = 100000;
G_Trolley_To_GCentroid_Trolley = Crane_System.Get_Two_Points_Vector("A_ast", "Centroid_Surface_Trolley");
Centroid_Surface_Trolley_Obj = Crane_System.Get_Point_Info("System_Points",'point',"Centroid_Surface_Trolley");
V_Centroid_Surface_Trolley = Centroid_Surface_Trolley_Obj{2}.Get_Info("Point_Velocity_Coordinates_From_Canonical");
F_Vis_Trolley = -c*A*V_Centroid_Surface_Trolley;
M_Vis_Trolley = simplify(cross(G_Trolley_To_GCentroid_Trolley,F_Vis_Trolley));
% Crane_System.Create_New_Action("Friction_Trolley",...
%                                 'Point',"Centroid_Surface_Trolley",...
%                                  'Base',"Jib", ...
%                                  'Rigid_Body',"Trolley", ...
%                                  'Vector',[M_Vis_Trolley;F_Vis_Trolley]);
I_Load = [145.5235    0    0;
    0   168.1075   5.8392;
    0    5.8392   43.3397];
Mass_Load = 1.1109e+03;
% I_Load = sym([]);
% for i = 1:3
%     for j = 1:3
%             if i <= j
%                 str = "I_Load_"+string(i)+string(j);
%                 I_Load(i,j)=str2sym(str);
%                 Crane_System.Create_New_Parameter(str,str2sym(str));
% 
%             else
%                 str = "I_Load_"+string(j)+string(i);
%                 I_Load(i,j)=str2sym(str);
%             end
%     end
% end
% Mass_Load = str2sym("m_Load");
% Crane_System.Create_New_Rigid_Body("Load", ...
%                                 'G_Point',"P_ast",...
%                                 'Base',"Bar_beta_1",...
%                                 'Mass',Mass_Load, ...
%                                 'Inertial_Tensor',I_Load);

Crane_System.Create_New_Rigid_Body("Load", ...
                                'G_Point',"P_ast",...
                                'Base',"Bar",...
                                'Mass',Mass_Load, ...
                                'Inertial_Tensor',I_Load);

Crane_System.Create_New_Action("Gravity_Load",...
                                'Point',"P_ast", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Load", ...
                                'Vector',sym([0;0;0;0;0;-Mass_Load*g]));
Crane_System.Create_New_Input("Force_In_Bar",...
                                 'Point',"P_ast", ...
                                 'Base',"Bar", ...
                                 'Rigid_Body',"Load", ...
                                 'Vector',sym([0;0;0;0;0;str2sym("F_h(t)")]), ...
                                 'External_Variables',[str2sym("F_h(t)")]);
addpath fun_handles
Crane_System.unify_system 

Crane_System.unify_symbolic_system
[A,B]=Crane_System.Linealized_Model([0;0;0;0;0;0;0;0;0;0],[0;0])
% [v_theta_1(t); v_r(t); v_beta_2(t); v_beta_1(t); v_Gamma(t); v_ly(t); u_theta_1(t); u_r(t); u_beta_2(t); u_beta_1(t); u_Gamma(t); u_ly(t); tau_in(t); F_in(t)]
Crane_System.Get_input_tk_xk(0,[0;0;0;0;0;0;0;0;0;0;0;0],[0;0])

A_uv_ode = @Crane_System.Get_A_tk_xk;
d_A_uv_ode =@Crane_System.Get_d_A_tk_xk;
tau_uv_ode = @Crane_System.Get_tau_tk_xk;
input_uv_ode = @Crane_System.Get_input_tk_xk;

full_b_handle_one_input = @(v) [Crane_System.Get_tau_tk_xk(v(1),v(2:11))+Crane_System.Get_input_tk_xk(v(1),v(2:11),v(12:13))].'
full_b_handle_one_input([[0];[0;0;0;0;0;0;0;0;0;0];[0;0]])
tau_uv_ode_one_input([[0];[0;0;0;0;0;0;0;0;0;0];[0;0]])
input_uv_ode_one_input([[0];[0;0;0;0;0;0;0;0;0;0];[0;0]])

tau_uv_ode_one_input(zeros([1,13]))

fminsearch(full_b_handle_one_input,[[0];[0;0;0;0;0;0;0;0;0;0];[0;0]])

T_Jib_uv_ode = Crane_System.Transformation_Matrix_Handles{1,3};
T_Trolley_uv_ode = Crane_System.Transformation_Matrix_Handles{2,3};
T_Load_uv_ode = Crane_System.Transformation_Matrix_Handles{3,3};
vars = [Crane_System.v;Crane_System.u];
v_ode = odeFunction(Crane_System.v,vars,"File","v_function");
%% Compute trajectory
frames_per_second = 60;
dt = 1/frames_per_second;
sim_time = 5;
tspan = [0:dt:sim_time];
x0 = [0;0;0;0;0;0;0;-pi/4];
%x0 = [0;0;0;0;0;0;0;0];
X=[];
X(:,1)=x0;
xin = x0;
external_xin = [];
sz = size(tspan);
M_in_x_in = [zeros(sz)+0;zeros(sz)+0];
F_in_x_in = [zeros(sz)+0;zeros(sz)+0;zeros(sz)+0];
%M_in_x_in = [[zeros(1,241)+1000000;zeros(1,241)+2000000],[zeros(1,240)-3000000;zeros(1,240)-4000000]];
%F_in_x_in = [[zeros(1,121)+10000;zeros(1,121)+0;zeros(1,121)+0],[zeros(1,120)-10000;zeros(1,120)+0;zeros(1,120)+0]]

external_xin = [M_in_x_in;F_in_x_in];
t_0 = datetime;
vars_csv = ["i","d_theta_1","d_r","d_beta_2","d_beta_1","theta_1","r","beta_2","beta_1", "t"];
writematrix(vars_csv,'xout.csv','Delimiter','comma','WriteMode','append');
for i=1:tspan(end)/dt-1
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
    writematrix([i, string(xout.'), string(datetime-t_0)],'xout.csv','Delimiter','comma','WriteMode','append');
    xin = xout;
    T_Jib = T_Jib_uv_ode(time,xout);
    writematrix(T_Jib,'Jib.csv','Delimiter','comma','WriteMode','append');
    T_Trolley = T_Trolley_uv_ode(time,xout);
    writematrix(T_Trolley,'Trolley.csv','Delimiter','comma','WriteMode','append');
    T_Load = T_Load_uv_ode(time,xout);
    writematrix(T_Load,'Load.csv','Delimiter','comma','WriteMode','append');
    %if mod(i,10) == 0
        disp(time)
        disp(xout.');
        disp(string(datetime-t_0));
    %end
end

%% 
plot(tspan(1:300),X,'LineWidth',1)
legend(string(vars)) 

hold on

plot(tspan(1:300), simulink_simulation.Data(2,:))
legend(string(vars(2:2))) 