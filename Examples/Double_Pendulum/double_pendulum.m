clear

%Parameters
g = str2sym("g");
l_1 = str2sym("l_1");
l_2 = str2sym("l_2");
m_1 = str2sym("m_1");
m_2 = str2sym("m_2");
% I_11_Bar_1 = m_1*(l_1)^2/12;
I_11_Bar_1 = str2sym("I_Bar_1_11");
I_Bar_1 = [I_11_Bar_1 0 0;
            0 0 0;
            0 0 0];

%I_11_Bar_2 = m_2*(l_2)^2/12;
I_11_Bar_2 = str2sym("I_Bar_2_11");

I_Bar_2 = [I_11_Bar_2 0 0;
            0 0 0;
            0 0 0];

theta_1 = str2sym("theta_1(t)");
theta_2 = str2sym("theta_2(t)");



sys_compound_pendulum = Dynamic_Library.System("compound_pendulum");

sys_compound_pendulum.Create_New_Point("N","Canonical",sym([0;0;0]));
sys_compound_pendulum.Create_New_Base("B_0",'axis_labels',[str2sym("x_0");str2sym("y_0");str2sym("z_0")]);
sys_compound_pendulum.Create_New_Coordinate_System("C_0","N","B_0");

%% BAR 1
sys_compound_pendulum.Create_New_Generalized_Coordinate("theta_1",theta_1);
sys_compound_pendulum.Create_New_Base("Bar_1", ...
                            'axis_labels',[str2sym("x_1");str2sym("y_1");str2sym("z_1")], ...
                              'father_base',"B_0", ...
                                'angle',theta_1, ...
                                'axis_rotation',"1");
sys_compound_pendulum.Create_New_Coordinate_System("Bar_1_CS_N","N","Bar_1");
sys_compound_pendulum.Create_New_Point("G_1","Bar_1_CS_N",[0;0;-l_1/2]);

sys_compound_pendulum.Create_New_Point("U","Bar_1_CS_N",[0;0;-l_1]);


sys_compound_pendulum.Create_New_Rigid_Body("Bar_1", ...
                                'G_Point',"G_1",...
                                'Base',"Bar_1",...
                                'Mass',m_1, ...
                                'Intertial_Tensor',I_Bar_1);

sys_compound_pendulum.Create_New_Action("Gravity_1",...
                                'Point',"G_1", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Bar_1", ...
                                'Vector',sym([0;0;0;0;0;-m_1*g]));
%% BAR 2

sys_compound_pendulum.Create_New_Generalized_Coordinate("theta_2",theta_2);
sys_compound_pendulum.Create_New_Base("Bar_2", ...
                            'axis_labels',[str2sym("x_2");str2sym("y_2");str2sym("z_2")], ...
                              'father_base',"B_0", ...
                                'angle',theta_2, ...
                                'axis_rotation',"1");
sys_compound_pendulum.Create_New_Coordinate_System("Bar_2_CS_N","U","Bar_2");
sys_compound_pendulum.Create_New_Point("G_2","Bar_2_CS_N",[0;0;-l_2/2]);


sys_compound_pendulum.Create_New_Rigid_Body("Bar_2", ...
                                'G_Point',"G_2",...
                                'Base',"Bar_2",...
                                'Mass',m_2, ...
                                'Intertial_Tensor',I_Bar_2);

sys_compound_pendulum.Create_New_Action("Gravity_2",...
                                'Point',"G_2", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Bar_2", ...
                                'Vector',sym([0;0;0;0;0;-m_2*g]));

sys_compound_pendulum.unify_symbolic_system

%% NUMERICO

clear

%Parameters
g = 9.81;
l_1 = 1;
l_2 = 1;
m_1 = 1;
m_2 = 1;
I_11_Bar_1 = m_1*(2*l_1)^2/12;
I_Bar_1 = [I_11_Bar_1 0 0;
            0 0 0;
            0 0 0];

I_11_Bar_2 = m_2*(2*l_2)^2/12;
I_Bar_2 = [I_11_Bar_2 0 0;
            0 0 0;
            0 0 0];

theta_1 = str2sym("theta_1(t)");
theta_2 = str2sym("theta_2(t)");


sys_compound_pendulum_numeric = Dynamic_Library.System("compound_pendulum");

sys_compound_pendulum_numeric.Create_New_Point("N","Canonical",sym([0;0;0]));
sys_compound_pendulum_numeric.Create_New_Base("B_0",'axis_labels',[str2sym("x_0");str2sym("y_0");str2sym("z_0")]);
sys_compound_pendulum_numeric.Create_New_Coordinate_System("C_0","N","B_0");

% BAR 1
sys_compound_pendulum_numeric.Create_New_Generalized_Coordinate("theta_1",theta_1);
sys_compound_pendulum_numeric.Create_New_Base("Bar_1", ...
                            'axis_labels',[str2sym("x_1");str2sym("y_1");str2sym("z_1")], ...
                              'father_base',"B_0", ...
                                'angle',theta_1, ...
                                'axis_rotation',"1");
sys_compound_pendulum_numeric.Create_New_Coordinate_System("Bar_1_CS_N","N","Bar_1");
sys_compound_pendulum_numeric.Create_New_Point("G_1","Bar_1_CS_N",[0;0;-l_1]);

sys_compound_pendulum_numeric.Create_New_Point("U","Bar_1_CS_N",[0;0;-2*l_1]);


sys_compound_pendulum_numeric.Create_New_Rigid_Body("Bar_1", ...
                                'G_Point',"G_1",...
                                'Base',"Bar_1",...
                                'Mass',m_1, ...
                                'Intertial_Tensor',I_Bar_1);

sys_compound_pendulum_numeric.Create_New_Action("Gravity_1",...
                                'Point',"G_1", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Bar_1", ...
                                'Vector',sym([0;0;0;0;0;-m_1*g]));
% BAR 2

sys_compound_pendulum_numeric.Create_New_Generalized_Coordinate("theta_2",theta_2);
sys_compound_pendulum_numeric.Create_New_Base("Bar_2", ...
                            'axis_labels',[str2sym("x_2");str2sym("y_2");str2sym("z_2")], ...
                              'father_base',"B_0", ...
                                'angle',theta_2, ...
                                'axis_rotation',"1");
sys_compound_pendulum_numeric.Create_New_Coordinate_System("Bar_2_CS_N","U","Bar_2");
sys_compound_pendulum_numeric.Create_New_Point("G_2","Bar_2_CS_N",[0;0;-l_2]);


sys_compound_pendulum_numeric.Create_New_Rigid_Body("Bar_2", ...
                                'G_Point',"G_2",...
                                'Base',"Bar_2",...
                                'Mass',m_2, ...
                                'Intertial_Tensor',I_Bar_2);

sys_compound_pendulum_numeric.Create_New_Action("Gravity_2",...
                                'Point',"G_2", ...
                                'Base',"B_0", ...
                                'Rigid_Body',"Bar_2", ...
                                'Vector',sym([0;0;0;0;0;-m_2*g]));

sys_compound_pendulum_numeric.unify_system


sys_compound_pendulum_numeric.unify_symbolic_system