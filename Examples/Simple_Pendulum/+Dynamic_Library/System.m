%> @file System.m
%> @brief System Class description
% ======================================================================
%> @brief System Class for the creation of System objects.
% ======================================================================
classdef System < handle
    %% SYSTEM
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            
        end
        % PROTECTED
        properties(Access = protected)

        end

        % PUBLIC
        properties(Access = public)
            %> String that defines the name of the system.
            System_Name
            %> [n x 1] symbolic array that contains the generalized coordinates of the system.
            q_variables
            %> [n x 1] handle of the functions that calculate the virtual displacements components.
            Virtual_Velocity_Components_fun_handles
            %> [n x n] handle of the functions that calculate the A matrix.
            A_Matrix_Fun_Handles
            u
            v
            external_vars
            Transformation_Matrix_Handles
        end
        %SYMBOLIC
        properties(Access = public)
            A
            A_uv
            d_A
            d_A_uv
            q
            uv
            vu
            b
            b_uv
            input_b
            input_b_uv
        end
     % METHODS

        methods(Access = private)
            
        end
        % PROTECTED
        methods(Access = protected)

        end

        % PUBLIC
        methods(Access = public)
            % SYSTEM CONSTRUCTOR
            function obj = System(Name)
                %SYSTEM INIT CONFIGURATION
                obj.System_Name = Name;
                %BASE INIT CONFIGURATION
                obj.System_Canonical_Base = obj.System_Create_Canonical_Base;
                obj.System_Bases = obj.System_Create_Bases_Table;
                %POINT INIT CONFIGURATION
                obj.System_Canonical_Point = obj.System_Create_Canonical_Point;
                obj.System_Points = obj.System_Create_Points_Table;
                %COORDINATE SYSTEM INIT CONFIGURATION
                obj.System_Canonical_Coordinate_System = obj.System_Create_Canonical_Coordinate_System;
                obj.System_Coordinate_Systems = obj.System_Create_Coordinate_System_Table;
                %RIGID BODY INIT CONFIGURATION
                obj.System_Canonical_Rigid_Body = obj.System_Create_Canonical_Rigid_Body;
                obj.System_Rigid_Bodies = obj.System_Create_Rigid_Body_Table;
                %ACTION INIT CONFIGURATION
                obj.System_Canonical_Action = obj.System_Create_Canonical_action;
                obj.System_Actions = obj.System_Create_Action_Table;
                %GENERALIZED COORDINATES INIT CONFIGURATION
                obj.System_Generalized_Coordinates = obj.System_Create_Generalized_Coordinates_Table;
                %INPUT INIT CONFIGURATION
                obj.System_Canonical_Input = obj.System_Create_Canonical_Input;
                obj.System_Inputs = obj.System_Create_Input_Table;
                
            end
            %> @brief The function unify_system creates the function handles for the calculation of virtual displacements of the system. This virtual displacements are used to calculate the matrix \f$ \left[ \mathbf{A} \right] \f$ (function Get_A_tk_xk), its derivative \f$ \frac{d}{dt} \left[ \mathbf{A} \right] \f$ (function Get_d_A_tk_xk) and the generalized actions \f$ \mathbf{\tau}\f$ (function Get_tau_tk_xk and function Get_input_tk_xk).
            function unify_system(obj)
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                n = length(q_cell);
                syms t
                q = sym([]);
                for i = 1:n
                    q = [q ; q_cell{i}];
                end
                d_q = diff(q,t);
                [uv,obj.u,obj.v] = obj.order_reduction(q,q,d_q);
                obj.q_variables = q;
                obj.Virtual_Velocity_Components_fun_handles = obj.Get_System_Virtual_Velocity_q_uv_handle;
                obj.A_Matrix_Fun_Handles = obj.Get_System_A_Matrix_Handles;
                obj.Transformation_Matrix_Handles = obj.Get_System_Transformation_Matrix_Handle

                
            end
            %> @brief The unify_symbolic_system function calculates the matrix \f$ \left[ \mathbf{A} \right] \f$, its derivative \f$ \frac{d}{dt} \left[ \mathbf{A} \right] \f$ and the generalized actions \f$ \mathbf{\tau} \f$ (the generalized actions are given by the inputs and other external actions). The system of equations is presented as \f$ \left[ \mathbf{A} \right] \mathbf{\ddot{q}} = \mathbf{b}\f$  where the vector \f$ \mathbf{b}\f$ has been calculated as \f$ \mathbf{b} =  -\frac{d}{dt} \left[ \mathbf{A} \right] \mathbf{\dot{q}} + \mathbf{\tau}\f$. As the system of equations presented is of second order the function also returns the reduced system of equations to a first order system of equations \f$ \left[\mathbf{u},\mathbf{v}, \mathbf{\dot{v}} \right]^T = \left[\mathbf{q} ,\mathbf{\dot{q}}, \mathbf{\ddot{q}} \right]^T\f$. This way the reduced system of equations is \f$ \left[ \left[\mathbf{A_{u}} \right] \mathbf{v} , \mathbf{u}\right]^T = \left[ \mathbf{b_{uv}} , \mathbf{v}\right]^T \f$ where \f$ \mathbf{A_u} =  \mathbf{A(\mathbf{u})}\f$ and \f$ \mathbf{b_{uv}} = \mathbf{b(\mathbf{u},\mathbf{v})}  \f$  
            function unify_symbolic_system(obj)
                mkdir("fun_handles/"+obj.System_Name);
                [obj.A,obj.A_uv] = obj.Get_System_Kinetic_Energy_Quadratic_Matrix;
                obj.A = simplify(obj.A,"Seconds",5);
                obj.A_uv = simplify(obj.A_uv,"Seconds",5);
                sz = size(obj.A_uv);
                obj.A_uv= [obj.A_uv, zeros(sz);[zeros(sz),eye(sz)]];
                
                uv = [obj.v;obj.u];
                uv_string = string(uv(1));

                for i = 2:length(uv)
                    uv_string = uv_string + "; "+ string(uv(i));
                end


                A_Matrix_uv_handle = odeFunction(obj.A_uv,[obj.v;obj.u], ...
                                                "File", "fun_handles/"+obj.System_Name+"/A_uv", ...
                                                "Comments","A matrix: "+obj.System_Name+". Variables structure: ["+uv_string+"]",...
                                                "Optimize",true);
                [obj.b,obj.b_uv] = obj.Lagrangian_Rigth_Hand_Side;
                b_uv_handle = odeFunction(obj.b_uv,[obj.v;obj.u], ...
                                                "File", "fun_handles/"+obj.System_Name+"/b_uv", ...
                                                "Comments","b vector: "+obj.System_Name+". Variables structure: ["+uv_string+"]",...
                                                "Optimize",true);
                [obj.input_b,obj.input_b_uv] = obj.Input_Lagrangian_Rigth_Hand_Side;
                
                uv_inputs = [obj.v;obj.u;obj.external_vars];

                uv_inputs_string = string(uv(1));

                for i = 2:length(uv_inputs)
                    uv_inputs_string = uv_inputs_string + "; "+ string(uv_inputs(i));
                end

                input_b_uv_handle = odeFunction(obj.input_b_uv,[obj.v;obj.u;obj.external_vars], ...
                                                "File", "fun_handles/"+obj.System_Name+"/input_b_uv", ...
                                                "Comments","input_b vector: "+obj.System_Name+". Variables structure: ["+uv_inputs_string+"]",...
                                                "Optimize",true);

                v_uv_handle = odeFunction([zeros(size(obj.v));obj.v],[obj.v;obj.u], ...
                                                "File", "fun_handles/"+obj.System_Name+"/v_uv", ...
                                                "Comments","v vector: "+obj.System_Name+". Variables structure: ["+uv_string+"]",...
                                                "Optimize",true);

            end

            function out = dot_A_dot_q(obj)
            % ======================================================================
            %> @brief dot_A_dot_q function gives the [N x 1] array that represents the \f$\dot{\left[A\right]}\dot{\vec{q}}\f$ term of the EOM. This function is used for the calculation of the RHS terms.
            %>
            %> @param obj System
            % ======================================================================
                syms t
                % A = obj.A;
                % d_A = diff(A,t);
                % d_A = simplify(diff(A,t),'Steps',10);
                d_A = obj.Get_System_d_Kinetic_Energy_Quadratic_Matrix;
                obj.d_A = d_A;
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                n = length(q_cell);
                q = sym([]);
                for i = 1:n
                    q = [q ; q_cell{i}];
                end
                d_q = diff(q,t);
                %out = simplify(d_A*d_q);
                out = d_A*d_q;
            end

            function out = d_K_d_q(obj)
                A = obj.A;
                q = sym([]);
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                n = length(q_cell);
                for i = 1:n
                    q = [q ; q_cell{i}];
                end
                syms t
                d_q = diff(q,t);
                out  = sym(zeros(length(d_q),1));
                for i = 1:n
                    for j = 1:n
                        out = out + obj.diff_scalar_vector(A(i,j),q)*d_q(i)*d_q(j);
                    end
                end
                %out = simplify(out,"Seconds",10);
            end

            
            function A_tk_xk = Get_A_tk_xk(obj,tk,xk)
            % ======================================================================
            %> @brief The function Get_A_tk_xk calculates the value of the A matrix in a \f$x_k\f$ configuration at time \f$t_k\f$.
            %>
            %> @param obj System
            %> @param tk time \f$t_k\f$
            %> @param xk configuration \f$x_k\f$
            % ======================================================================
                n = length(obj.q_variables);
                A_tk_xk = [zeros(n,n)];
                A_System_Cells = obj.A_Matrix_Fun_Handles;
                sz = size(A_System_Cells);
                A_aux = [];
                for i = 1:sz(1)
                    Body_A_Functions = A_System_Cells{i,2};
                    m = Body_A_Functions{2};
                    I =Body_A_Functions{3};
                    A_Function_Handle_Cells = Body_A_Functions{4};
                    sz2 = size(A_Function_Handle_Cells);
                        for i_2 = 1:sz2(1)
                            for j_2 = 1:sz2(2)
                                bc_i = A_Function_Handle_Cells{i_2,j_2}{1};
                                bc_j = A_Function_Handle_Cells{i_2,j_2}{2};
                                bc_i_k = bc_i(tk,xk);
                                bc_j_k = bc_j(tk,xk);
                                b_i_k = bc_i_k(4:6);
                                c_i_k = bc_i_k(1:3);
                                b_j_k = bc_j_k(4:6);
                                c_j_k = bc_j_k(1:3);
                                a_ij_term = m*dot(b_i_k,b_j_k)+dot((I*c_i_k),c_j_k);
                                A_aux(i_2,j_2)= a_ij_term;
                            end
                        end
                   A_tk_xk = A_tk_xk + A_aux;
                end
            end

            function d_A_tk_xk = Get_d_A_tk_xk(obj,tk,xk)
                n = length(obj.q_variables);
                d_A_tk_xk = [zeros(n,n)];
                A_System_Cells = obj.A_Matrix_Fun_Handles;
                sz = size(A_System_Cells);
                A_aux = [];
                for i = 1:sz(1)
                    Body_A_Functions = A_System_Cells{i,2};
                    m = Body_A_Functions{2};
                    I =Body_A_Functions{3};
                    d_A_Function_Handle_Cells = Body_A_Functions{5};
                    sz2 = size(d_A_Function_Handle_Cells);
                        for i_2 = 1:sz2(1)
                            for j_2 = 1:sz2(2)
                                bc_i = d_A_Function_Handle_Cells{i_2,j_2}{1};
                                bc_j = d_A_Function_Handle_Cells{i_2,j_2}{2};
                                bc_i_k = bc_i(tk,xk);
                                bc_j_k = bc_j(tk,xk);
                                b_i_k = bc_i_k(4:6);
                                c_i_k = bc_i_k(1:3);
                                b_j_k = bc_j_k(4:6);
                                c_j_k = bc_j_k(1:3);
                                a_ij_term = m*dot(b_i_k,b_j_k)+dot((I*c_i_k),c_j_k);
                                A_aux(i_2,j_2)= a_ij_term;
                            end
                        end
                   d_A_tk_xk = d_A_tk_xk + A_aux;
                end
            end
            
            
            function tau_tk_xk = Get_tau_tk_xk(obj,tk,xk)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                syms t
                q = obj.q_variables;
                d_q = diff(q,t);
                n = length(q);
                tau_tk_xk = [zeros(n,1)];
                Actions = obj.System_Actions;
                Virtual_Velocity_Components = obj.Virtual_Velocity_Components_fun_handles;
                sz = size(Actions);
                for i = 2:sz(1)
                    Aux_Tau = [];
                    Action_Name = Actions{i,1};
                    Solid_Name = Actions{i,2};
                    Action_Vector_in_G = obj.Get_Action_in_G(Action_Name);
                    [Action_Vector_in_G_uv,u,v] = obj.order_reduction(Action_Vector_in_G,q,d_q);
                    Action_Vector_in_G_uv_handle = odeFunction(Action_Vector_in_G_uv,[v;u]);
                    Cell_Find_Array = cellfun(cellfind(Solid_Name),Virtual_Velocity_Components(:,1));
                    Solid_Position = find(Cell_Find_Array,1);
                    Virtual_Velocity_Components_Solid = Virtual_Velocity_Components{Solid_Position,2};
                    for j = 1:n
                        bc_j = Virtual_Velocity_Components_Solid{j,3};
                        bc_j_tk_xk = bc_j(tk,xk);
                        Aux_Tau = [Aux_Tau;dot(bc_j_tk_xk,Action_Vector_in_G_uv_handle(tk,xk))];
                    end
                    tau_tk_xk = tau_tk_xk+ Aux_Tau;
                end
            end
                % ======================================================================
                %> @brief The function Get_input_tk_xk calculates the value of the generalized inputs (generalized forces of the inputs) in a \f$\left[x_k, x_{external_k}\right]^T\f$ configuration at time \f$t_k\f$. 
                %>
                %> @param obj System
                %> @param tk time \f$t_k\f$
                %> @param xk configuration \f$x_k\f$
                % ======================================================================
            function input_tk_xk = Get_input_tk_xk(obj,tk,xk,external_vars_k)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                syms t
                q = obj.q_variables;
                d_q = diff(q,t);
                n = length(q);
                input_tk_xk = [zeros(n,1)];
                Inputs = obj.System_Inputs;
                Virtual_Velocity_Components = obj.Virtual_Velocity_Components_fun_handles;
                sz = size(Inputs);
                external_vars = sym([]);
                for i = 2:sz(1)
                    external_vars = [external_vars; Inputs{i,4}];
                end
                for i = 2:sz(1)
                    Aux_Tau = [];
                    Input_Name = Inputs{i,1};
                    Solid_Name = Inputs{i,2};
                    Input_Vector_in_G = obj.Get_Input_in_G(Input_Name);
                    [Action_Vector_in_G_uv,u,v] = obj.order_reduction(Input_Vector_in_G,q,d_q);
                    Action_Vector_in_G_uv_handle = odeFunction(Action_Vector_in_G_uv,{[v;u;external_vars]});
                    Cell_Find_Array = cellfun(cellfind(Solid_Name),Virtual_Velocity_Components(:,1));
                    Solid_Position = find(Cell_Find_Array,1);
                    Virtual_Velocity_Components_Solid = Virtual_Velocity_Components{Solid_Position,2};
                    for j = 1:n
                        bc_j = Virtual_Velocity_Components_Solid{j,3};
                        bc_j_tk_xk = bc_j(tk,xk);
                        Aux_Tau = [Aux_Tau;dot(bc_j_tk_xk,Action_Vector_in_G_uv_handle(tk,[xk;external_vars_k]))];
                    end
                input_tk_xk = input_tk_xk+ Aux_Tau;
            end
            end
            %> @brief The function Lagrangian_Rigth_Hand_Side returns the vector \f$ \mathbf{b} =  -\frac{d}{dt} \left[ \mathbf{A} \right] \mathbf{\dot{q}} + \mathbf{\tau}\f$ of the \f$ \left[ \mathbf{A} \right] \mathbf{\ddot{q}} = \mathbf{b}\f$ system of equations.
            %>
            %>
            function [RHS, RHS_uv] = Lagrangian_Rigth_Hand_Side(obj)
                syms t
                RHS = -obj.dot_A_dot_q+obj.Get_System_Generalized_Actions;
                %RHS = simplify(RHS,"Seconds",10);
                n = length(RHS);
                system_RHS_functions_handle = cell(n,1);
                q = sym([]);
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                n = length(q_cell);
                for i = 1:n
                    q = [q ; q_cell{i}];
                end
                b_terms = string([]);
                for i = 1:n
                    b_terms(i) = "b_"+string(i);
                end
                d_q = diff(q,t);
                [RHS_uv,u,v] = obj.order_reduction(RHS,q,d_q);
                RHS_uv= [RHS_uv; zeros(size(RHS_uv))];
                obj.q = q;
                obj.uv = [u; v];
                obj.vu = [v; u];
            end

            function [INPUT_RHS, INPUT_RHS_uv] = Input_Lagrangian_Rigth_Hand_Side(obj)
                syms t
                Inputs = obj.System_Inputs;
                sz = size(Inputs);
                obj.external_vars = sym([]);
                for i = 2:sz(1)
                    obj.external_vars = [obj.external_vars; Inputs{i,4}];
                end
                INPUT_RHS = obj.Get_System_Generalized_Inputs;
                %RHS = simplify(RHS,"Seconds",10);
                n = length(INPUT_RHS);
                system_RHS_functions_handle = cell(n,1);
                q = sym([]);
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                n = length(q_cell);
                for i = 1:n
                    q = [q ; q_cell{i}];
                end
                b_terms = string([]);
                for i = 1:n
                    b_terms(i) = "b_"+string(i);
                end
                d_q = diff(q,t);
                [INPUT_RHS_uv,u,v] = obj.order_reduction(INPUT_RHS,q,d_q);
                INPUT_RHS_uv= [INPUT_RHS_uv; zeros(size(INPUT_RHS_uv))];
                obj.q = q;
                obj.uv = [u; v];
                obj.vu = [v; u];
            end
            function [A,B] = Linealized_Model(obj,x_op,input_op)
                %delta_dot(x) = A * delta_x + B * delta_tau

                % Linearized model A matrix caculation
                vars = [obj.v;obj.u;obj.external_vars];
                x = [obj.v;obj.u];
                external_vars = obj.external_vars;
                for i = 1:length(vars)
                    partial_A_partial_x_i = simplify(diff(obj.A_uv,vars(i)),'Steps',10);
                    partial_A_partial_x{i,1} = vars(i);
                    partial_A_partial_x{i,2} = partial_A_partial_x_i;
                    partial_A_partial_x{i,3} = odeFunction(partial_A_partial_x_i,[x;external_vars]);
                end

                for i = 1:length(vars)
                    partial_full_B_partial_x_i = simplify(diff(obj.b_uv+obj.input_b_uv+[zeros(size(obj.v));obj.v],vars(i)),'Steps',10);
                    partial_full_B_partial_x{i,1} = vars(i);
                    partial_full_B_partial_x{i,2} = partial_full_B_partial_x_i;
                    partial_full_B_partial_x{i,3} = odeFunction(partial_full_B_partial_x_i,[x;external_vars]);
                end

                v_uv_handle = odeFunction(obj.v,[x;external_vars]);
                A_xk = obj.Get_A_tk_xk(0,x_op);
                full_b = obj.Get_tau_tk_xk(0,x_op)+obj.Get_input_tk_xk(0,x_op,input_op);
                full_inv_A = inv([A_xk,zeros(size(A_xk)); ...
                                    zeros(size(A_xk)), eye(size(A_xk))]);
                full_b_uv = [full_b;v_uv_handle(0,[x_op;input_op])];
                grad_mat = [];
                for i = 1:length(vars)
                    partial_A_partial_x_i = partial_A_partial_x{i,3}(0,[x_op;input_op]);
                    partial_full_B_partial_x_i = partial_full_B_partial_x{i,3}(0,[x_op;input_op]);
                    grad_mat = [grad_mat,(-full_inv_A*partial_A_partial_x_i*full_inv_A)*full_b_uv + full_inv_A*partial_full_B_partial_x_i];
                end
                n_x = length(x);
                A = grad_mat(:,1:n_x);
                B = grad_mat(:,n_x+1:end);
            end
        end
        methods(Static,Access = public)
            function out = diff_scalar_vector(scalar, vector)
                n = length(vector);
                out = sym(zeros([n 1]));
                for i = 1:n
                    out(i) = diff(scalar,vector(i));
                end
            end
            function X = Back_Substitution(U, Y)
                N = length(Y);
                X = sym([zeros(size(Y))]);
                X(N) = Y(N)/U(N,N);
                for I = N-1:-1:1
                    S = Y(I);
                    for J = N:-1:I+1
                        S = S-U(I,J)*X(J);
                    end
                    X(I) = S / U(I,I);
                end
            end
            function Y = Forward_Substitution(L,B)
                N = length(B);
                Y = sym([zeros(size(B))]);
                Y(1) = B(1)/L(1,1);
                for I = 2:N
                    S = B(I);
                    for J = 1:I
                        S = S-L(I,J)*Y(J);
                    end
                    Y(I) = S / L(I,I);
                end  
            end
            %> @brief The function order_reduction gets a symbolic array with \f$ \mathbf{q} \f$ and \f$ \mathbf{\dot{q}} \f$ variables and gives an array with the change of variables \f$ \left[\mathbf{u},\mathbf{v} \right]^T = \left[\mathbf{q} ,\mathbf{\dot{q}} \right]^T\f$ 
            function [A_uv,u,v] = order_reduction(A,q,d_q)
                syms t
                n = length(q);
                u = sym([]);
                v = sym([]);
                for i = 1: length(q)
                    str = string(q(i));
                    u = [u; str2sym("u_"+str)];
                    v = [v; str2sym("v_"+str)];
                end
                d_u = diff(u,t,1);
                d_v = diff(v,t,1);
                A_uv = subs(A,[d_q], [v]);
                A_uv = subs(A_uv,[q], [u]);
            end
        end
        
    %% BASE
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            System_Canonical_Base
            
            
        end
        % PROTECTED
        properties(Access = protected)

        end

        % PUBLIC
        properties(Access = public)
            System_Bases
        end
      % METHODS

        methods(Static ,Access = private)
        %> @brief This function creates a Canonical base \f$ \left[ 1,0,0\right], \left[ 0,1,0\right], \left[ 0,0,1\right] \f$.
        %>
        %> This Canonical Base is needed for the first time creation of the Base table.
            function base_out = System_Create_Canonical_Base()
               base_out = Dynamic_Library.Classes.Base( ...
                    "Canonical", ...
                    "axis_labels",[str2sym("x");str2sym("y");str2sym("z")]);
            end
        end

        methods(Access = private)
         %> @brief The System_Create_Bases_Table function creates a table of all of the bases in the system.
        %>
        %> The table has 2 columns: Name(string variable) and Base (Base class variable). Each time the user introduces a base in the system the table is updated and a new base is added.
            function table_out = System_Create_Bases_Table(obj)
                sz = [0 2];
                varTypes = {'string','Dynamic_Library.Classes.Base'};
                varNames = {'Name','Base'};
                table_out = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames);
                table_out = [obj.System_Bases; {obj.System_Canonical_Base.Get_Info("Name"), obj.System_Canonical_Base}];
            end
        end
        % PROTECTED
        methods(Access = protected)

        end

        % PUBLIC
        methods(Access = public)
            % ======================================================================
            %> @brief Base introduction in System
            %>
            %> This function creates a Base object with the same inputs that the Base Class requires. This base is introduced in the System object.
            %>
            %> @param obj: System object
            %> @param Name
            %> @param axis_labels(optional): [3x1] symbolic array that contains the labels of the 3 vectors that represents the base. If this parameter is not specified the default axis_labels will be \f$\left[x,y,z\right]^T\f$.
            %> @param father_base(optional): The parent Base object defines the base from which the constructor will create a new Base object. From this parent base, a new base is generated through a simple rotation. The simple rotation can only be performed in the direction indicated by the right-hand rule for each of the vectors of the parent base. If father_base parameter is not defined, the default parent base is \f$[1,0,0]\f$, \f$[0,1,0]\f$, and \f$[0,0,1]\f$.
            %> @param axis_rotation(optional): The simple rotation can only be performed in the direction indicated by the right-hand rule for each of the vectors of the parent base. The axis_rotation parameter defines the rotation axis with a string "1", "2" or "3" defining the parent basis vector to use. If axis_rotation parameter is not defined, the default axis of rotation "1".
            %> @param angle_rotation(optional): Symbolic or double parameter that defines the angle of rotation (in radians) of the new base. If angle_rotation parameter is not defined the default angle of rotation is 0.
            %>
            %>
            % ======================================================================
            function R_Base_1_Base_2 = Create_New_Base(obj, Name, varargin)
                %CELLFIND Find string matches in a cell array. Used to
                %avoid having bases with the same name.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Bases(:,1));
                find_match_father_base = cellfun(cellfind("father_base"),varargin);
                if(dot(find_match_father_base,find_match_father_base) == 1)
                    Base_Name = varargin([find(find_match_father_base,1)]+1);
                    Base_Object = obj.Get_Base_Info("System_Bases",'base',Base_Name{:});
                    Base_Object = Base_Object{2};
                    varargin([find(find_match_father_base,1)]+1) = {Base_Object};
                end
                if any(find_match==1)
                    prompt = "Do you want to overwrite the current "+Name+" base? Y/N: ";
                    choice = string(input(prompt,"s"));
                    switch(choice)
                        case "Y"
                            obj.System_Bases([find(find_match,1)],:) = [];
                            Base = Dynamic_Library.Classes.Base(Name,varargin{:});
                            obj.System_Bases = [obj.System_Bases; {Base.Get_Info("Name"), Base}];
                        case "N"
                    end
                else
                    Base = Dynamic_Library.Classes.Base(Name,varargin{:});
                    obj.System_Bases = [obj.System_Bases; {Base.Get_Info("Name"), Base}];
                end
            end

            function out = Get_Base_Info(obj, Info, varargin)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                default_Base = "All";
                p = inputParser;
                addRequired(p,'info',@isstring);
                addParameter(p,'base',default_Base,@isstring);

                parse(p,Info,varargin{:});
                
                switch(p.Results.info)
                    case "System_Bases"
                        switch(p.Results.base)
                            case "All"
                                out = obj.System_Bases;
                            otherwise
                                row = cellfun(cellfind(p.Results.base),obj.System_Bases(:,1));
                                    if(dot(row,row)==0)
                                        error("The base "+p.Results.base +" is not in the System "+obj.System_Name+".")
                                    end
                                out = obj.System_Bases(row,:);
                        end
                    
                end              

            end
            % ======================================================================
            %> @brief Given some vector components in a Base_1 the Change_Basis_Matrix function gives the vector components in base Base_2.
            %>
            %> @param Base_1: Base object that represent the fixed frame
            %> @param Base_2: Base object that represent the moving frame
            % ======================================================================
            function R_Base_1_Base_2 = Change_Basis_Matrix(obj,Base_1, Base_2)
                Base_1_Obj = obj.Get_Base_Info("System_Bases",'base',Base_1);
                Base_2_Obj = obj.Get_Base_Info("System_Bases",'base',Base_2);
                R_Canonical_Base_1 = Base_1_Obj{2}.Get_Info("Base_Matrix_From_Canonical");
                R_Canonical_Base_2 = Base_2_Obj{2}.Get_Info("Base_Matrix_From_Canonical");
                R_Base_1_Base_2 = simplify(R_Canonical_Base_2*R_Canonical_Base_1.');
            end
            % ======================================================================
            %> @brief Angular_Velocity function gives the angular velocity vector from Base_1 to Base_2. The given vector components can be represented in the Base_Components base.
            %>
            %> @param Base_1: Base object that represent the fixed frame
            %> @param Base_2: Base object that represent the moving frame
            %> @param Base_Components: Base object in which the components will be given.
            % ======================================================================
            function out = Angular_Velocity(obj, Base_1, Base_2, Base_Components)
                syms t 
                S = transpose(obj.Change_Basis_Matrix(Base_1,Base_2));
                Ang_Vel_Matrix = simplify(S.'*diff(S,t),'Steps',100);
                Ang_Vel_Vector = [Ang_Vel_Matrix(3,2);Ang_Vel_Matrix(1,3);Ang_Vel_Matrix(2,1)];
                R = obj.Change_Basis_Matrix(Base_2,Base_Components);
                out = simplify(R*Ang_Vel_Vector,'Steps',100);
            end
        end
        methods(Static,Access = public)
            
        end

    %% POINT
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            System_Canonical_Point
            
        end
        % PROTECTED
        properties(Access = protected)

        end

        % PUBLIC
        properties(Access = public)
            System_Points
        end
      % METHODS

        methods(Static ,Access = private)
        %> @brief This function creates a Canonical Point \f$ \left[0,0,0\right] \f$.
        %>
        %> This Canonical Point is needed for the first time creation of the Points table.
            function point_out = System_Create_Canonical_Point()
                point_out = Dynamic_Library.Classes.Point("Canonical");
            end
        end

        methods(Access = private)
        %> @brief The System_Create_Points_Table function creates a table of all of the points in the system.
        %>
        %> The table has 2 columns: Name(string variable) and Point (Point class variable). Each time the user introduces a point in the system the table is updated and a new point is added.
            function table_out = System_Create_Points_Table(obj)
                sz = [0 2];
                varTypes = {'string','Dynamic_Library.Classes.Point'};
                varNames = {'Name','Point'};
                table_out = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames);
                table_out = [obj.System_Points; {obj.System_Canonical_Point.Get_Info("Name"), obj.System_Canonical_Point}];
            end
        end
        % PROTECTED
        methods(Access = protected)

        end

        % PUBLIC
        methods(Access = public)
            % ======================================================================
            %> @brief Point introduction in System
            %>
            %> This function creates a Point object with the same inputs that the Point Class requires. This base is introduced in the System object.
            %>
            %>
            %> @param Name: String that defines the name of the object
            %> @param point_coordinates: [3 x 1] dobule or symbolic array that defines the position of the point with respect the absolute point (canonical point [0,0,0]).
            %>
            %> The second way of creating a point is with respect a coordinate system. A coordinate system is defined by a base and a point, and the new point is given by its position with respect to the father point with the father_base components. 
            %> @param Name: String that defines the name of the object.
            %> @param father_point: Father Point (String).
            %> @param father_base: Father Base (String).
            %> @param point_coordinates: [3 x 1] dobule or symbolic array that defines the position of the point with respect the relative coordinate system (Father_Point and Father_Base).
            function Create_New_Point(obj, Name, Coordinate_System, Coordinates)
                %CELLFIND Find string matches in a cell array. Used to
                %avoid having points with the same name.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Points(:,1));
                Coordinate_System_Info = obj.Get_Coordinate_System_Info("System_Coordinate_Systems",'coordinate_system',Coordinate_System);
                Point_Name = Coordinate_System_Info{2};
                Base_Name = Coordinate_System_Info{3};
                Point_Object = obj.Get_Point_Info("System_Points",'point',Point_Name);
                Point_Object = Point_Object{2};
                Base_Object = obj.Get_Base_Info("System_Bases",'base',Base_Name);
                Base_Object = Base_Object{2};

                if any(find_match==1)
                    prompt = "Do you want to overwrite the current "+Name+" point? Y/N: ";
                    choice = string(input(prompt,"s"));
                    switch(choice)
                        case "Y"
                            obj.System_Points([find(find_match,1)],:) = [];
                            Point = Dynamic_Library.Classes.Point(Name, "father_base",Base_Object,"father_point",Point_Object,"point_coordinates",Coordinates);
                            obj.System_Points = [obj.System_Points; {Point.Get_Info("Name"), Point_Object}];
                        case "N"
                    end
                else
                    Point = Dynamic_Library.Classes.Point(Name, "father_base",Base_Object,"father_point",Point_Object,"point_coordinates",Coordinates);
                    obj.System_Points = [obj.System_Points; {Point.Get_Info("Name"), Point}];
                end
                
            end
            %> @brief The function Get_Two_Points_Vector returns the vector componenents from Start_Point to End_Point in the Canonical base.
            %>
            %> @param Start_Point The initial point.
            %> @param End_Point The end point.
            %> @retval out \f$ \left[ 3 \times 1 \right] \f$ position vector
            function out = Get_Two_Points_Vector(obj,Start_Point, End_Point)
                 Start_Point_Obj = obj.Get_Point_Info("System_Points",'point',Start_Point);
                 End_Point_Obj = obj.Get_Point_Info("System_Points",'point',End_Point);
                 Start_Point_Absolute_Coordinates = Start_Point_Obj{2}.Get_Info("Point_Coordinates_From_Canonical");
                 End_Point_Absolute_Coordinates = End_Point_Obj{2}.Get_Info("Point_Coordinates_From_Canonical");
                 out = simplify(End_Point_Absolute_Coordinates-Start_Point_Absolute_Coordinates);
            end
            %> @brief The Get_Two_Points_Vector_Base function returns the vector componenents from Start_Point to End_Point in any base.
            %>
            %> @param Start_Point The initial point.
            %> @param End_Point The end point.
            %> @param Base Components base (String).
            %> @retval out \f$ \left[ 3 \times 1 \right] \f$ position vector
            function out = Get_Two_Points_Vector_Base(obj,Start_Point, End_Point, Base)
                 r = obj.Get_Two_Points_Vector(Start_Point, End_Point);
                 Base_Obj = obj.Get_Base_Info("System_Bases",'base',Base);
                 M_B_E = Base_Obj{2}.Get_Info("Base_Matrix_From_Canonical");
                 out = simplify(M_B_E*r,'Steps',100);
            end

            function Get_Vector_Velocity(obj,Start_Point,End_Point,Derivation_Base,Components_Base)
                r = obj.Get_Two_Points_Vector(Start_Point, End_Point);
                Base_Obj = obj.Get_Base_Info("System_Bases",'base',Base);
                M_B_E = Base_Obj{2}.Get_Info("Base_Matrix_From_Canonical");
            end
            function out = Get_Point_Info(obj, Info, varargin)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                default_Point = "All";
                p = inputParser;
                addRequired(p,'info',@isstring);
                addParameter(p,'point',default_Point,@isstring);
                parse(p,Info,varargin{:});
                
                switch(p.Results.info)
                    case "System_Points"
                        switch(p.Results.point)
                            case "All"
                                out = obj.System_Points;
                            otherwise
                                row = cellfun(cellfind(p.Results.point),obj.System_Points(:,1));
                                    if(dot(row,row)==0)
                                        error("The point "+p.Results.point +" is not in the System "+obj.System_Name+".")
                                    end
                                out = obj.System_Points(row,:);
                        end
                    
                end              

            end

        end
    %% COORDINATE SYSTEM
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            System_Canonical_Coordinate_System
            
            
        end
        % PROTECTED
        properties(Access = protected)

        end
        % PUBLIC
        properties(Access = public)
            System_Coordinate_Systems
        end
      % METHODS
        methods(Access = private)
        %> @brief This function creates a Canonical Coordinate System with the Canonical Base and the Canonical Point.
        %>
        %> This Canonical Coordinate System is needed for the first time creation of the Coordinate System table.
            function coordinate_system_out = System_Create_Canonical_Coordinate_System(obj)            
                System_Canonical_Point_Name = obj.System_Canonical_Point.Get_Info("Name");
                System_Canonical_Base_Name = obj.System_Canonical_Base.Get_Info("Name");
                coordinate_system_out = {"Canonical", ... 
                                         System_Canonical_Point_Name, ...
                                         System_Canonical_Base_Name};
            end
        %> @brief The System_Create_Coordinate_System_Table function creates a table of all of the coordinate systems in the system.
          %>
          %> The table has 2 columns: Point_Name(string variable) and Base_Name (string variable). Each time the user introduces a new coordinate system in the system the table is updated and a new coordinate system is added.
            function table_out = System_Create_Coordinate_System_Table(obj)
                sz = [0 2];
                varTypes = {'string','string'};
                varNames = {'Point_Name','Base_Name'};
                table_out = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames);
                table_out = [obj.System_Coordinate_Systems; obj.System_Canonical_Coordinate_System];
            end
        end
        % PROTECTED
        methods(Access = protected)

        end
        % PUBLIC
        methods(Access = public)
            % ======================================================================
            %> @brief The Create_New_Coordinate_System function introduces a new coordinate system in the System object. This Coordinte System is given by a Point and a Base object.
            %>
            %> @param Name: Name given to the new coordinate system.
            %> @param Point_Name: Point Name (String) of the Point used to create the new coordinate system.
            %> @param Base_Name: Base Name (String) of the Base used to create the new coordinate system.
            % ======================================================================
            function Create_New_Coordinate_System(obj, Name, Point_Name, Base_Name)
                %CELLFIND Find string matches in a cell array. Used to
                %avoid having Cordinate_Systems with the same name.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Coordinate_Systems(:,1));
                find_match_point = cellfun(cellfind(Point_Name),obj.System_Points(:,1));
                find_match_base = cellfun(cellfind(Base_Name),obj.System_Bases(:,1));
                if any(find_match==1)
                    prompt = "Do you want to overwrite the current "+Name+" Cordinate_System? Y/N: ";
                    choice = string(input(prompt,"s"));
                    switch(choice)
                        case "Y"
                            Point_Cells = obj.System_Points([find(find_match_point,1)],1);
                            Base_Cells = obj.System_Bases([find(find_match_base,1)],1);
                            obj.System_Coordinate_Systems = [obj.System_Coordinate_Systems; {Name, Point_Name, Base_Name}];
                        case "N"
                    end
                else
                    Point_Cells = obj.System_Points([find(find_match_point,1)],1);
                    Base_Cells = obj.System_Bases([find(find_match_base,1)],1);
                    obj.System_Coordinate_Systems = [obj.System_Coordinate_Systems; {Name, Point_Name, Base_Name}];
                end
            end
            function out = Get_Coordinate_System_Info(obj, Info, varargin)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                default_Coordinate_System = "All";
                p = inputParser;
                addRequired(p,'info',@isstring);
                addParameter(p,'coordinate_system',default_Coordinate_System,@isstring);
                
                parse(p,Info,varargin{:});
                
                switch(p.Results.info)
                    case "System_Coordinate_Systems"
                        switch(p.Results.coordinate_system)
                            case "All"
                                out = obj.System_Coordinate_Systems;
                            otherwise
                                row = cellfun(cellfind(p.Results.coordinate_system),obj.System_Coordinate_Systems(:,1));
                                    if(dot(row,row)==0)
                                        error("The Coordinate System "+p.Results.coordinate_system +" is not in the System "+obj.System_Name+".")
                                    end
                                out = obj.System_Coordinate_Systems(row,:);
                        end
                    
                end

            end
        end
    %%  RIGID BODY
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            System_Canonical_Rigid_Body
            
            
        end
        % PROTECTED
        properties(Access = protected)

        end
        % PUBLIC
        properties(Access = public)
            System_Rigid_Bodies
        end
      % METHODS
      methods(Static,Access = private)
        %> @brief This function creates a Canonical Rigid Body (all default settings).
        %>
        %> This Canonical Rigid Body is needed for the first time creation of the Rigid Bodies table.
          function rigid_body_out = System_Create_Canonical_Rigid_Body()
            rigid_body_out = Dynamic_Library.Classes.Rigid_Body("Canonical");
          end
      end
      methods(Access = private)
        %> @brief The System_Create_Rigid_Body_Table function creates a table of all of the points in the system.
        %>
        %> The table has 4 columns: Rigid_Body_Name(string variable), Point_Name (string variable), Base_Name(string variable) and Rigid_Body(Rigid_Body class variable) . Each time the user introduces a rigid body in the system the table is updated and a new rigid body is added.
            function table_out = System_Create_Rigid_Body_Table(obj)
                sz = [0 4];
                varTypes = {'string','string','string','Dynamic_Library.Classes.Rigid_Body'};
                varNames = {'Rigid_Body_Name','Point_Name','Base_Name','Rigid_Body'};
                table_out = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames);
                table_out = [obj.System_Rigid_Bodies; {obj.System_Canonical_Rigid_Body.Get_Info("Name"), "Canonical","Canonical",obj.System_Canonical_Rigid_Body}];
            end
        end
        % PROTECTED
        methods(Access = protected)

        end
        % PUBLIC
        methods(Access = public)
            % ======================================================================
            %> @brief Create_New_Rigid_Body is a function uses the methods of the class Rigid_Body to create a Rigid Body object.
            %>
            %> @param name: String that defines the name of the rigid body.
            %> @param G_Point: Center of mass of the rigid body from the canonical point OG. Symbolic or double [3 x 1] array.
            %> @param Base: Base object that defines the kinematic base linked with the rigid body.
            %> @param Mass: Symbolic or double scalar that represents the total mass of the rigid body.
            %> @param Inertial_Tensor: [3 x 3] symbolic or double variable that represents the inertial tensor of the rigid body. A zero [3 x 3] array represents a particle.
            % ======================================================================
            function Create_New_Rigid_Body(obj, Name, varargin)
                %CELLFIND Find string matches in a cell array. Used to
                %avoid having Cordinate_Systems with the same name.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Rigid_Bodies(:,1));

                default_G_Point = "Canonical";
                default_Rigid_Body_Base = "Canonical";
                default_Mass = str2sym("m_"+Name);
                default_Inertial_Tensor = sym([]);
                default_Inertial_Tensor = [default_Inertial_Tensor; "I_"+Name+"_"+string(1)+"1", "I_"+Name+"_"+string(1)+"2", "I_"+Name+"_"+string(1)+"3"];
                default_Inertial_Tensor = [default_Inertial_Tensor; "I_"+Name+"_"+string(1)+"2", "I_"+Name+"_"+string(2)+"2", "I_"+Name+"_"+string(2)+"3"];
                default_Inertial_Tensor = [default_Inertial_Tensor; "I_"+Name+"_"+string(1)+"3", "I_"+Name+"_"+string(2)+"3", "I_"+Name+"_"+string(3)+"3"];
                
                ispoint = @(x) isequal(class(x),"Dynamic_Library.Classes.Point");
                issym = @(x) isequal(class(x),"sym");
                issym_3_1 = @(x) isequal(class(x),"sym") && isequal(size(x),[3 1]);
                issym_or_base = @(x) issym(x) || isequal(class(x),"Dynamic_Library.Classes.Base");
                issym_or_numeric = @(x) issym(x) || isnumeric(x);

                p = inputParser;
                addRequired(p,'name',@isstring);
                addParameter(p,'G_Point',default_G_Point,@isstring);
                addParameter(p,'Base',default_Rigid_Body_Base,@isstring);
                addParameter(p,'Mass',default_Mass,issym_or_numeric);
                addParameter(p,'Inertial_Tensor',default_Inertial_Tensor,issym_or_numeric);

                parse(p,Name,varargin{:});

                find_match_point = cellfun(cellfind(p.Results.G_Point),obj.System_Points(:,1));
                find_match_base = cellfun(cellfind(p.Results.Base),obj.System_Bases(:,1));
                if any(find_match==1)
                    prompt = "Do you want to overwrite the current "+Name+" rigid body? Y/N: ";
                    choice = string(input(prompt,"s"));
                    switch(choice)
                        case "Y"
                            Point_Object = obj.System_Points([find(find_match_point,1)],1);
                            Base_Object = obj.System_Bases([find(find_match_base,1)],1);
                            Rigid_Body = Dynamic_Library.Classes.Rigid_Body(Name, ...,
                                                                    'G_Point',Point_Object{:},...
                                                                    'Base', Base_Object{:}, ...
                                                                    'Mass', p.Results.Mass, ...
                                                                    'Inertial_Tensor',p.Results.Inertial_Tensor);
                            obj.System_Rigid_Bodies = [obj.System_Rigid_Bodies; {Name, p.Results.G_Point, p.Results.Base, Rigid_Body}];
                        case "N"
                    end
                else
                    Point_Object = obj.System_Points([find(find_match_point,1)],2);
                    Base_Object = obj.System_Bases([find(find_match_base,1)],2);
                    Rigid_Body = Dynamic_Library.Classes.Rigid_Body(Name, ...,
                                                                    'G_Point',Point_Object{:},...
                                                                    'Base', Base_Object{:}, ...
                                                                    'Mass', p.Results.Mass, ...
                                                                    'Inertial_Tensor',p.Results.Inertial_Tensor);
                    obj.System_Rigid_Bodies = [obj.System_Rigid_Bodies; {Name, p.Results.G_Point, p.Results.Base, Rigid_Body}];
                end
            end
            function out = Get_Rigid_Body_Info(obj, Info, varargin)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                default_Rigid_Body = "All";
                p = inputParser;
                addRequired(p,'info',@isstring);
                addParameter(p,'rigid_body',default_Rigid_Body,@isstring);
                
                parse(p,Info,varargin{:});
                
                switch(p.Results.info)
                    case "System_Rigid_Bodies"
                        switch(p.Results.rigid_body)
                            case "All"
                                out = obj.System_Rigid_Bodies;
                            otherwise
                                row = cellfun(cellfind(p.Results.rigid_body),obj.System_Rigid_Bodies(:,1));
                                    if(dot(row,row)==0)
                                        error("The Rigid_Body "+p.Results.rigid_body +" is not in the System "+obj.System_Name+".")
                                    end
                                out = obj.System_Rigid_Bodies(row,:);
                        end
                    case "Kinetic_Energy"
                        switch(p.Results.rigid_body)
                            case "All"
                                    out = obj.System_Rigid_Bodies;
                                otherwise
                                    row = cellfun(cellfind(p.Results.rigid_body),obj.System_Rigid_Bodies(:,1));
                                        if(dot(row,row)==0)
                                            error("The Rigid_Body "+p.Results.rigid_body +" is not in the System "+obj.System_Name+".")
                                        end
                                    out = simplify(obj.Get_Kinetic_Energy_Quadratic_Matrix(p.Results.rigid_body),"Seconds",5);
                        end
                end
                
            end
            %> @brief The Get_Virtual_Velocity_q_Component function provides the \f$ \left[ \mathbf{c_k^i, b_k^i}\right]^T \f$ array given by the Get_Info function of the Rigid_Body class.
            %> 
            %> @param Name Name of the rigid body.
            %> @param q Generalized coordinate
            function out = Get_Virtual_Velocity_q_Component(obj,Name,q)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                Rigid_Body_Name = Name;
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Rigid_Body_Object = Rigid_Body_Cell{4};
                out = Rigid_Body_Object.Get_Info("Virtual_Velocities",'Generalized_Coordinate',q);
            end
            %> @brief The Get_Virtual_Velocity_q_uv_handle function provides the \f$ \left[ \mathbf{c_k^i, b_k^i}\right]^T \f$ array given by the Get_Info function of the Rigid_Body class in the variables \f$ \left[\mathbf{u}, \mathbf{v} \right]^T \f$ of the order reduction
            %> 
            %>
            %> This function saves for each of the generalized coordinates in the system a function in the folder fun_handles in the work directory.
            %>
            %> @param Name Name of the rigid body.
            function [virtual_velocity_uv, vars] = Get_Virtual_Velocity_q_uv_handle(obj,Name)
                syms t
                q = obj.q_variables;
                d_q = diff(q,t);
                virtual_velocity_uv = {};
                for i = 1:length(q)
                    vv = obj.Get_Virtual_Velocity_q_Component(Name,q(i));
                    dt_vv = simplify(diff(vv,t),'Steps',100);
                    [vv_uv,u,v] = obj.order_reduction(vv,q,d_q);
                    d_vv_uv = obj.order_reduction(dt_vv,q,d_q);
                    virtual_velocity_uv{i,1} = string(q(i));
                    virtual_velocity_uv{i,2} = vv;
                    % virtual_velocity_uv{i,3} = odeFunction(vv_uv,[v;u]);
                    virtual_velocity_uv{i,3} = odeFunction(vv_uv,[v;u], ...
                                                "File", "fun_handles/Virtual_Velocity_Components_uv_"+Name+"_"+string(i), ...
                                                "Comments","Virtual Velocity Components: Rigid Body: "+Name+" "+ string(q(i)));
                    virtual_velocity_uv{i,4} = dt_vv;
                    % virtual_velocity_uv{i,5} = odeFunction(d_vv_uv,[v;u]);
                    virtual_velocity_uv{i,5} = odeFunction(d_vv_uv,[v;u], ...
                                                "File", "fun_handles/d_Virtual_Velocity_Components_uv_"+Name+"_"+string(i), ...
                                                "Comments","Virtual Velocity Components: Rigid Body: "+Name+" "+ string(q(i)));
                    vars = [v;u];
                end
            end
            function [EOM_Cell, vars] = Get_A_Matrix_Handle(obj, Name)
            %> @brief The function Get_A_Matrix_Handle creates a \f$ \left[ 1 \times 5\right]\f$ cell matrix for one rigid body in the system and helps the calculation of the \f$ \left[ A \right]\f$ values. The cell arrays columns are: name of the rigid body (string), mass of the rigid body (double number), inertial tensor of the rigid body (\f$ \left[ 3 \times 3\right]\f$ double array), \f$ \left[ n \times n\right]\f$ cell array used to get the \f$ \left[ A \right]\f$ matrix and a \f$ \left[ n \times n\right]\f$ cell array used to get the \f$ \dot{\left[ A \right]}\f$ matrix. The value of \f$ n\f$ is the number of generalized coordinates in the system (the degrees of freedom in a holonomic system).
            %>
            %> @param Name Name of the rigid body (string)
            %>
            %> Each of the components of the \f$ \left[ n \times n\right]\f$ cell array contains the function handles to get the numeric values of the \f$ \left[ A \right]\f$ and \f$ \dot{\left[ A \right]} \f$ matrices. For example, the first component (1,1) of the \f$ \left[ A \right]\f$ matrix of a rigid body is gonna be \f$ -m \mathbf{b_1}^T \mathbf{b_1} - \left[ \mathbf{I} \mathbf{c_1} \right]^T \mathbf{c_1} \f$. In this expression \f$ m\f$ is the mass of the rigid body, \f$ \mathbf{I}\f$ is the inertial tensor of the rigid body and \f$ \mathbf{b_1}\f$ and \f$  \mathbf{c_1}\f$ are the virtual displacements of the rigid body with respect to the first generalized coordinate. The \f$ \left[ n \times n\right]\f$ cell array gives the handles to calculate the virtual displacements \f$ \mathbf{b_1}\f$ and \f$ \mathbf{c_1}\f$ and in which order the product should be done. With this handles, the mass, and the inertial tensor, the numerical (1,1) component of the \f$ \left[ A \right]\f$ matrix can be calculated. Analogous methodology is used for the other components and also for the \f$ \dot{\left[ A \right]} \f$ matrix aswell.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                Rigid_Body_Name = Name;
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Rigid_Body_Object = Rigid_Body_Cell{4};
                I = Rigid_Body_Object.Get_Info("Inertial_Tensor");
                Mass = Rigid_Body_Object.Get_Info("Mass");
                [virtual_velocity_uv, vars] = obj.Get_Virtual_Velocity_q_uv_handle(Name);
                sz = size(virtual_velocity_uv);
                A_Matrix_Handle = {};
                d_A_Matrix_Handle = {};
                virtual_velocity_uv_handles = virtual_velocity_uv(:,3);
                d_virtual_velocity_uv_handles = virtual_velocity_uv(:,5);
                for i = 1:sz(1)
                    for j=1:sz(1)
                        A_Matrix_Handle{i,j} = {virtual_velocity_uv_handles{i};virtual_velocity_uv_handles{j}};
                        d_A_Matrix_Handle{i,j} = {d_virtual_velocity_uv_handles{i};virtual_velocity_uv_handles{j}};
                    end
                end
                EOM_Cell = {Name,Mass,I,A_Matrix_Handle,d_A_Matrix_Handle};
            end
            function [system_virtual_velocity_uv, vars] = Get_System_Virtual_Velocity_q_uv_handle(obj)
                Rigid_Bodies = obj.System_Rigid_Bodies(:,1);
                n = length(Rigid_Bodies);
                system_virtual_velocity_uv = {};
                for i = 2:n
                    Name = Rigid_Bodies{i};
                    Virtual_Velocity_Component_Handle = obj.Get_Virtual_Velocity_q_uv_handle(Name);
                    system_virtual_velocity_uv{i-1,1} = Name;
                    system_virtual_velocity_uv{i-1,2} = Virtual_Velocity_Component_Handle;
                end
            end
            function [EOM_Cell, vars] = Get_System_A_Matrix_Handles(obj)
            %> @brief The function Get_System_A_Matrix_Handles creates a \f$ \left[ n_{RB} \times 2\right]\f$ cell matrix, where \f$ n_{RB}\f$ is the number of rigid bodies in the system. This cell arrays give the function handles and parameters to calculate the \f$ \left[ A \right]\f$ and \f$ \dot{\left[ A \right]}\f$ matrices. This matrices represent the contribution of the inertial forces to the EOM.
            %>
            %> @param obj System
                Rigid_Bodies = obj.System_Rigid_Bodies(:,1);
                n = length(Rigid_Bodies);
                system_virtual_velocity_uv = {};
                for i = 2:n
                    Name = Rigid_Bodies{i};
                    A_Matrix_Handle = obj.Get_A_Matrix_Handle(Name);
                    EOM_Cell{i-1,1} = Name;
                    EOM_Cell{i-1,2} = A_Matrix_Handle;
                end
            end
            %> @brief The function Get_Transformation_Matrix_Handle gives the function handle for the calculation of the Transformation Matrix (T) of one of the rigid bodies in the system.
            %>
            %> This transformation matrix is given in the reduced order variables \f$ \left[\mathbf{u}, \mathbf{v} \right]^T \f$.
            %>
            %> @param Name
            %> @retval T_uv Symbolic Transformation Matrix (T).
            %> @retval T_fun_handle Function handle of the Transformation Matrix (T).
            function [T_uv,T_fun_handle] = Get_Transformation_Matrix_Handle(obj,Name)
                Rigid_Body = obj.Get_Rigid_Body_Info("System_Rigid_Bodies",'rigid_body',Name);
                Point_Name = Rigid_Body{2};
                Point = obj.Get_Point_Info("System_Points",'point',Point_Name);
                Point_obj = Point{2};
                Base_Name = Rigid_Body{3};
                Base = obj.Get_Base_Info("System_Bases",'base',Base_Name);
                Base_obj = Base{2};
                r = Point_obj.Get_Info("Point_Coordinates_From_Canonical");
                R = Base_obj.Get_Info("Base_Matrix_From_Canonical");
                T = [[R.'; 0 0 0], [r;1] ];
                q = obj.q_variables;
                syms t
                d_q = diff(q,t);
                [T_uv,u,v] = obj.order_reduction(T,q,d_q);
                T_fun_handle = odeFunction(T_uv,[v;u]);
            end
            %> @brief The function Get_System_Transformation_Matrix_Handle returns a \f$ \left[ n_{RB} \times 3\right]\f$ array of cells that contains the Symbolic matrices and function handles of all of the rigid bodies transformation matrices in the system, where \f$ n_{RB} \f$ is the number of rigid bodies in the system.
            %>
            %> The first column of the cells represent the name (string) of each rigid body, the second contains the Symbolic Transformation Matrix and the third one keeps the function handle for the numerical calculation of the transformation matrix.
            function T_Cells = Get_System_Transformation_Matrix_Handle(obj)
                Rigid_Bodies = obj.System_Rigid_Bodies(:,1);
                n = length(Rigid_Bodies);
                Transformation_Matrix = {};
                for i = 2:n
                    Name = Rigid_Bodies{i};
                    [T_Matrix, T_Matrix_Handle] = obj.Get_Transformation_Matrix_Handle(Name);
                    T_Cells{i-1,1} = Name;
                    T_Cells{i-1,2} = T_Matrix;
                    T_Cells{i-1,3} = T_Matrix_Handle;
                end
            end
            function [A,A_uv] = Get_Kinetic_Energy_Quadratic_Matrix(obj,Name)
                %> @brief The function Get_Kinetic_Energy_Quadratic_Matrix gets the \f$ \left[ A \right]\f$ matrix of a rigid body in the system.
                %> This function uses the Get_Info function of the Rigid_Body class with the A_Matrix option to get the A matrix.

                %> @param Name Name of the rigid body.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                Rigid_Body_Name = Name;
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Rigid_Body_Object = Rigid_Body_Cell{4};
                % K = Rigid_Body_Object.Get_Info("Kinetic_Energy");
                %Get Generalized Coordintes of System
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                n = length(q_cell);
                q = sym([]);
                for i = 1:n
                    q = [q ; q_cell{i}];
                end
                A = Rigid_Body_Object.Get_Info("A_Matrix",'q_vect',q);
                syms t
                d_q = diff(q,t);
                % A = obj.Get_Bilineal_Matrix(K,d_q);
                [A_uv,u,v] = obj.order_reduction(A,q,d_q);
                obj.u = u;
                obj.v = v;
            end

            function [d_A,d_A_uv] = Get_d_Kinetic_Energy_Quadratic_Matrix(obj,Name)
                %> @brief The function Get_d_Kinetic_Energy_Quadratic_Matrix gets the \f$ \dot{\left[ A \right]}\f$ matrix of a rigid body in the system.
                %> This function uses the Get_Info function of the Rigid_Body class with the d_A_Matrix option to get the A matrix.

                %> @param Name Name of the rigid body.
                %Get Rigid Body Object
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                Rigid_Body_Name = Name;
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Rigid_Body_Object = Rigid_Body_Cell{4};
                % K = Rigid_Body_Object.Get_Info("Kinetic_Energy");
                %Get Generalized Coordintes of System
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                n = length(q_cell);
                q = sym([]);
                for i = 1:n
                    q = [q ; q_cell{i}];
                end
                d_A = Rigid_Body_Object.Get_Info("d_A_Matrix",'q_vect',q);
                syms t
                d_q = diff(q,t);
                % A = obj.Get_Bilineal_Matrix(K,d_q);
                [d_A_uv,u,v] = obj.order_reduction(d_A,q,d_q);
            end

            function [A,A_uv] = Get_System_Kinetic_Energy_Quadratic_Matrix(obj)
                %> @brief The function Get_System_Kinetic_Energy_Quadratic_Matrix gets the \f$ \left[ A \right]\f$ matrix of the whole system.
                %> This function uses the Get_Kinetic_Energy_Quadratic_Matrix function and the system rigid body table to merge all the individual matrices.
                Rigid_Bodies = obj.System_Rigid_Bodies(:,1);
                n = length(Rigid_Bodies);
                aux = obj.Get_Kinetic_Energy_Quadratic_Matrix(Rigid_Bodies{2});
                sz = size(aux);
                Kinetic_Energy_Rigid_Body_Matrices = sym(zeros(sz));
                A_uv = sym(zeros(sz));
                for i = 2:n
                    [A_aux, A_uv_aux] = obj.Get_Kinetic_Energy_Quadratic_Matrix(Rigid_Bodies{i});
                    A_uv = A_uv + A_uv_aux;
                    Kinetic_Energy_Rigid_Body_Matrices = Kinetic_Energy_Rigid_Body_Matrices+ A_aux;
                end
                A = Kinetic_Energy_Rigid_Body_Matrices;
            end

            function [d_A,d_A_uv] = Get_System_d_Kinetic_Energy_Quadratic_Matrix(obj)
                %> @brief The function Get_System_Kinetic_Energy_Quadratic_Matrix gets the \f$\dot{\left[ A \right]}\f$ matrix of the whole system.
                %> This function uses the Get_d_Kinetic_Energy_Quadratic_Matrix function and the system rigid body table to merge all the individual matrices.
                Rigid_Bodies = obj.System_Rigid_Bodies(:,1);
                n = length(Rigid_Bodies);
                aux = obj.Get_d_Kinetic_Energy_Quadratic_Matrix(Rigid_Bodies{2});
                sz = size(aux);
                Kinetic_Energy_Rigid_Body_Matrices = sym(zeros(sz));
                d_A_uv = sym(zeros(sz));
                for i = 2:n
                    [A_aux, A_uv_aux] = obj.Get_d_Kinetic_Energy_Quadratic_Matrix(Rigid_Bodies{i});
                    d_A_uv = d_A_uv + A_uv_aux;
                    Kinetic_Energy_Rigid_Body_Matrices = Kinetic_Energy_Rigid_Body_Matrices+ A_aux;
                end
                d_A = Kinetic_Energy_Rigid_Body_Matrices;
            end
        end

        methods(Static,Access = public)
            function A = Get_Bilineal_Matrix(K, d_q)
                A = sym(zeros(length(d_q),length(d_q)));
                for i = 1:length(d_q)
                    for j = 1:length(d_q)
                            A(i,j) = diff(diff(K,d_q(i)),d_q(j));
                    end
                end
            end
        end
    %%  ACTION
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            System_Canonical_Action
        end
        % PROTECTED
        properties(Access = protected)

        end
        % PUBLIC
        properties(Access = public)
            System_Actions
        end
      % METHODS
      methods(Access = private)
      %> @brief This function creates a null action \f$ \left[ 0,0,0,0,0,0 \right] \f$ named Canonical.
      %>
      %> This Canonical Action is needed for the first time creation of the Action table.
          function action_out = System_Create_Canonical_action(obj)
            action_out = Dynamic_Library.Classes.Action("Canonical", ...
                                                         'Point',obj.System_Canonical_Point, ...
                                                         'Base',obj.System_Canonical_Base, ...
                                                         'Vector',sym([0;0;0;0;0;0]));
          end
          %> @brief The System_Create_Action_Table function creates a table of all of the external Actions in the system.
          %>
          %> The table has 3 columns: Action_Name(string variable), Rigid_Body (Rigid_Body class variable) and Action(Action class variable). Each time the user introduces a new Action in the system the table is updated and a new Action is added.
          function table_out = System_Create_Action_Table(obj)
                sz = [0 3];
                varTypes = {'string','string','Dynamic_Library.Classes.Action'};
                varNames = {'Action_Name','Rigid_Body','Action'};
                table_out = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames);
                table_out = [obj.System_Actions; {obj.System_Canonical_Action.Get_Info("Name"), "Canonical", obj.System_Canonical_Action}];
            end
        end
        % PROTECTED
        methods(Access = protected)

        end
        % PUBLIC
        methods(Access = public)
            % ======================================================================
            %> @brief The Create_New_Action function introduces a new action in the current system.
            %>
            %> @param obj: System in which to introduce a new action
            %> @param Name: Name of the new action
            %> @param Point: Point of application of the action. The point is given by its String Name and has to be introduced previously in the System.
            %> @param Base: Base in which the action components are given. The Base is given by its String Name and has to be introduced previously in the System.
            %> @param Rigid_Body: Rigid Body to which the action applies. The Rigid_Body is given by its String Name and has to be introduced previously in the System.
            %> @param Vector: Components of the Action. The action components can me symbolic or double and they are given by 3 torque components and 3 force components. Eg: [Mx;My;Mz;Fx;Fy;Fz].
            % ======================================================================
            function Create_New_Action(obj, Name, varargin)
                %CELLFIND Find string matches in a cell array. Used to
                %avoid having Cordinate_Systems with the same name.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Actions(:,1));

                default_Point = "Canonical";
                default_Base = "Canonical";
                default_Rigid_Body = "Canonical";
                default_Vector = sym([0;0;0;0;0;0]);
                
                issym = @(x) isequal(class(x),"sym");
                issym_6_1 = @(x) isequal(class(x),"sym") && isequal(size(x),[6 1]);
                issym_or_base = @(x) issym(x) || isequal(class(x),"Dynamic_Library.Classes.Base");

                p = inputParser;
                addRequired(p,'name',@isstring);
                addParameter(p,'Point',default_Point,@isstring);
                addParameter(p,'Base',default_Base,@isstring);
                addParameter(p,'Rigid_Body',default_Rigid_Body,@isstring);
                addParameter(p,'Vector',default_Vector,issym_6_1);

                parse(p,Name,varargin{:});

                find_match_point = cellfun(cellfind(p.Results.Point),obj.System_Points(:,1));
                find_match_base = cellfun(cellfind(p.Results.Base),obj.System_Bases(:,1));
                if any(find_match==1)
                    prompt = "Do you want to overwrite the current "+Name+" action? Y/N: ";
                    choice = string(input(prompt,"s"));
                    switch(choice)
                        case "Y"
                            Point_Object = obj.System_Points([find(find_match_point,1)],2);
                            Base_Object = obj.System_Bases([find(find_match_base,1)],2);
                            Action = Dynamic_Library.Classes.Action(p.Results.name, ...
                                                                            'Point',Point_Object{:},...
                                                                            'Base',Base_Object{:},...
                                                                            'Vector',p.Results.Vector);
                            obj.System_Actions = [obj.System_Actions; {Name, p.Results.Rigid_Body, Action}];
                        case "N"
                    end
                else
                    Point_Object = obj.System_Points([find(find_match_point,1)],2);
                    Base_Object = obj.System_Bases([find(find_match_base,1)],2);
                    Action = Dynamic_Library.Classes.Action(p.Results.name, ...
                                                                    'Point',Point_Object{:},...
                                                                    'Base',Base_Object{:},...
                                                                    'Vector',p.Results.Vector);
                    obj.System_Actions = [obj.System_Actions; {Name, p.Results.Rigid_Body, Action}];
                end
            end
            function out = Get_Action_Info(obj, Info, varargin)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                default_Action = "All";
                p = inputParser;
                addRequired(p,'info',@isstring);
                addParameter(p,'action',default_Action,@isstring);
                
                parse(p,Info,varargin{:});
                
                switch(p.Results.info)
                    case "System_Actions"
                        switch(p.Results.action)
                            case "All"
                                out = obj.System_Actions;
                            otherwise
                                row = cellfun(cellfind(p.Results.action),obj.System_Actions(:,1));
                                    if(dot(row,row)==0)
                                        error("The Action "+p.Results.action +" is not in the System "+obj.System_Name+".")
                                    end
                                out = obj.System_Actions(row,:);
                        end              
                end
            end
            function out = Get_Action_in_G(obj,Name)
                %> @brief Get_Action_in_G is a function that takes the Name of an Action in the System and gives the analogous Action in the center of mass G of the rigid body.
                %> @param Name Name of the action.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Actions(:,1));
                Action_Cell = obj.System_Actions([find(find_match,1)],:);
                Rigid_Body_Name = Action_Cell{2};
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Action = Action_Cell{3};
                G_Name = Rigid_Body_Cell{2};
                find_match_point = cellfun(cellfind(G_Name),obj.System_Points(:,1));
                G_Point_Object = obj.System_Points([find(find_match_point,1)],2);
                G_Point_Coordinates = G_Point_Object{:}.Get_Info("Point_Coordinates_From_Canonical");
                Action_Point_Coordinates = Action.Action_Point.Get_Info("Point_Coordinates_From_Canonical");
                GP = Action_Point_Coordinates-G_Point_Coordinates;
                Action_Vector_in_G = [Action.Action_Vector(1:3)+cross(GP,Action.Action_Vector(4:end)); Action.Action_Vector(4:end)];
                out = Action_Vector_in_G;
            end
            function out = Get_Generalized_Action(obj,Name)
                %> @brief The Get_Generalized_Action function gives the [N x 1] array that represents the contribution of an Action to the EOM. Generally this components are known as generalized forces.
                %> 
                %> @param Name Name of the Action
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Actions(:,1));
                Action_Cell = obj.System_Actions([find(find_match,1)],:);
                Rigid_Body_Name = Action_Cell{2};
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Rigid_Body_Object = Rigid_Body_Cell{4};
                %Get Generalized Coordintes of System
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                Vector_Action_in_G = obj.Get_Action_in_G(Name);
                Virtual_Velocities = sym([]);
                Generalized_Action = sym([]);
                for i = 1:length(q_cell)
                    Virtual_Velocity_i = Rigid_Body_Object.Get_Info("Virtual_Velocities",'Generalized_Coordinate',q_cell{i});
                    Virtual_Velocities = [Virtual_Velocities, Virtual_Velocity_i];
                    Generalized_Action = [Generalized_Action; Vector_Action_in_G.'*Virtual_Velocity_i];
                end
                out = Generalized_Action;
            end
            %> @brief The Get_System_Generalized_Actions function gives the [N x 1] array that represents the contribution of all Actions of the System to the EOM. Generally this components are known as generalized forces.
            %> 
            %> @param obj System
            function out = Get_System_Generalized_Actions(obj)
                Actions = obj.System_Actions(:,1);
                n = length(Actions);
                Generalized_Actions_Matrix = sym([]);
                for i = 2:n
                    Generalized_Actions_Matrix = [Generalized_Actions_Matrix, obj.Get_Generalized_Action(Actions{i})];
                end
                sz = size(Generalized_Actions_Matrix);
                Generalized_Actions = sym([]);
                for i = 1:sz(1)
                    Generalized_Actions = [Generalized_Actions; sum(Generalized_Actions_Matrix(i,:))];
                end
                out = Generalized_Actions;
            end
        end
    %% GENERALIZED COORDINATES
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            System_Canonical_Generalized_Coordinate      
        end
        % PROTECTED
        properties(Access = protected)

        end
        % PUBLIC
        properties(Access = public)
            System_Generalized_Coordinates
        end
      % METHODS
        methods(Access = private)
        %> @brief The System_Create_Generalized_Coordinates_Table function creates a table of all of the generalized coordinates in the system.
        %>
        %> The table has 2 columns: Generalized_Coordinate_Name(string variable) and Generalized_Coordinate_Symbol (symbolic class variable). Each time the user introduces a new generalized coordinate in the system the table is updated and a new generalized coordinate is added.
            function table_out = System_Create_Generalized_Coordinates_Table(obj)
                sz = [0 2];
                varTypes = {'string','sym'};
                varNames = {'Generalized_Coordinate_Name','Generalized_Coordinate_Symbol'};
                table_out = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames);
                table_out = [obj.System_Generalized_Coordinates; {"Canonical",sym(nan)}];
            end
        end
        % PROTECTED
        methods(Access = protected)

        end
        % PUBLIC
        methods(Access = public)
            % ======================================================================
            %> @brief Create_New_Generalized_Coordinate is a function that includes in the system one generalized coordinate.
            %>
            %> @param obj: System object in which to include the new generalized coordinate.
            %> @param Name: Generalized coordinate name (string).
            %> @param Symbol: Generalized coordinate symbol (symbolic variable)
            % ======================================================================
            function Create_New_Generalized_Coordinate(obj, Name, Symbol)
                %CELLFIND Find string matches in a cell array. Used to
                %avoid having Cordinate_Systems with the same name.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Generalized_Coordinates(:,1));
                if any(find_match==1)
                    prompt = "Do you want to overwrite the current "+Name+" Generalized Coordinate? Y/N: ";
                    choice = string(input(prompt,"s"));
                    switch(choice)
                        case "Y"
                            obj.System_Generalized_Coordinates = [obj.System_Generalized_Coordinates; {Name, Symbol}];
                        case "N"
                    end
                else
                    obj.System_Generalized_Coordinates = [obj.System_Generalized_Coordinates; {Name, Symbol}];
                end
            end
            
            function out = Get_Generalized_Coordinates_Info(obj, Info, varargin)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                default_Generalized_Coordinate = "All";
                p = inputParser;
                addRequired(p,'info',@isstring);
                addParameter(p,'generalized_coordinate',default_Generalized_Coordinate,@isstring);
                
                parse(p,Info,varargin{:});
                
                switch(p.Results.info)
                    case "System_Generalized_Coordinates"
                        switch(p.Results.generalized_coordinate)
                            case "All"
                                out = obj.System_Generalized_Coordinates;
                            otherwise
                                row = cellfun(cellfind(p.Results.generalized_coordinate),obj.System_Generalized_Coordinates(:,1));
                                    if(dot(row,row)==0)
                                        error("The Coordinate System "+p.Results.generalized_coordinate +" is not in the System "+obj.System_Name+".")
                                    end
                                out = obj.System_Generalized_Coordinates(row,:);
                        end
                end
            end
        end

    %%  Input
    % PROPERTIES
        % PRIVATE
        properties(Access = private)
            System_Canonical_Input
        end
        % PROTECTED
        properties(Access = protected)

        end
        % PUBLIC
        properties(Access = public)
            System_Inputs
        end
      % METHODS
      methods(Access = private)
        %> @brief This function creates a null Input \f$ \left[ 0,0,0,0,0,0\right] named canonical.
        %>
        %> This Canonical Input is needed for the first time creation of the Input table.
          function Input_out = System_Create_Canonical_Input(obj)
            Input_out = Dynamic_Library.Classes.Action("Canonical", ...
                                                         'Point',obj.System_Canonical_Point, ...
                                                         'Base',obj.System_Canonical_Base, ...
                                                         'Vector',sym([0;0;0;0;0;0]));
          end
            %> @brief The System_Create_Input_Table function creates a table of all of the inputs in the system.
            %>
            %> The table has 4 columns: Input_Name(string variable), Rigid_Body (string variable), Input (Action class variable) and External_Variables(symbolic class variable). Each time the user introduces a new input in the system the table is updated and a new input is added.
            function table_out = System_Create_Input_Table(obj)
                    sz = [0 4];
                    varTypes = {'string','string','Dynamic_Library.Classes.Action','sym'};
                    varNames = {'Input_Name','Rigid_Body','Input','External_Variables'};
                    table_out = table('Size',sz,'VariableTypes',varTypes,'VariableNames',varNames);
                    table_out = [obj.System_Inputs; {obj.System_Canonical_Input.Get_Info("Name"), "Canonical", obj.System_Canonical_Input,sym(nan)}];
            end
        end
        % PROTECTED
        methods(Access = protected)

        end
        % PUBLIC
        methods(Access = public)
            % ======================================================================
            %> @brief Create_New_Input is a function that introduces a new input object in the System.
            %>
            %> Inputs in the System are Action objects. As the library is aimed to design control studies, the System Inputs and Actions have each own table. This helps with the creation of function handles to give an easy way of introducing inputs in simulations.
            %>
            %> @param obj: System in which to introduce a new input
            %> @param Name: Name of the new input
            %> @param Point: Point of application of the input. The point is given by its String Name and has to be introduced previously in the System.
            %> @param Base: Base in which the input components are given. The Base is given by its String Name and has to be introduced previously in the System.
            %> @param Rigid_Body: Rigid Body to which the input is applied. The Rigid_Body is given by its String Name and has to be introduced previously in the System.
            %> @param Vector: Components of the input. The input components can me symbolic or double and they are given by 3 torque components and 3 force components. Eg: [Mx;My;Mz;Fx;Fy;Fz].
            %> @param External_Variables: Variables that define the inputs. If an input is a constant force of 1000 N the vector components are [0;0;0;1000;0;0] and there is no new variables added to the system. If there is a need of study a input that varies with time [0;0;0;F(t);0;0] the external variable should be added as a symbolic variable F(t). Eg: str2sym("F(t)"). 
            % ======================================================================
            function Create_New_Input(obj, Name, varargin)
                %CELLFIND Find string matches in a cell array. Used to
                %avoid having Cordinate_Systems with the same name.
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Inputs(:,1));

                default_Point = "Canonical";
                default_Base = "Canonical";
                default_Rigid_Body = "Canonical";
                default_Vector = sym([0;0;0;0;0;0]);
                
                issym = @(x) isequal(class(x),"sym");
                issym_6_1 = @(x) isequal(class(x),"sym") && isequal(size(x),[6 1]);
                issym_or_base = @(x) issym(x) || isequal(class(x),"Dynamic_Library.Classes.Base");

                p = inputParser;
                addRequired(p,'name',@isstring);
                addParameter(p,'Point',default_Point,@isstring);
                addParameter(p,'Base',default_Base,@isstring);
                addParameter(p,'Rigid_Body',default_Rigid_Body,@isstring);
                addParameter(p,'Vector',default_Vector,issym_6_1);
                addParameter(p,'External_Variables',default_Vector,issym);
                parse(p,Name,varargin{:});

                find_match_point = cellfun(cellfind(p.Results.Point),obj.System_Points(:,1));
                find_match_base = cellfun(cellfind(p.Results.Base),obj.System_Bases(:,1));
                if any(find_match==1)
                    prompt = "Do you want to overwrite the current "+Name+" Input? Y/N: ";
                    choice = string(input(prompt,"s"));
                    switch(choice)
                        case "Y"
                            Point_Object = obj.System_Points([find(find_match_point,1)],2);
                            Base_Object = obj.System_Bases([find(find_match_base,1)],2);
                            Input = Dynamic_Library.Classes.Action(p.Results.name, ...
                                                                            'Point',Point_Object{:},...
                                                                            'Base',Base_Object{:},...
                                                                            'Vector',p.Results.Vector);
                            obj.System_Inputs = [obj.System_Inputs; {Name, p.Results.Rigid_Body, Input,p.Results.External_Variables}];
                        case "N"
                    end
                else
                    Point_Object = obj.System_Points([find(find_match_point,1)],2);
                    Base_Object = obj.System_Bases([find(find_match_base,1)],2);
                    Input = Dynamic_Library.Classes.Action(p.Results.name, ...
                                                                    'Point',Point_Object{:},...
                                                                    'Base',Base_Object{:},...
                                                                    'Vector',p.Results.Vector);
                    obj.System_Inputs = [obj.System_Inputs; {Name, p.Results.Rigid_Body, Input,p.Results.External_Variables}];
                end
            end
            function out = Get_Input_Info(obj, Info, varargin)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                default_Input = "All";
                p = inputParser;
                addRequired(p,'info',@isstring);
                addParameter(p,'Input',default_Input,@isstring);
                
                parse(p,Info,varargin{:});
                
                switch(p.Results.info)
                    case "System_Inputs"
                        switch(p.Results.Input)
                            case "All"
                                out = obj.System_Inputs;
                            otherwise
                                row = cellfun(cellfind(p.Results.Input),obj.System_Inputs(:,1));
                                    if(dot(row,row)==0)
                                        error("The Input "+p.Results.Input +" is not in the System "+obj.System_Name+".")
                                    end
                                out = obj.System_Inputs(row,:);
                        end              
                end
            end
            function out = Get_Input_in_G(obj,Name)
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Inputs(:,1));
                Input_Cell = obj.System_Inputs([find(find_match,1)],:);
                Rigid_Body_Name = Input_Cell{2};
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Input = Input_Cell{3};
                G_Name = Rigid_Body_Cell{2};
                find_match_point = cellfun(cellfind(G_Name),obj.System_Points(:,1));
                G_Point_Object = obj.System_Points([find(find_match_point,1)],2);
                G_Point_Coordinates = G_Point_Object{:}.Get_Info("Point_Coordinates_From_Canonical");
                Input_Point_Coordinates = Input.Action_Point.Get_Info("Point_Coordinates_From_Canonical");
                GP = Input_Point_Coordinates-G_Point_Coordinates;
                Input_Vector_in_G = [Input.Action_Vector(1:3)+cross(GP,Input.Action_Vector(4:end)); Input.Action_Vector(4:end)];
                out = Input_Vector_in_G;
            end
            function out = Get_Generalized_Input(obj,Name)
                %> @brief The Get_Generalized_Input function gives the [N x 1] array that represents the contribution of an Input to the EOM. Generally this components are known as generalized forces.
                %> 
                %> @param Name Name of the Input
                cellfind = @(string)(@(cell_contents)(strcmp(string,cell_contents)));
                find_match = cellfun(cellfind(Name),obj.System_Inputs(:,1));
                Input_Cell = obj.System_Inputs([find(find_match,1)],:);
                Rigid_Body_Name = Input_Cell{2};
                find_match_Rigid_Body = cellfun(cellfind(Rigid_Body_Name),obj.System_Rigid_Bodies(:,1));
                Rigid_Body_Cell = obj.System_Rigid_Bodies([find(find_match_Rigid_Body,1)],:);
                Rigid_Body_Object = Rigid_Body_Cell{4};
                %Get Generalized Coordintes of System
                q_cell = obj.System_Generalized_Coordinates(2:end,2);
                Vector_Input_in_G = obj.Get_Input_in_G(Name);
                Virtual_Velocities = sym([]);
                Generalized_Input = sym([]);
                for i = 1:length(q_cell)
                    Virtual_Velocity_i = Rigid_Body_Object.Get_Info("Virtual_Velocities",'Generalized_Coordinate',q_cell{i});
                    Virtual_Velocities = [Virtual_Velocities, Virtual_Velocity_i];;
                    Generalized_Input = [Generalized_Input; Vector_Input_in_G.'*Virtual_Velocity_i];
                end
                out = Generalized_Input;
            end
            %> @brief The Get_System_Generalized_Inputs function gives the [N x 1] array that represents the contribution of all Inputs of the system to the EOM. Generally this components are known as generalized forces.
            %> 
            %> @param obj System
            function out = Get_System_Generalized_Inputs(obj)
                Inputs = obj.System_Inputs(:,1);
                n = length(Inputs);
                Generalized_Inputs_Matrix = sym([]);
                for i = 2:n
                    Generalized_Inputs_Matrix = [Generalized_Inputs_Matrix, obj.Get_Generalized_Input(Inputs{i})];
                end
                sz = size(Generalized_Inputs_Matrix);
                Generalized_Inputs = sym([]);
                for i = 1:sz(1)
                    Generalized_Inputs = [Generalized_Inputs; sum(Generalized_Inputs_Matrix(i,:))];
                end
                out = Generalized_Inputs;
            end
        end
end