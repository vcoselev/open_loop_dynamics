%> @file Rigid_Body.m
%> @brief Rigid Body Class description
% ======================================================================
%> @brief Rigid Body Class for the creation of Rigid Body objects.
% ======================================================================
classdef Rigid_Body < handle
    %% PROPERTIES
        % PRIVATE
        %> Public properties for Rigid Body Class
        properties(Access = public)
            %> Name of Rigid Body. This property is a String class variable representing que name of the Rigid Body Object.
            Rigid_Body_Name
            %> [3x1] symbolic or double array that represents the position of the center of mass from the canonical point (in the System Class this point is named "Canonical").
            G_Point_From_Canonical
            %> Rigid Body base represented by the Base Class.
            Rigid_Body_Base
            %> Symbolic or double scalar that represents the mass of the Rigid Body Object.
            Mass
            %> [3x3] symbolic or double array. This array represent the inertial tensor of the Rigid Body Object.
            Inertial_Tensor
        end
        % PROTECTED
        properties(Access = protected)
            
        end

        % PUBLIC
        properties(Access = public)

        end
      %% METHODS

        methods(Access = private)
            
        end

        % PROTECTED
        methods(Access = public)

        end

        % PUBLIC
        methods(Access = public)
            function obj = Rigid_Body(Name, varargin)
                default_G_Point = Dynamic_Library.Classes.Point("Canonical");
                default_Rigid_Body_Base = Dynamic_Library.Classes.Base( ...
                    "Canonical", ...
                    "axis_labels",[str2sym("x");str2sym("y");str2sym("z")]);
                default_Mass = str2sym("m_"+Name);
                default_Inertial_Tensor = sym([]);
                default_Inertial_Tensor = [default_Inertial_Tensor; "I_"+Name+"_"+string(1)+"1", "I_"+Name+"_"+string(1)+"2", "I_"+Name+"_"+string(1)+"3"];
                default_Inertial_Tensor = [default_Inertial_Tensor; "I_"+Name+"_"+string(1)+"2", "I_"+Name+"_"+string(2)+"2", "I_"+Name+"_"+string(2)+"3"];
                default_Inertial_Tensor = [default_Inertial_Tensor; "I_"+Name+"_"+string(1)+"3", "I_"+Name+"_"+string(2)+"3", "I_"+Name+"_"+string(3)+"3"];
                
                %Function to test if variable is sym class.
                issym = @(x) isequal(class(x),"sym");
                issym_3_1 = @(x) isequal(class(x),"sym") && isequal(size(x),[3 1]);
                issym_or_base = @(x) issym(x) || isequal(class(x),"Dynamic_Library.Classes.Base");

                %Function to test if a variable is either a sym or a
                %numeric type.
                issym_or_numeric = @(x) issym(x) || isnumeric(x);

                ispoint = @(x) isequal(class(x),"Dynamic_Library.Classes.Point");
                p = inputParser;
                addRequired(p,'name',@isstring);
                addParameter(p,'G_Point',default_G_Point,ispoint);
                addParameter(p,'Base',default_Rigid_Body_Base,issym_or_base);
                addParameter(p,'Mass',default_Mass,issym_or_numeric);
                addParameter(p,'Intertial_Tensor',default_Inertial_Tensor,issym_or_numeric);
                
                parse(p,Name,varargin{:});

                obj.Rigid_Body_Name = p.Results.name;
                obj.G_Point_From_Canonical = p.Results.G_Point;
                obj.Rigid_Body_Base = p.Results.Base;
                obj.Mass = p.Results.Mass;
                obj.Inertial_Tensor = p.Results.Intertial_Tensor;
            end

            function out = Get_Info(obj, Info, varargin)
            %GET_INFO Get properties of an "Base" object.
                issym = @(x) isequal(class(x),"sym");
                p = inputParser;
                default_generalized_coordinate = sym(nan);
                default_q_vect = sym(nan);
                addParameter(p,'Generalized_Coordinate',default_generalized_coordinate,issym);
                addParameter(p,'q_vect',default_q_vect,issym);
                parse(p,varargin{:});

                switch(Info)
                    case "Name"
                        out = obj.Rigid_Body_Name;
                    case "G_Point_From_Canonical"
                        out = obj.G_Point_From_Canonical;
                    case "Rigid_Body_Base"
                        out = obj.Rigid_Body_Base;
                    case "Mass"
                        out = obj.Mass;
                    case "Inertial_Tensor"
                        out = obj.Inertial_Tensor;
                    case "Absolute_Velocity"
                        syms t
                        out = simplify(diff(obj.G_Point_From_Canonical.Get_Info("Point_Coordinates_From_Canonical"),t),'Steps',10);
                    case "Absolute_Acceleration"
                        syms t
                        out = simplify(diff(obj.G_Point_From_Canonical.Get_Info("Point_Coordinates_From_Canonical"),t,2),'Steps',10);
                    case "Absolute_Angular_Velocity"
                        syms t
                        S = transpose(obj.Rigid_Body_Base.Get_Info("Base_Matrix_From_Canonical"));
                        S_dot = simplify(diff(S,t),'Steps',10);
                        Ang_Vel_Mat = simplify(transpose(S)*S_dot);
                        out = [Ang_Vel_Mat(3,2); Ang_Vel_Mat(1,3); Ang_Vel_Mat(2,1)];
                    case "Virtual_Velocities"
                        G_Point_From_Canonical = obj.Get_Info("G_Point_From_Canonical");
                        S = transpose(obj.Rigid_Body_Base.Get_Info("Base_Matrix_From_Canonical"));
                        b = simplify(diff(G_Point_From_Canonical.Get_Info("Point_Coordinates_From_Canonical"),p.Results.Generalized_Coordinate,1),'Steps',10);
                        c_moving_frame =  simplify(transpose(S)*diff(S,p.Results.Generalized_Coordinate),'Steps',100);
                        c = simplify(S*[c_moving_frame(3,2); c_moving_frame(1,3); c_moving_frame(2,1)],'Steps',100);                        
                        out = [c;b];
                    case "Kinetic_Energy"
                        v = obj.Get_Info("Absolute_Velocity");
                        omega = obj.Get_Info("Absolute_Angular_Velocity");
                        m = obj.Get_Info("Mass");
                        I = obj.Get_Info("Inertial_Tensor");
                        K = 1/2*(m*transpose(v)*v+transpose(omega)*I*omega);
                        K = simplify(K, 'Seconds',10);
                        out = K;
                    case "A_Matrix"
                        m = obj.Get_Info("Mass");
                        I = obj.Get_Info("Inertial_Tensor");
                        q = p.Results.q_vect;
                        n = length(q);
                        B = sym.empty;
                        C = sym.empty;
                        A = sym.empty;
                        d_q = diff(q,str2sym("t"),1);
                        dd_q = diff(q,str2sym("t"),2);
                        for i = 1:n
                            c_i_b_i = obj.Get_Info("Virtual_Velocities",'Generalized_Coordinate',q(i));
                            b_i = c_i_b_i(4:6);
                            c_i = c_i_b_i(1:3);
                            B = [B, b_i];
                            C = [C, c_i];
                        end
                        for j = 1:n
                            for k = 1:n
                                A(j,k) =m*B(:,k).'*B(:,j)+(I*C(:,k)).'*C(:,j);
                                A(j,k) = simplify(A(j,k),'Steps',100);
                            end
                        end
                        out = A;
                    case "d_A_Matrix"
                        m = obj.Get_Info("Mass");
                        I = obj.Get_Info("Inertial_Tensor");
                        q = p.Results.q_vect;
                        n = length(q);
                        d_B = sym.empty;
                        d_C = sym.empty;
                        B = sym.empty;
                        C = sym.empty;
                        d_A = sym.empty;
                        d_q = diff(q,str2sym("t"),1);
                        dd_q = diff(q,str2sym("t"),2);
                        for i = 1:n
                            c_i_b_i = obj.Get_Info("Virtual_Velocities",'Generalized_Coordinate',q(i));
                            b_i = c_i_b_i(4:6);
                            c_i = c_i_b_i(1:3);
                            B = [B, b_i];
                            C = [C, c_i];
                            d_b_i = diff(b_i,str2sym("t"));
                            d_b_i = simplify(d_b_i,'Steps',100);
                            d_c_i = diff(c_i,str2sym("t"));
                            d_c_i = simplify(d_c_i,'Steps',100);
                            d_B = [d_B, d_b_i];
                            d_C = [d_C, d_c_i];
                        end
                        for j = 1:n
                            for k = 1:n
                                d_A(j,k) =m*d_B(:,k).'*B(:,j)+(I*d_C(:,k)).'*C(:,j);
                                d_A(j,k) = simplify(d_A(j,k),'Steps',100);
                            end
                        end
                        out = d_A;
                end
            end
        end

        % PUBLIC STATIC
        methods(Static,Access = public)

        end
end