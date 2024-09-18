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
        %> @brief Rigid_Body Class Constructor
        %>
        %>
        %> The Rigid_Body Constructor can initialize the object by defining the parameters of a rigid body.
        %> 
        %> @param name String variable that defines the name of the rigid body.
        %> @param G_Point [3x1] symbolic or double array that contains the position of the center of mass of the rigid body.
        %> @param Base Base object that represents the base that is connected with the rigid body.
        %> @param Mass Symbolic or double scalar that represents the total mass of the rigid body.
        %> @param Inertial_Tensor [3x3] symbolic or double array that represents the inertial tensor of the rigid body. If this array is a zero [3 x 3] array the object is a particle.
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
                addParameter(p,'Inertial_Tensor',default_Inertial_Tensor,issym_or_numeric);
                
                parse(p,Name,varargin{:});

                obj.Rigid_Body_Name = p.Results.name;
                obj.G_Point_From_Canonical = p.Results.G_Point;
                obj.Rigid_Body_Base = p.Results.Base;
                obj.Mass = p.Results.Mass;
                obj.Inertial_Tensor = p.Results.Inertial_Tensor;
            end

            %> @brief The Get_Info method in the Rigid_Body class gets information of the rigid body object depending on the input parameter.
            %>
            %>
            %> @param Name Every Rigid_Body object has an asociated name with it. The Name option gives the name of the current rigid body object.
            %> @param G_Point_From_Canonical The movement of a rigid body is given by its center of mass. This option gives the [3 x 1] symbolic or double array of the center of mass of the rigid body.
            %> @param Rigid_Body_Base This option gives the base object that it is associated to the rigid body.
            %> @param Inertial_Tensor This option gives the [3 x 3] array that represents inertial tensor \f$ \mathbf{I}\f$ of the rigid body.
            %> @param Absolute_Velocity As the position \f$ \mathbf{r} \f$ of the rigid body center of mass is given in the canonical reference frame (and the canonical point) the velocity can be calculated as the first derivative respect to time of the position vector \f$ \mathbf{v} = \frac{d \mathbf{r}}{dt}\f$. This option makes the differentiation of the position vector and returns the absolute velocity.
            %> @param Absolute_Acceleration As the position \f$ \mathbf{r} \f$ of the rigid body center of mass is given in the canonical reference frame (and the canonical point) the acceleration can be calculated as the second derivative respect to time of the position vector \f$ \mathbf{a} = \frac{d^2 \mathbf{r}}{dt^2}\f$. This option makes the second derivative of the position vector and returns the absolute acceleration.
            %> @param Absolute_Angular_Velocity The angular velocity of the rigid body is given by the components of the skew-symmetric matrix \f$ \left[\mathbf{S}\right]^T \dot{\left[\mathbf{S}\right]} \f$, where \f$ \mathbf{S} \f$ is the  change of base matrix from the rigid body base to the canonical base. The components of the skew-symmetric matrix give the components of the angular velocity vector in the Rigid Body reference frame as \f$ \left\{\mathbf{\Omega_{R}(BM)}\right\}_{BM} \f$. This option calculates the vector components of the angular velocity in the Rigid Body reference frame and then returns the vector components in the canonical reference frame \f$ \left\{\mathbf{\Omega_{R}(BM)}\right\}_{R} \f$.
            %> @param Virtual_Velocities In a holonomic system the real and virtual velocities can be expressed in terms of the generalized coordinates of the system. This equations for the lineal and angular velocity are given by  \f$ \mathbf{v_i} = \sum_{k}^{} \frac{\partial \mathbf{v_i}}{\partial \dot{q_k}} \dot{q_k} \f$ and \f$ \mathbf{\Omega_i} = \sum_{k}^{} \frac{\partial \mathbf{\Omega_i}}{\partial \dot{q_k}} \dot{q_k} \f$, where \f$ i\f$ represents the rigid body and \f$ k\f$ the generalized coordinates. This equations  can be rewritten as \f$ \mathbf{v_i} = \sum_{k}^{} \mathbf{b^i_k} \dot{q_k} \f$ and \f$ \mathbf{\Omega_i} = \sum_{k}^{} \mathbf{c^i_k} \dot{q_k} \f$. The option Virtual_Velocities in the Get_Info function gives the symbolic array \f$ \left[ \mathbf{c_k^i, b_k^i}\right]^T \f$ for the specific rigid body object \f$ i\f$ and a generalized coordinate \f$ q_k\f$.
            %> @param Kinetic_Energy This option returns the kinetic energy of the rigid body.
            %> @param A_Matrix The first component (1,1) of the \f$ \left[ A \right]\f$ matrix of a rigid body is gonna be \f$ -m \mathbf{b_1}^T \mathbf{b_1} - \left[ \mathbf{I} \mathbf{c_1} \right]^T \mathbf{c_1} \f$. In this expression \f$ m\f$ is the mass of the rigid body, \f$ \mathbf{I}\f$ is the inertial tensor of the rigid body and \f$ \mathbf{b_1}\f$ and \f$  \mathbf{c_1}\f$ are the virtual displacements of the rigid body with respect to the first generalized coordinate. This option returns the symbolic or double \f$ \left[ n \times n\right]\f$ matrix \f$ \left[ A \right]\f$.
            %> @param d_A_Matrix This option gives the derivative matrix of matrix \f$ \left[ \mathbf{A} \right]\f$.
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