%> @file Base.m
%> @brief Base Class description
% ======================================================================
%> @brief Base Class for the creation of orthonormal vectorial bases.
% ======================================================================
    classdef Base < handle
    %% PROPERTIES
        % PRIVATE
        properties(Access = public)
            %> String that indicates the name of the base object.
            Base_Name
            %> [3x1] symbolic array that represents the labels for the vectors of the base.
            Axis_Labels
            %> [3x3] symbolic or double array that represents the matrix to change the coordinates of a vector in the canonical base (E) to the actual base (B).
            M_B_E
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
        % ======================================================================
            %> @brief Base Class constructor
            %>
            %> 
            %>
            %> The Base constructor can initialize the object in one of two ways: either as the canonical basis \f$[1,0,0]\f$, \f$[0,1,0]\f$, and \f$[0,0,1]\f$, or through rotation with respect 
            %> to another base. This means that the base can be directly set to the standard orthonormal basis vectors, which are the default coordinate axes, or it can be 
            %> created by rotating an already existing base.
            %>
            %> @param Name
            %> @param axis_labels(optional) [3x1] symbolic array that contains the labels of the 3 vectors that represents the base. If this parameter is not specified the default axis_labels will be \f$\left[x,y,z\right]^T\f$.
            %> @param father_base(optional) The parent Base object defines the base from which the constructor will create a new Base object. From this parent base, a new base is generated through a simple rotation. The simple rotation can only be performed in the direction indicated by the right-hand rule for each of the vectors of the parent base. If father_base parameter is not defined, the default parent base is \f$[1,0,0]\f$, \f$[0,1,0]\f$, and \f$[0,0,1]\f$.
            %> @param axis_rotation(optional) The simple rotation can only be performed in the direction indicated by the right-hand rule for each of the vectors of the parent base. The axis_rotation parameter defines the rotation axis with a string "1", "2" or "3" defining the parent basis vector to use. If axis_rotation parameter is not defined, the default axis of rotation "1".
            %> @param angle_rotation(optional) Symbolic or double parameter that defines the angle of rotation (in radians) of the new base. If angle_rotation parameter is not defined the default angle of rotation is 0.
            %>
            %>
            %>@return Instance of the Rigid_Body class.
            % ======================================================================
        methods(Access = public)
            function obj = Base(Name, varargin)
                default_axis_labels = [str2sym("x");str2sym("y");str2sym("z")];
                default_base = sym([1 0 0; 0 1 0; 0 0 1]);
                default_axis = "1";
                default_angle = 0;
                %Function to test if variable is sym class.
                issym = @(x) isequal(class(x),"sym");
                issym_3_1 = @(x) isequal(class(x),"sym") && isequal(size(x),[3 1]);
                issym_or_base = @(x) issym(x) || isequal(class(x),"Dynamic_Library.Classes.Base");
                %Function to test if variable is string "1", "2" or "3".
                %Defined for axis.
                isaxis = @(x) isequal(x,"1") || isequal(x,"2") || isequal(x,"3");

                %Function to test if a variable is either a sym or a
                %numeric type.
                issym_or_numeric = @(x) issym(x) || isnumeric(x);
                
                p = inputParser;
                addRequired(p,'name',@isstring);
                addParameter(p,'axis_labels',default_axis_labels,issym_3_1);
                addParameter(p,'father_base',default_base,issym_or_base);
                addParameter(p,'axis_rotation',default_axis,isaxis);
                addParameter(p,'angle_rotation',default_angle,issym_or_numeric);
                
                parse(p,Name,varargin{:});

                obj.Base_Name = p.Results.name;
                obj.Axis_Labels = p.Results.axis_labels;
                
                switch(class(p.Results.father_base))
                    case "sym"
                        father_M_B_E = p.Results.father_base;
                        M_BF_BM = Dynamic_Library.Classes.Base.Rotation_Change_Base_Matrix(p.Results.axis_rotation, p.Results.angle_rotation);
                        obj.M_B_E = (father_M_B_E.'*M_BF_BM).';
                    case "Dynamic_Library.Classes.Base"
                        father_M_B_E = p.Results.father_base.Get_Info("Base_Matrix_From_Canonical");
                        M_BF_BM = Dynamic_Library.Classes.Base.Rotation_Change_Base_Matrix(p.Results.axis_rotation, p.Results.angle_rotation);
                        obj.M_B_E = (father_M_B_E.'*M_BF_BM).';
                    otherwise
                        error("Father base needs to be specified as a 3x3 'sym' matrix or an object of the 'Dynamic_Library.Classes.Base' class");
                end
            end
            % ======================================================================
            %> @brief Get_Info is a public function to get the properties of the Base objects.
            % ======================================================================
            function out = Get_Info(obj, Info)
            %GET_INFO Get properties of an "Base" object.
                switch(Info)
                    case "Name"
                        out = obj.Base_Name;
                    case "Axis_Labels"
                        out = obj.Axis_Labels;
                    case "Base_Matrix_From_Canonical"
                        out = obj.M_B_E;
                end
            end
        end

        % PUBLIC STATIC
        methods(Static,Access = public)
        % ======================================================================
        %> @brief Rotation_Change_Base_Matrix is a Static function that calculates a matrix to change the components of a vector in a moving base (BM) to a fixed base (BF).
        %>
        %> @param axis: Father base axis of rotation. String that indicates which father base vector use to calculate the rotation.
        %> @param angle: Symbolic or double parameter which indicates the angle of rotation in radians.
        % ======================================================================
            function M_BF_BM = Rotation_Change_Base_Matrix(axis, angle)
                % FUNCTION TO GET THE ROTATION MATRIX OF CERTAIN ANGLE. 
                % ANGLE IN RADIANS
                switch(axis)
                    case "1"
                        M_BF_BM = sym([1,          0,              0;
                                                                0,          cos(angle),    -sin(angle);
                                                                 0,          sin(angle),  cos(angle)]);
                    case "2"
                        M_BF_BM = sym([cos(angle),     0  , sin(angle);
                                                                0,               1,          0;
                                                                -sin(angle),     0,    cos(angle)]);
                    case "3"
                        M_BF_BM = sym([cos(angle), -sin(angle), 0; 
                                                                sin(angle), cos(angle), 0;
                                                                    0,             0,       1]);
                    otherwise
                        error("Axis of rotation has not been specified correctly. Please use string notation for the axis of rotation.");
                end
            end
        end
end