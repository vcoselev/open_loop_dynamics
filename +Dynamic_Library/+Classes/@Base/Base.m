    classdef Base < handle
    %% PROPERTIES
        % PRIVATE
        properties(Access = public)
            Base_Name
            Axis_Labels
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