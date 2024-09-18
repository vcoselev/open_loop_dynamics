classdef Action < handle
    %% PROPERTIES
        % PRIVATE
        properties(Access = public)
            Action_Name
            Action_Point
            Action_Base
            Action_Vector
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
            % ======================================================================
            %> @brief Action Class constructor
            %>
            %> @param obj: System in which to introduce a new action
            %> @param Name: Name of the new action
            %> @param Point: Point (object) of application of the action.
            %> @param Base: Base (object) in which the action components are given.
            %> @param Vector: Components of the Action. The action components can me symbolic or double and they are given by 3 torque components and 3 force components. Eg: [Mx;My;Mz;Fx;Fy;Fz].
            % ======================================================================
            function obj = Action(Name, varargin)
                default_Action_Point = Dynamic_Library.Classes.Point("Canonical");
                default_Action_Base = Dynamic_Library.Classes.Base( ...
                    "Canonical", ...
                    "axis_labels",[str2sym("x");str2sym("y");str2sym("z")]);
                default_Action_Vector = sym([0;0;0;0;0;0]);
                
                %Function to test if variable is sym class.
                issym = @(x) isequal(class(x),"sym");
                issym_6_1 = @(x) isequal(class(x),"sym") && isequal(size(x),[6 1]);
                issym_or_base = @(x) issym(x) || isequal(class(x),"Dynamic_Library.Classes.Base");

                %Function to test if a variable is either a sym or a
                %numeric type.
                issym_or_numeric = @(x) issym(x) || isnumeric(x);

                ispoint = @(x) isequal(class(x),"Dynamic_Library.Classes.Point");
                p = inputParser;
                addRequired(p,'name',@isstring);
                addParameter(p,'Point',default_Action_Point,ispoint);
                addParameter(p,'Base',default_Action_Base,issym_or_base);
                addParameter(p,'Vector',default_Action_Vector,issym_6_1);
                
                parse(p,Name,varargin{:});

                obj.Action_Name = p.Results.name;
                obj.Action_Point = p.Results.Point;
                obj.Action_Base = p.Results.Base;
                obj.Action_Vector = p.Results.Vector;
            end

            function out = Get_Info(obj, Info)
            %GET_INFO Get properties of an "Action" object.
                switch(Info)
                    case "Name"
                        out = obj.Action_Name;
                    case "Action_Point"
                        out = obj.Action_Point;
                    case "Action_Base"
                        out = obj.Action_Base;
                    case "Action_Vector"
                        out = obj.Action_Vector;
                end
            end
        end

        % PUBLIC STATIC
        methods(Static,Access = public)

        end
end