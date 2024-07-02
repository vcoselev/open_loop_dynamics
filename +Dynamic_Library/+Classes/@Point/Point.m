classdef Point
    %% PROPERTIES
        % PRIVATE
        properties(Access = public)
            Point_Name
            Point_Coordinates_From_Canonical
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
            %> @brief Point Class Constructor
            %> 
            %>
            %> Point objects can be created by two different ways. A point is created in an absolute way if the point coordinates are given with respect the canonical point.
            %>
            %> @param Name: String that defines the name of the object
            %> @param point_coordinates: [3 x 1] dobule or symbolic array that defines the position of the point with respect the absolute point (canonical point [0,0,0]).
            %>
            %> The second way of creating a point is with respect a coordinate system. A coordinate system is defined by a base and a point, and the new point is given by its position with respect to the father point with the father_base components. 
            %> @param Name: String that defines the name of the object.
            %> @param father_point: Father Point object.
            %> @param father_base: Father Base object.
            %> @param point_coordinates: [3 x 1] dobule or symbolic array that defines the position of the point with respect the relative coordinate system (Father_Point and Father_Base).
            function obj = Point(Name, varargin)

                default_father_Point = sym([0; 0; 0]);
                default_father_Base = sym([1 0 0; 0 1 0; 0 0 1]);
                default_Point_Coordinates = sym([0; 0; 0]);
                %Function to test if variable is sym class.
                issym_3_1 = @(x) or(isequal(class(x),"sym"),isequal(class(x),"double")) && isequal(size(x),[3 1]);
                issym_3_1_or_point_class = @(x) issym_3_1(x) || isequal(class(x),"Dynamic_Library.Classes.Point");
                issym = @(x) isequal(class(x),"sym");
                issym_or_base = @(x) issym(x) || isequal(class(x),"Dynamic_Library.Classes.Base");
                
                p = inputParser;
                addRequired(p,'name',@isstring);
                addParameter(p,'father_point',default_father_Point,issym_3_1_or_point_class);
                addParameter(p,'father_base', default_father_Base,issym_or_base);
                addParameter(p,'point_coordinates',default_Point_Coordinates,issym_3_1);
                
                parse(p,Name,varargin{:});

                obj.Point_Name = p.Results.name;
                switch(class(p.Results.father_base))
                    case "sym"
                        father_M_B_E = p.Results.father_base;
                        r_B = p.Results.point_coordinates;
                        r_E = father_M_B_E.'*r_B;
                    case "Dynamic_Library.Classes.Base"
                        father_M_B_E = p.Results.father_base.Get_Info("Base_Matrix_From_Canonical");
                        r_B = p.Results.point_coordinates;
                        r_E = father_M_B_E.'*r_B;
                    otherwise
                        error("Father base needs to be specified as a 3x3 'sym' matrix or an object of the 'Dynamic_Library.Classes.Base' class");
                end

                switch(class(p.Results.father_point))
                    case "sym"
                        obj.Point_Coordinates_From_Canonical = p.Results.father_point + r_E;
                    case "Dynamic_Library.Classes.Point"
                        obj.Point_Coordinates_From_Canonical = p.Results.father_point.Get_Info("Point_Coordinates_From_Canonical") + r_E;
                    otherwise
                        error("Father point needs to be specified as a 3x1 'sym' vector or an object of the 'Dynamic_Library.Classes.Point' class");
                end
            end
            

            function out = Get_Info(obj, Info)
            %GET_BASE_INFO Get properties of an "Base" object.
                switch(Info)
                    case "Name"
                        out = obj.Point_Name;
                    case "Point_Coordinates_From_Canonical"
                        out = obj.Point_Coordinates_From_Canonical;
                    case "Point_Velocity_Coordinates_From_Canonical"
                        syms t
                        r = obj.Get_Info("Point_Coordinates_From_Canonical");
                        v = simplify(diff(r,t));
                        out = v;
                end
            end
        end

        % PUBLIC STATIC
        methods(Static,Access = public)

        end
end