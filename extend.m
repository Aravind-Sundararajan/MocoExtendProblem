classdef extend_problem < handle
    
    properties (Access = private)
        id_ % ID of the session
    end
    
    methods
        function this = extend_problem(Cptr)
            %extendProblem Create a new
            disp(Cptr)
            assert(isinteger(Cptr));
            this.id_ = extendProblem('new',Cptr);
        end
        
        function delete(this)
            %delete Destructor
            extendProblem('delete', this.id_);
        end
        function addMocoActivationSquaredGoal(this,goalname,weight, tf, end_point_goal)
            extendProblem('addMocoActivationSquaredGoal', this.id_,goalname,weight, tf, end_point_goal);
        end
        function addMocoCoordinateAccelerationGoal(this,goalname,weight, tf, refCoordNames)
            extendProblem('addMocoCoordinateAccelerationGoal', this.id_,goalname,weight, tf, refCoordNames);
        end
        function addMocoMarkerAccelerationGoal(this,goalname,weight, name, tf)
            extendProblem('addMocoMarkerAccelerationGoal', this.id_,goalname,weight, name, tf);
        end
        function addMocoMaxCoordinateGoal(this,goalname,weight, tf)
            extendProblem('addMocoMaxCoordinateGoal', this.id_,goalname,weight, tf);
        end
        function addMocoOutputTrackingGoal(this,goalname,weight, tf, end_point_goal)
            extendProblem('addMocoOutputTrackingGoal', this.id_,goalname,weight, tf, end_point_goal);
        end
        function addMocoZMPGoal(this,goalname,weight, tf)
            extendProblem('addMocoZMPGoal', this.id_,goalname,weight, tf);
        end
    end
end
