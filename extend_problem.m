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
        
        function addActivationGoal(this,weight)
            assert(isscalar(weight));
            extendProblem('addActivationGoal', this.id_,weight)
        end
        function addAccelerationGoal(this,weight)
            assert(isscalar(weight));
            extendProblem('addAccelerationGoal', this.id_,weight)
        end
        function addZMPGoal(this,weight)
            assert(isscalar(weight));
            extendProblem('addZMPGoal', this.id_,weight)
        end
        function addMaxCoordinateGoal(this,weight)
            assert(isscalar(weight));
            extendProblem('addMaxCoordinateGoal', this.id_,weight)
        end
    end
end