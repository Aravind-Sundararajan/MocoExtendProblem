classdef extend_problem < handle
    
    properties (Access = private)
        id_ % ID of the session
    end
    
    methods
        function this = extend_problem(Cptr)
            %extendProblem Create a new
            assert(isinteger(Cptr));
            this.id_ = extendProblem('new',Cptr);
        end
        
        function delete(this)
            %delete Destructor
            extendProblem('delete', this.id_);
        end
        
        function addCustomGoal(this,weight)
            assert(isscalar(weight));
            extendProblem('addCustomGoal', this.id_,weight)
        end
        
    end
end