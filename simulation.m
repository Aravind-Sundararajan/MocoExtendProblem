classdef simulation < uint32
    methods
        function o = getFolderName(obj)
            o = string(obj);
        end
    end
    enumeration
        EFF (0)
        ACC (1)
        BOS (2)
        ZMP (3)
    end
end

