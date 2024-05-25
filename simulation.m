classdef simulation < uint32
    methods
        function o = getFolderName(obj)
            o = string(obj);
        end
    end
    enumeration
        EFF (0)
        COP (1)
        BOS (2)
        ACC (3)
        ZMP (4)
    end
end

