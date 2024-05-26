import org.opensim.modeling.*;

output_dirs = ["./output/effpred/",...
    "./output/meppredmarkerAccel/",...
    "./output/meppredBOS/",...
    "./output/meppredZMP/",...    
    ];%

for s = 1:4
    output_dir = output_dirs(s);
    ref = MocoTrajectory(output_dir + '/outputReference/states_half.sto');
    gaitPredictiveSolution = MocoTrajectory(output_dir + 'states_half.sto');
    if gaitPredictiveSolution.isNumericallyEqual(ref);
        warning("output matches output reference for " + output_dir);
    else
        error("Sim failed to match reference output for goal " + output_dir);
    end

end