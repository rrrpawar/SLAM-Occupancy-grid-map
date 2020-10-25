function [pose, negScore, hessian, exitFlag, iters ] = matchScans(parsedInputs)
%This function is for internal use only. It may be removed in the future.

%matchScans Match two scans using in-house NLP solver

%   Copyright 2016-2020 The MathWorks, Inc.

%#codegen

    referenceScan = parsedInputs.ReferenceScan;
    currentScan = parsedInputs.CurrentScan;
    cellSize = parsedInputs.CellSize;
    maxIterations = parsedInputs.MaxIterations;
    scoreTolerance = parsedInputs.ScoreTolerance;
    initialPose = parsedInputs.InitialPose';

    if coder.target('MATLAB')
        % When running in MATLAB, use MEX file for improved performance

        if isa(referenceScan.Ranges, 'single') || isa(currentScan.Ranges, 'single')
            % The MEX file is compiled to expect double ranges and angles
            [pose, negScore, hessian, exitFlag, iters ] = nav.algs.internal.mex.matchScans(...
                lidarScan(double(referenceScan.Ranges), double(referenceScan.Angles)), ...
                lidarScan(double(currentScan.Ranges), double(currentScan.Angles)), ...
                initialPose, cellSize, maxIterations, scoreTolerance);
        else
            [pose, negScore, hessian, exitFlag, iters ] = nav.algs.internal.mex.matchScans(...
                referenceScan, currentScan, initialPose, cellSize, maxIterations, scoreTolerance);
        end

    else
        % When generating code, use MATLAB implementation
        [pose, negScore, hessian, exitFlag, iters ] = nav.algs.internal.impl.matchScans(referenceScan, currentScan, initialPose, cellSize, maxIterations, scoreTolerance);
    end

end
