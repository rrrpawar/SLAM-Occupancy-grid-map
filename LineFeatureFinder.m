classdef LineFeatureFinder < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%LINEFEATUREFINDER This class provides utilities to extract line features
%   from lidarScan object.

%   Copyright 2019-2020 The MathWorks, Inc.

%#codegen

    properties (Constant)
        %MinNumPoints (before line merging)
        MinNumPoints = 3
    end

    properties
        %DoubleDiffThreshold
        DoubleDiffThreshold
        
        %MinCornerProminence
        MinCornerProminence
        
        %MinNumPointsPerSegment (after line merging)
        MinNumPointsPerSegment
        
        %RangeLowerBound
        RangeLowerBound = 0.03
        
        %MergeTolerance Tolerance on line parameters to determine merging
        %   [rhoTol, alphaTol]
        MergeTolerance
    end
    
    methods
        function obj = LineFeatureFinder()
            %LINEFEATUREFINDER Constructor
            params = nav.algs.internal.LineFeatureFinder.getDefaultParams();
            obj.DoubleDiffThreshold = params.DoubleDiffThreshold;
            obj.MinCornerProminence = params.MinCornerProminence;
            obj.MinNumPointsPerSegment = params.MinNumPointsPerSegment;
            obj.MergeTolerance = params.MergeTolerance;
        end
        
        function [lineFeatures, lineSegs, cornerIds] = extractLineFeatures(obj, scan)
            %extractLineFeatures Main method to extract line features from the give scan
            %   Two outputs are returned:
            %   lineFeaturs - N-by-2 matrix with each row representing a
            %                 line parameter ([rho, alpha])
            %   lineSegs    - N-by-2 matrix with kth row representing the
            %                 start and end indices of the scan points 
            %                 corresponding to the kth lineFeature 
            
            ranges = scan.Ranges;
            ranges(ranges < obj.RangeLowerBound) = 0;
            
            points = scan.Cartesian;
            
            [~, segments] = obj.findBreakpoints(ranges);
            [cornerIds, lineSegs] = obj.findCorners(ranges, points, segments);
            
            weights = ones(scan.Count, 1); % with current lidarScan object, point weights are always the same
            
            lineFeatures = zeros(size(lineSegs,1), 2);
            
            for k = 1:size(lineSegs,1)
                ids = lineSegs(k,1):lineSegs(k,2);
                pts = points(ids,:);
                wts = weights(ids);
                [rho, alpha] = obj.computeLineParameters(pts, wts);
                lineFeatures(k,:) = [rho, alpha];
            end
            
        end
        
        function [lineFeaturesMergedOut, scanSegMasksMergedOut] = mergeLineFeatures(obj, lineFeatures, scanSegs, scan) 
            %mergeLineFeatures
            numFeatures = size(lineFeatures, 1);
            lineFeaturesMerged = zeros(numFeatures, 2);
            scanSegMasksMerged = false(numFeatures, scan.Count);
            
            i = 1;
            for k = 1:numFeatures
                if k == 1
                    lineFeaturesMerged(i,:) = lineFeatures(k,:);
                    scanSegMasksMerged(i, scanSegs(k,1):scanSegs(k,2)) = true;
                    i = i + 1;
                else
                    f = lineFeatures(k,:);
                    rho = f(1);
                    alpha = f(2);
                    
                    fp = lineFeaturesMerged(i-1,:);
                    rhoPrev = fp(1);
                    alphaPrev = fp(2);
                    
                    % if close enough in line parameter space
                    if abs(rhoPrev - rho) < obj.MergeTolerance(1) && ...
                            abs(alphaPrev - alpha) < obj.MergeTolerance(2)
                        
                        scanSegMasksMerged(i-1, scanSegs(k,1):scanSegs(k,2)) = true;
                        
                        scanPts = scan.Cartesian(scanSegMasksMerged(i-1,:), :);
                        weights = ones(size(scanPts,1), 1);
                        [rhoNew, alphaNew] = obj.computeLineParameters(scanPts, weights);
                        lineFeaturesMerged(i-1,:) = [rhoNew, alphaNew];
                    else
                        lineFeaturesMerged(i,:) = lineFeatures(k,:);
                        scanSegMasksMerged(i, scanSegs(k,1):scanSegs(k,2)) = true;
                        i = i + 1;                        
                    end
                end
                    
            end
            
            % check number of points in each line feature
            numLines = i - 1;
            msk = false(numLines, 1);
            for j = 1:numLines
                if nnz(scanSegMasksMerged(j,:)) > obj.MinNumPointsPerSegment
                    msk(j) = true;
                end
            end
            
            lineFeaturesMergedOut = lineFeaturesMerged(msk,:);
            scanSegMasksMergedOut = scanSegMasksMerged(msk,:);
        end
        
    end
    
    methods (Access = {?nav.algs.internal.InternalAccess})
        
        function [outMsk, segments] = findBreakpoints(obj, ranges)
            %findBreakpoints Find line break points from lidar range data
            
            msk = abs(diff(diff(ranges))) < obj.DoubleDiffThreshold; % the "double diff" check
            msk = [0;0; msk];
            msk(ranges == 0) = 0;

            % if odd number of break points are identified, add last point
            ids_ = find([0; diff(msk)]);
            if msk(end) > 0.5 && mod(numel(ids_), 2) == 1
                ids = [ids_; numel(msk)];
            else
                ids= ids_;
            end

            segs = reshape(ids, 2, [])';

            % adjust index
            segs(:,2) = segs(:,2) - ones(size(segs,1), 1);

            % filter out segments with fewer than minimally requied number of  points
            segMsk = zeros(size(segs,1),1);
            outMsk = zeros(numel(ranges), 1);
            for j = 1:size(segs,1)
                if segs(j,2) - segs(j,1) >= obj.MinNumPoints
                    segMsk(j) = 1;
                    outMsk(segs(j,1):segs(j,2)) = 1;
                end
            end
            segments = segs(logical(segMsk),:);
            
        end
        
        function [cornerIndicesOut, linSegmentsOut] = findCorners(obj, ranges, carts, segments)
            %findCorners

            maxNumSegs = ceil(size(ranges,1));
            linSegments = zeros(maxNumSegs, 2);
            cornerIndices = zeros(numel(ranges), 1);

            % tighter threahold, this also tends to remove far away line 
            % features as they are less reliable
            threshold = obj.DoubleDiffThreshold * 0.2;

            kl = 1;
            kc = 1;
            for i = 1:size(segments,1)
                segIndices = segments(i,1):segments(i,2);
                rangeSeg = ranges(segIndices);
                cartSeg = carts(segIndices, :);

                % range data "double diff" check with tighter threshold
                segCornerIndices1 = segIndices(  ( abs([0;0;diff(diff(rangeSeg))]) > threshold)  );

                % change coordinates for current Cartesian segment
                delta = cartSeg(end,:) - cartSeg(1,:);
                theta = atan2(delta(2), delta(1));
                R = [cos(theta), sin(theta);
                    -sin(theta), cos(theta)];

                cartSegTformed = (R*cartSeg')';
                N = numel(segIndices);
                cartSegTformed(:,2) = cartSegTformed(:,2) - cartSegTformed(1,2)*ones(N, 1);
                prominence = obj.MinCornerProminence;

                % Cartesian points local min check
                TF1 = islocalmin(cartSegTformed(:,2), 'MinProminence', prominence);
                TF2 = islocalmax(cartSegTformed(:,2), 'MinProminence', prominence);
                TF = TF1|TF2;
                segCornerIndices2 = segIndices(TF);

                % combine
                segCornerIndices = unique([segCornerIndices1, segCornerIndices2]);
                N = numel(segCornerIndices);

                if isempty(segCornerIndices)
                    linSegments(kl,:) = segments(i,:);
                    kl = kl + 1;
                else
                    segs = [segments(i,1), segCornerIndices;
                           segCornerIndices, segments(i,2)]';
                    linSegments(kl : kl + N, :) = segs;
                    kl = kl + N + 1;
                end
                
                cornerIndices(kc : kc + N - 1) = segCornerIndices;
                kc = kc + N;
            end

            % throw away segments with too few number of points
            rmMask = true(kl-1, 1);
            for m = 1:kl-1
                if linSegments(m,2) - linSegments(m,1) < obj.MinNumPoints
                    rmMask(m) = 0;
                end
            end
            linSegmentsOut = linSegments(rmMask,:);

            cornerRmMask = ismember(cornerIndices(1:kc-1), linSegmentsOut(:));
            cornerIndicesOut = cornerIndices(cornerRmMask);

        end
        
        function [rho, alpha, quality] = computeLineParameters(obj, points, pointWeights) %#ok<INUSL>
            %computeLineParameters Compute parameters of a line feature
            %   Find rho (distance to origin) and alpha (angle) that
            %
            %   solves min sum( w_m *(rho - x_m * cos(alpha) - y_m * sin(alpha) ))^2
            %               m
            %
            %   This least-squares problem has an analytical solution

            x = points(:,1);
            y = points(:,2);

            W = sum(pointWeights);
            xBar = (1/W)*sum(x.*pointWeights);
            yBar = (1/W)*sum(y.*pointWeights);

            dx = x - xBar;
            dy = y - yBar;
            sXY = sum(pointWeights.*dx.*dy);
            sX = sum(pointWeights.*dx.*dx);
            sY = sum(pointWeights.*dy.*dy);

            alpha = 0.5 * atan2(-2*sXY, sY - sX);
            rho = xBar * cos(alpha) + yBar * sin(alpha);
            
            quality = var(ones(size(x))*rho - x.*cos(alpha) - y.*sin(alpha));

            % make sure all distances are nonnegative
            if rho < 0
                alpha = robotics.internal.wrapToPi(alpha + pi);
                rho = - rho;
            end
            
        end
        
    end
    
    methods (Static, Access = {?nav.algs.internal.InternalAccess})
        function params = getDefaultParams()
            %getDefaultParams
            params = struct('DoubleDiffThreshold', 0.05, ...
                            'MinCornerProminence', 0.1, ...
                            'MinNumPointsPerSegment', 3, ...
                            'MergeTolerance', [0.05, 0.1]);
        end
    end
end

