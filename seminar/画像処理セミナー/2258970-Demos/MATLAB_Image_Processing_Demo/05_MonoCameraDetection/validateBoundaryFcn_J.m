function isGood = validateBoundaryFcn_J(params)
% *validateBoundaryFcn* function rejects some of the lane boundary curves
% computed using RANSAC algorithm in "findParabolicLaneBoundaries"
%   params => 3 parabolic parametersのベクトル
% 前フレームで検出した白線パラメータの値によって判断する処理も可能

if ~isempty(params)
    a = params(1);
    
    % 係数aが大きい場合（カーブが大きい）は 不採用
    isGood = abs(a) < 0.003; % a from ax^2+bx+c
else
    isGood = false;
end

end
