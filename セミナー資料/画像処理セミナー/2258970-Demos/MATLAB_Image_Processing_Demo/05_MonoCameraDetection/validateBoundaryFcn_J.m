function isGood = validateBoundaryFcn_J(params)
% *validateBoundaryFcn* function rejects some of the lane boundary curves
% computed using RANSAC algorithm in "findParabolicLaneBoundaries"
%   params => 3 parabolic parameters�̃x�N�g��
% �O�t���[���Ō��o���������p�����[�^�̒l�ɂ���Ĕ��f���鏈�����\

if ~isempty(params)
    a = params(1);
    
    % �W��a���傫���ꍇ�i�J�[�u���傫���j�� �s�̗p
    isGood = abs(a) < 0.003; % a from ax^2+bx+c
else
    isGood = false;
end

end
