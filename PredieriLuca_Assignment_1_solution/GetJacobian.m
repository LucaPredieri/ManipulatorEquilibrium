function J = GetJacobian(Thom, com, jointType, current_link)

    numLinks = length(jointType);
    J = zeros(6, numLinks);
    r = zeros(3, numLinks);
    k = zeros(3 ,1);
    z_axis = [0 ;0 ; 1];
    for i = 1 : current_link
        %Call GetJacobianColumn for each joint
        r(:, i) = com(1:3) - Thom{1}((1:3),4,i); % com wrt base - distbf(wrt base)
        k = Thom{1}((1:3),(1:3),i) * z_axis;
        J(:, i) = GetJacobianColumn(k, r(:,i), jointType(i));
    end
end