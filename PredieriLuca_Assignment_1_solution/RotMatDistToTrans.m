function [T, Thom] = RotMatDistToTrans (R, distbf)
    T = zeros(3,4);
    T = [R distbf];
    Thom = zeros(4,4);
    Thom = [R distbf; 0 0 0 1];
end