%% Computing center of mass.
clc
clear

Assembly_DataFile
smiData_es1=smiData; %CAD data for exercise 1
Assembly_2_DataFile
smiData_es2=smiData; %CAD data for exercise 2
%% Frame distances cell
% This cell contains the distances from the first link frame to the second
% link frame projected on the base frame. These quantities are needed for
% computing the Transformation matrices. 
distancebf = cell(2,5);
%% Relative Angles between link1 and link2
angles = cell(2,5);
angles{1,1} = [pi/2, -pi/2];    %Relative angles for sub-exercise 1.1
angles{1,2} = [0, pi/2];        %Relative angles for sub-exercise 1.2
angles{1,3} = [pi/6, pi/3];     %Relative angles for sub-exercise 1.3
angles{1,4} = [pi/6, pi/3];     %Relative angles for sub-exercise 1.4
angles{1,5} = [pi/6, -pi/3];    %Relative angles for sub-exercise 1.5
angles{2,1} = [0, pi/4];        %Relative angles for sub-exercise 2.1
angles{2,2} = [0, pi/2];        %Relative angles for sub-exercise 2.2
angles{2,3} = [0, pi/4];        %Relative angles for sub-exercise 2.3
angles{2,4} = [0, pi/4];        %Relative angles for sub-exercise 2.4
angles{2,5} = [0, pi/4];        %Relative angles for sub-exercise 2.5
%% Z, Y, X axis
z_axis = [0, 0 ,1];
y_axis = [0, 1, 0];
x_axis = [1, 0, 0];
%% Jacobians Cell
J = cell(2,5);
%% Gravity vector
g = [0; -9.81; 0]; %m/s^2
%% Building the Rotational and Transformation Matrices cells
R = cell(2,5);
T = cell(2,5);

for i = 1:2 % exercise from 1 to 2
    for j = 1:5 % sub-exercise from 1 to 5

        if (i == 1) % exercise number 1

            %axis and angle of rotation of the first link frame and the
            %base
            axang_1 = [z_axis, angles{i,j}(1,1)];  
            % in this case the rotation axis between frames is always the
            % z-axis about an angle defined by the current configuration

        else % exercise number 2

            %Denavit Hartenberg rule states that the axis of
            % traslation must be the z-axis. So, in this case 
            % the rotation axis between frames is always the same...
            axang_1 = [y_axis, pi/2]; 
        end

        %axis and angle of rotation of the second link frame and the
        %base
        axang_2 = [z_axis, angles{i,j}(1,2) + angles{i,j}(1,1)];

        % Rotation matrices between link1 frame and base frame
        R{i,j}(:,:,1) = axang2rotm(axang_1);
        % Rotation matrices between link2 frame and base frame
        R{i,j}(:,:,2) = axang2rotm(axang_2);

        if (i == 1) 
            distancebf{i,j} = R{i,j}(:,:,1)*(smiData_es1.RigidTransform(5).translation)';
        else  
            distancebf{i,j} = (smiData_es2.RigidTransform(5).translation)';
        end
        
        % Transformation matrices between link1 frame and base frame
        T{i,j}(:,:,1) = RotMatDistToTrans(R{i,j}(:,:,1), zeros(3, 1));
        % Transformation matrices between link2 frame and base frame
        T{i,j}(:,:,2) = RotMatDistToTrans(R{i,j}(:,:,2), distancebf{i,j});
    end 
end

%% Joint-Types
jointTypes_es_1 = [2, 2]; % Revolute joints
jointTypes_es_2 = [1, 2]; % Prismatic and rotational joints

%% CAD Data of Bodies.

% The COM from the CAD depends on the kind of joint because of the Denavit
% Hartember rule. That's why I computed multiple COM. 

% Link 1
m1 = smiData_es1.Solid(2).mass * 0.453592 ; %Kg. Link1 mass. 
com1_es1 = smiData_es1.Solid(2).CoM; %mm. Link1 COM computed for exercise 1
com1_es2 = smiData_es2.Solid(3).CoM; %mm. Link1 COM computed for exercise 2

% Motor exercise 1
m_motor= smiData_es1.Solid(3).mass * 0.453592 ; %Kg. Motor mass
com_motor_es1 = smiData_es1.Solid(3).CoM; %mm. Motors COM computed for exercise 1
% Motor exercise 2
com_motor_es2_1 = smiData_es2.Solid(2).CoM; %mm. First motor COM computed for exercise 2
com_motor_es2_2 = smiData_es2.Solid(4).CoM; %mm. Second motor COM computed for exercise 2

% Link 2
m2 =smiData_es1.Solid(4).mass * 0.453592 ; %Kg. Link 2 mass. 
com2_es1 = smiData_es1.Solid(4).CoM; %mm. Link2 COM computed for exercise 1
com2_es2 = smiData_es2.Solid(5).CoM; %mm. Link2 COM computed for exercise 2

%% Computing COM of links for the exercise 1 in the starting configuration.
% In the computation I considered the bodies link1 + motor1 and link2 + motor2 as a single
% object. 
COM_link_1_es1=ComputeCenterOfMass(m1,m_motor,com1_es1,com_motor_es1);
COM_link_2_es1=ComputeCenterOfMass(m2,m_motor,com2_es1,com_motor_es1);
%% Computing COM of links for the exercise 2 in the starting configuration.
COM_link_1_es2=ComputeCenterOfMass(m1,m_motor,com1_es2,com_motor_es2_1);
COM_link_2_es2=ComputeCenterOfMass(m2,m_motor,com2_es2,com_motor_es2_2);
%% Putting it into a cell

COM_link = cell(2,2);
COM_link{1,1} = COM_link_1_es1; 
COM_link{1,2} = COM_link_1_es2; 
COM_link{2,1} = COM_link_2_es1; 
COM_link{2,2} = COM_link_2_es2; 

%% Computing all the COM wrt the base frame

COM_link_wrt_base = cell(2, 5);
for i=1:2
    for j=1:5
        COM_link_wrt_base{i,j}(:,:,1) = T{i,j}(:,:,1)* [COM_link{1,i} ; 1];
        COM_link_wrt_base{i,j}(:,:,2) = T{i,j}(:,:,2)* [COM_link{2,i}; 1];
    end
end

%% ASSIGNMENT 
%% COMPUTING JACOBIAN MATRICES OF THE LINK'S CENTER OF MASSES WRT THE BASE FRAME
for i=1:2
    for j=1:5
        if (i == 1)
            J{i,j}(:,:,1) = GetJacobian(T(i,j), COM_link_wrt_base{i,j}(:,:,1), jointTypes_es_1, 1);
            J{i,j}(:,:,2) = GetJacobian(T(i,j), COM_link_wrt_base{i,j}(:,:,2), jointTypes_es_1, 2);
        else
            J{i,j}(:,:,1) = GetJacobian(T(i,j), COM_link_wrt_base{i,j}(:,:,1), jointTypes_es_2, 1);
            J{i,j}(:,:,2) = GetJacobian(T(i,j), COM_link_wrt_base{i,j}(:,:,2), jointTypes_es_2, 2);
        end
    end
end

%% Exercise 1.1
% Only weight forces are active
W1 = [zeros(3,1); m1*g];
W2 = [zeros(3,1); m2*g];
tau_1_1 = - transpose(J{1,1}(:,:,1))*W1 - transpose(J{1,1}(:,:,2))*W2; %N*m

%% Exercise 1.2
% Only weight forces are active
tau_1_2 = - transpose(J{1,2}(:,:,1))*W1 - transpose(J{1,2}(:,:,2))*W2;

%% Exercise 1.3

F_ext_1_3_p1 = [-0.7 ; -0.5 ; 0]; %N

% Computing the rigid body Jacobian for point P1 in the current configuration. (First question)
P1_wrt_f2 = [1000;0;0];
P1_wrt_base= T{1,3}(:,:,2) * [P1_wrt_f2;1];
S_a = ComputeRigidBodyJ(COM_link_wrt_base{1 ,3}(:,:,2) - P1_wrt_base);

% Computing the rigid body Jacobian for point P2 in the current configuration. (Second question)
P2_wrt_f2 = [300;0;0];
P2_wrt_base= T{1,3}(:,:,2) * [P2_wrt_f2;1];
S_b = ComputeRigidBodyJ(COM_link_wrt_base{1 ,3}(:,:,2) - P2_wrt_base);

W1_3 = [0 ; 0 ; 0; F_ext_1_3_p1];

tau_1_3_a = - transpose(J{1,3}(:,:,2)) * transpose(S_a) * W1_3;   %first question
tau_1_3_b = - transpose(J{1,3}(:,:,2)) * transpose(S_b) * W1_3;   %second question

% The second vector of generalized actuator forces is lower (in absolute
% value) because the momentum generated by the force in P2 is lower wrt the
% momentum  generated by the force acting in P1. 

%% Exercise 1.4

F_ext_1_4_p3 = [1.5 ; -0.3 ; 0];    %N, acting on P3
M_ext_1_4 = [0; 0; 1200];           %N*mm, acting about an axis pssing through P1

% Computing the rigid body Jacobian for point P3 in the current configuration.
P3_wrt_f1 = [900;0;0];
P3_wrt_base= T{1,4}(:,:,1) * [P3_wrt_f1;1];
S_p3 = ComputeRigidBodyJ(COM_link_wrt_base{1 ,3}(:,:,1)-P3_wrt_base);

W1_4_a = [zeros(3,1); F_ext_1_4_p3];
W1_4_b = [M_ext_1_4; zeros(3,1)];

tau_1_4 = - (transpose(J{1,4}(:,:,1)) * transpose(S_p3) * W1_4_a + transpose(J{1,4}(:,:,2)) * W1_4_b);

%% Exercise 1.5

F_ext_1_5_p2 = [1.2 ; -0.2; 0]; %N, acting on P2
F_ext_1_5_p3 = [-0.4 ; 1.2 ; 0]; %N, acting on P3

% Computing the rigid body Jacobian for point P2 in the current configuration.
P2_wrt_base= T{1,5}(:,:,2) * [P2_wrt_f2;1];
S_p2 = ComputeRigidBodyJ(COM_link_wrt_base{1 ,5}(:,:,2) - P2_wrt_base);

% Computing the rigid body Jacobian for point P3 in the current configuration.
P3_wrt_base= T{1,5}(:,:,1) * [P3_wrt_f1;1];
S_p3 = ComputeRigidBodyJ(COM_link_wrt_base{1 ,5}(:,:,1) - P3_wrt_base);


W1_5_p2 = [zeros(3,1); F_ext_1_5_p2 ];
W1_5_p3 = [zeros(3,1); F_ext_1_5_p3 ];

tau_1_5 = - (transpose(J{1,5}(:,:,1)) * transpose(S_p3) * W1_5_p3 + transpose(J{1,5}(:,:,2)) * transpose(S_p2) * W1_5_p2) - (transpose(J{1,5}(:,:,1)) * W1 + transpose(J{1,5}(:,:,2)) * W2);
    
%% Exercise 2.1

% Only weight forces are active
tau_2_1 = - transpose(J{2,1}(:,:,2)) * W2;

%% Exercise 2.2

% Only weight forces are active
tau_2_2 = - transpose(J{2,2}(:,:,2)) * W2;

%% Exercise 2.3

F_ext_2_3_p1 = [-0.8 ; -0.8; 0]; %N, acting on P1

% Computing the rigid body Jacobian for point P1 in the current configuration.
P1_wrt_base= T{2,3}(:,:,2) * [P1_wrt_f2;1];
S_p1 = ComputeRigidBodyJ(COM_link_wrt_base{2, 3}(:,:,2) - P1_wrt_base);

W2_3 = [0 ; 0 ; 0; F_ext_2_3_p1];

tau_2_3 = - transpose(J{2,3}(:,:,2)) * transpose(S_p1) * W2_3;

%% Exercise 2.4

F_ext_2_4_p2 = [-0.8 ; -0.2; 0];    %N, acting on P2
M_ext_2_4_p1 = [0; 0; 500];         %N*mm, acting on P1

% Computing the rigid body Jacobian for point P1 in the current configuration.
S_p2 = ComputeRigidBodyJ(COM_link_wrt_base{2, 4}(:,:,2)-P2_wrt_base);

W2_4_p2 = [0 ; 0 ; 0; F_ext_2_4_p2];
W2_4_p1 = [M_ext_2_4_p1; zeros(3,1)];

tau_2_4 = - (transpose(J{2,4}(:,:,2))*transpose(S_p2) * W2_4_p2 + transpose(J{2,4}(:,:,2))*W2_4_p1) ;
%% Exercise 2.5

F_ext_2_5_p1 = [0.5 ; -0.6; 0]; %N, acting on P1
F_ext_2_5_p2 = [1.0 ; -0.4; 0]; %N, acting on P2

% Computing the rigid body Jacobian for point P1 in the current configuration.
S_p1 = ComputeRigidBodyJ(COM_link_wrt_base{2, 5}(:,:,2) - P1_wrt_base);
% Computing the rigid body Jacobian for point P2 in the current configuration.
S_p2 = ComputeRigidBodyJ(COM_link_wrt_base{2, 5}(:,:,2) - P2_wrt_base);

W2_5_p1 = [0 ; 0 ; 0; F_ext_2_5_p1];
W2_5_p2 = [0 ; 0 ; 0; F_ext_2_5_p2];

tau_2_5 = - (transpose(J{2,5}(:,:,2)) * transpose(S_p2) * W2_5_p2 + transpose(J{2,5}(:,:,2)) * transpose(S_p1) * W2_5_p1 + transpose(J{2,5}(:,:,2)) * W2) ;
%%
clear j
clear i