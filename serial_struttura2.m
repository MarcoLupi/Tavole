d1=0.2;    %[m]
d2=2*d1;     %[m]
d3=d2;
d4=d1/2;    %[m]

%distanze del baricentro dal sistema body [m]
o1g1=[0;d1/2;0];    
o2g2=[-d2/2;0;0];
o3g3=[0;0;d1/2];
o4g4=o1g1;
o5g5=o3g3;
o6g6=[0;0;-d1/2];

%parametri link cilindrico
m=0.5;  %massa [kg]
r=0.05; %raggio [m] 
h=d1;   %altezza[m]

%tensore d'inerzia assi body [kg*m^2]
[J_o1_vec,J_o2_vec,J_o3_vec,J_o4_vec,J_o5_vec,J_o6_vec]=inertia_tensor_compute_new(o1g1,o2g2,o3g3,o6g6,m,r,h);

%parametri DH (a, alpha, d, theta) 
dhparams= [ 0    -pi/2   d1    0
            d2   0       0     -pi/2
            0    pi/2    0     pi/2
            0    -pi/2   d3    0
            0    pi/2    0     0
            0    0       d4    0];


robot=rigidBodyTree;
robot.Gravity=[0 0 -9.81];
robot.DataFormat='column';

L1=rigidBody('L1');
L1.Mass=m;
L1.CenterOfMass=o1g1';  %vuole in riga
L1.Inertia=J_o1_vec;
J1=rigidBodyJoint('J1','revolute');
J1.JointAxis=[0; 0; 1];

L2=rigidBody('L2');
L2.Mass=m;
L2.CenterOfMass=o2g2';
L2.Inertia=J_o2_vec;
J2=rigidBodyJoint('J2','revolute');
J2.JointAxis=[0; 0; 1];
J2.HomePosition=-2/3*pi;
J2.PositionLimits = [-5*pi/4, pi/4];

L3=rigidBody('L3');
L3.Mass=m;
L3.CenterOfMass=o3g3';
L3.Inertia=J_o3_vec;
J3=rigidBodyJoint('J3','revolute');
J3.JointAxis=[0; 0; 1];
J3.HomePosition=pi;
J3.PositionLimits = [-pi/4, 5*pi/4];

L4=rigidBody('L4');
L4.Mass=m;
L4.CenterOfMass=o4g4';
L4.Inertia=J_o4_vec;
J4=rigidBodyJoint('J4','revolute');
J4.JointAxis=[0; 0; 1];

L5=rigidBody('L5');
L5.Mass=m;
L5.CenterOfMass=o5g5';
L5.Inertia=J_o5_vec;
J5=rigidBodyJoint('J5','revolute');
J5.JointAxis=[0; 0; 1];
J5.HomePosition=pi/3;
J3.PositionLimits = [-5*pi/4, 5*pi/4];

L6=rigidBody('L6');
L6.Mass=m;
L6.CenterOfMass=o6g6';
L6.Inertia=J_o6_vec;
J6=rigidBodyJoint('J6','revolute');
J6.JointAxis=[0; 0; 1];

setFixedTransform(J1,dhparams(1,:),'dh');
setFixedTransform(J2,dhparams(2,:),'dh');
setFixedTransform(J3,dhparams(3,:),'dh');
setFixedTransform(J4,dhparams(4,:),'dh');
setFixedTransform(J5,dhparams(5,:),'dh');
setFixedTransform(J6,dhparams(6,:),'dh');

L1.Joint=J1;
L2.Joint=J2;
L3.Joint=J3;
L4.Joint=J4;
L5.Joint=J5;
L6.Joint=J6;

addBody(robot,L1,'base');
addBody(robot,L2,'L1');
addBody(robot,L3,'L2');
addBody(robot,L4,'L3');
addBody(robot,L5,'L4');
addBody(robot,L6,'L5');

run calcolo_jacobiano_analitico2.m

% showdetails(robot);
%show(robot);
% axis([-0.3,0.3,-0.3,0.3,-0.2,1.1])
% figure(Name="Interactive GUI")
% gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);