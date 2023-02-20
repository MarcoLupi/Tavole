%% CALCOLO JACOBIANO ANALITICO

    syms q1 q2 q3 q4 q5 q6 real
    
    % Matrici omogenee per ogni trasformazione 
    T_01 =  [cos(q1+dhparams(1,4)) -cos(dhparams(1,2))*sin(q1+dhparams(1,4))  sin(dhparams(1,2))*sin(q1+dhparams(1,4)) dhparams(1,1)*cos(q1+dhparams(1,4));
             sin(q1+dhparams(1,4))  cos(dhparams(1,2))*cos(q1+dhparams(1,4)) -sin(dhparams(1,2))*cos(q1+dhparams(1,4)) dhparams(1,1)*sin(q1+dhparams(1,4));
                     0                          sin(dhparams(1,2))                 cos(dhparams(1,2))               dhparams(1,3);
                     0                                 0                                  0                              1];
    
    T_12 =  [cos(q2+dhparams(2,4)) -cos(dhparams(2,2))*sin(q2+dhparams(2,4))  sin(dhparams(2,2))*sin(q2+dhparams(2,4)) dhparams(2,1)*cos(q2+dhparams(2,4));
             sin(q2+dhparams(2,4))  cos(dhparams(2,2))*cos(q2+dhparams(2,4)) -sin(dhparams(2,2))*cos(q2+dhparams(2,4)) dhparams(2,1)*sin(q2+dhparams(2,4));
                     0                          sin(dhparams(2,2))                 cos(dhparams(2,2))               dhparams(2,3);
                     0                                 0                                  0                              1];
    
    T_23 =  [cos(q3+dhparams(3,4)) -cos(dhparams(3,2))*sin(q3+dhparams(3,4))  sin(dhparams(3,2))*sin(q3+dhparams(3,4)) dhparams(3,1)*cos(q1+dhparams(3,4));
             sin(q3+dhparams(3,4))  cos(dhparams(3,2))*cos(q3+dhparams(3,4)) -sin(dhparams(3,2))*cos(q3+dhparams(3,4)) dhparams(3,1)*sin(q1+dhparams(3,4));
                     0                          sin(dhparams(3,2))                 cos(dhparams(3,2))               dhparams(3,3);
                     0                                 0                                  0                              1];
    
    T_34 =  [cos(q4+dhparams(4,4)) -cos(dhparams(4,2))*sin(q4+dhparams(4,4))  sin(dhparams(4,2))*sin(q4+dhparams(4,4)) dhparams(4,1)*cos(q4+dhparams(4,4));
             sin(q4+dhparams(4,4))  cos(dhparams(4,2))*cos(q4+dhparams(4,4)) -sin(dhparams(4,2))*cos(q4+dhparams(4,4)) dhparams(4,1)*sin(q4+dhparams(4,4));
                     0                          sin(dhparams(4,2))                 cos(dhparams(4,2))               dhparams(4,3);
                     0                                 0                                  0                              1];
    
    T_45 =  [cos(q5+dhparams(5,4)) -cos(dhparams(5,2))*sin(q5+dhparams(5,4))  sin(dhparams(5,2))*sin(q5+dhparams(5,4)) dhparams(5,1)*cos(q5+dhparams(5,4));
             sin(q5+dhparams(5,4))  cos(dhparams(5,2))*cos(q1+dhparams(5,4)) -sin(dhparams(5,2))*cos(q5+dhparams(5,4)) dhparams(5,1)*sin(q5+dhparams(5,4));
                     0                          sin(dhparams(5,2))                 cos(dhparams(5,2))               dhparams(5,3);
                     0                                 0                                  0                              1];
    
    T_5EE =  [cos(q6+dhparams(6,4)) -cos(dhparams(6,2))*sin(q6+dhparams(6,4))  sin(dhparams(6,2))*sin(q6+dhparams(6,4)) dhparams(6,1)*cos(q6+dhparams(6,4));
              sin(q6+dhparams(6,4))  cos(dhparams(6,2))*cos(q6+dhparams(6,4)) -sin(dhparams(6,2))*cos(q6+dhparams(6,4)) dhparams(6,1)*sin(q6+dhparams(6,4));
                     0                          sin(dhparams(6,2))                 cos(dhparams(6,2))               dhparams(6,3);
                     0                                 0                                  0                              1];
    
    T_EE = T_01*T_12*T_23*T_34*T_45*T_5EE;
    
    pos = [T_EE(1,4); T_EE(2,4); T_EE(3,4)];
    pitch = atan2(-T_EE(3,1), sqrt(T_EE(3,2)^2+T_EE(3,3)^2));
    yaw = atan2 (T_EE(3,2), T_EE(3,3));
    roll = atan2 (T_EE(2,1), T_EE(1,1));
    
    % Calcolo Jacobiano
    
    J_a = jacobian([pos(1); pos(2); pos(3); roll; pitch; yaw], [q1 q2 q3 q4 q5 q6]);