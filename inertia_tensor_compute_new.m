function [J_o1_vec, J_o2_vec, J_o3_vec, J_o4_vec, J_o5_vec, J_o6_vec] = inertia_tensor_compute_new(o1g1,o2g2,o3g3,o6g6,m,r,h)


o1g1_ss=[0         -o1g1(3) o1g1(2);     %forma skewsymmetric della distnaza del baricentro in assi body  
         o1g1(3)   0        -o1g1(1);
         -o1g1(2)  o1g1(1)  0];

o2g2_ss=[0         -o2g2(3) o2g2(2);     %forma skewsymmetric della distnaza del baricentro in assi body  
         o2g2(3)   0        -o2g2(1);
         -o2g2(2)  o2g2(1)  0];

o3g3_ss=[0         -o3g3(3) o3g3(2);     %forma skewsymmetric della distnaza del baricentro in assi body  
         o3g3(3)   0        -o3g3(1);
         -o3g3(2)  o3g3(1)  0];

o6g6_ss=[0         -o6g6(3) o6g6(2);     %forma skewsymmetric della distnaza del baricentro in assi body  
         o6g6(3)   0        -o6g6(1);
         -o6g6(2)  o6g6(1)  0];

o4g4_ss=o1g1_ss;
o5g5_ss=o3g3_ss;


%tensore d'inerzia baricentrico
Jxx=m*(3*r^2+h^2)/12;       
Jyy=Jxx;
Jzz=0.5*m*r^2;
J_g=blkdiag(Jxx,Jyy,Jzz);    

%orientazione del sistema di riferimento baricentrico rispetto al body
R_b1g1=[1 0 0;
        0 0 -1;
        0 1 0];            

R_b2g2=[0 0 1;              
        1 0 0;
        0 1 0];

R_b3g3=eye(3);
R_b4g4=R_b1g1;
R_b5g5=R_b3g3;

R_b6g6=R_b3g3;

%tensore d'inerzia in assi body
J_o1= m*(o1g1_ss)*o1g1_ss'+R_b1g1*J_g*R_b1g1';   
J_o2= m*(o2g2_ss)*o2g2_ss'+R_b2g2*J_g*R_b2g2';
J_o3= m*(o3g3_ss)*o3g3_ss'+R_b3g3*J_g*R_b3g3';
J_o4= m*(o4g4_ss)*o4g4_ss'+R_b4g4*J_g*R_b4g4';
J_o5= m*(o5g5_ss)*o5g5_ss'+R_b5g5*J_g*R_b5g5';
J_o6= m*(o6g6_ss)*o6g6_ss'+R_b6g6*J_g*R_b6g6';

%tensore d'inerzia in assi body, forma vettore riga
J_o1_vec= [J_o1(1,1) J_o1(2,2) J_o1(3,3) J_o1(2,3) J_o1(1,3) J_o1(1,2)];
J_o2_vec= [J_o2(1,1) J_o2(2,2) J_o2(3,3) J_o2(2,3) J_o2(1,3) J_o2(1,2)];
J_o3_vec= [J_o3(1,1) J_o3(2,2) J_o3(3,3) J_o3(2,3) J_o3(1,3) J_o3(1,2)];
J_o4_vec= [J_o4(1,1) J_o4(2,2) J_o4(3,3) J_o4(2,3) J_o4(1,3) J_o4(1,2)];
J_o5_vec= [J_o5(1,1) J_o5(2,2) J_o5(3,3) J_o5(2,3) J_o5(1,3) J_o5(1,2)];
J_o6_vec= [J_o6(1,1) J_o6(2,2) J_o6(3,3) J_o6(2,3) J_o6(1,3) J_o6(1,2)];

end