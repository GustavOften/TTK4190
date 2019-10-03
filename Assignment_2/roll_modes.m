A2_model
%Dutch roll mode
dutch_roll_matrix = [A(1,1) A(1,4); A(4,1) A(4,4)];
eigs(dutch_roll_matrix);

%Spiral-divergence mode
eig_spiral = (A(4,4)*A(3,1)-A(4,1)*A(3,4))/A(3,1);

%Roll mode
eig_roll = A(3,3);