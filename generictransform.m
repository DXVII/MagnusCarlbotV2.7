syms a cal sal d cth sth

Rx = [1 0 0 0; 0 cal -sal 0; 0 sal cal 0; 0 0 0 1]
Dx = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1]
Dz = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1]
Rz = [cth -sth 0 0; sth cth 0 0; 0 0 1 0; 0 0 0 1]

T = Rx*Dx*Dz*Rz
% inv(T)