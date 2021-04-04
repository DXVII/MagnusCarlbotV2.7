syms d1 cth1 sth1 d2 cth2 sth2 a2 d3 cth3 sth3 a3 d4 cth4 sth4 d5 cth5 sth5 d6 cth6 sth6
syms th1 th2 th3 th4 th5 th6

T01 = [cth1 -sth1 0 0; sth1 cth1 0 0; 0 0 1 d1; 0 0 0 1];
TT01 = [cos(th1) -sin(th1) 0 0; sin(th1) cos(th1) 0 0; 0 0 1 d1; 0 0 0 1];
T12 = [cth2 -sth2 0 0; 0 0 -1 -d2; sth2 cth2 0 0; 0 0 0 1];
TT12 = [cos(th2) -sin(th2) 0 0; 0 0 -1 -d2; sin(th2) cos(th2) 0 0; 0 0 0 1];
T23 = [cth3 -sth3 0 a2; sth3 cth3 0 0; 0 0 1 d3; 0 0 0 1];
TT23 = [cos(th3) -sin(th3) 0 a2; sin(th3) cos(th3) 0 0; 0 0 1 d3; 0 0 0 1];
T34 = [cth4 -sth4 0 a3; sth4 cth4 0 0; 0 0 1 d4; 0 0 0 1];
TT34 = [cos(th4) -sin(th4) 0 a3; sin(th4) cos(th4) 0 0; 0 0 1 d4; 0 0 0 1];
T45 = [cth5 -sth5 0 0; 0 0 1 d5; -sth5 -cth5 0 0; 0 0 0 1];
TT45 = [cos(th5) -sin(th5) 0 0; 0 0 1 d5; -sin(th5) -cos(th5) 0 0; 0 0 0 1];
T56 = [cth6 -sth6 0 0; sth6 cth6 0 0; 0 0 1 d6; 0 0 0 1];
TT56 = [cos(th6) -sin(th6) 0 0; sin(th6) cos(th6) 0 0; 0 0 1 d6; 0 0 0 1];

T06 = T01*T12*T23*T34*T45*T56
TT06 = TT01*TT12*TT23*TT34*TT45*TT56;
TT06simp = simplify(TT06);
TT04 = TT01*TT12*TT23*TT34
TT04simp = simplify(TT04)