
%% xa --> xb --> xc at times ta --> tb --> tc
syms t ta tb tc
syms xa xb xc ya yb yc za zb zc
syms v_xa v_xb v_xc v_ya v_yb v_yc v_za v_zb v_zc
syms a0 a1 a2 a3 b0 b1 b2 b3

% (a,b)'s = 3x1 matrix coeffs ,

X_ab = @(t) a0 +   a1*t +   a2*t^2 + a3*t^3
V_ab = @(t) a1 + 2*a2*t + 3*a3*t^2

X_bc = @(t) b0 +   b1*t +   b2*t^2 + b3*t^3
V_bc = @(t) b1 + 2*b2*t + 3*b3*t^2


pa = [xa; ya; za];
pb = [xb; yb; zb];
pc = [xc; yc; zc];

va = [v_xa; v_ya; v_za];
vb = [v_xb; v_yb; v_zb];
vc = [v_xc; v_yc; v_zc];


ta = 0;
% tb = ;
% tc = ;

pos_eq1 = X_ab(ta) == pa; 
pos_eq2 = X_ab(tb) == pb;
pos_eq3 = X_bc(tb) == pb;
pos_eq4 = X_bc(tc) == pc;

vel_eq1 = V_ab(ta) == va;
vel_eq2 = V_ab(tb) == vb;
vel_eq3 = V_bc(tb) == vb;
vel_eq4 = V_bc(tc) == vc;


