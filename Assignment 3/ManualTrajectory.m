syms tx ty tz

syms a0_x a1_x a2_x a3_x
syms a0_y a1_y a2_y a3_y
syms a0_z a1_z a2_z a3_z

syms b0_x b1_x b2_x b3_x
syms b0_y b1_y b2_y b3_y
syms b0_z b1_z b2_z b3_z

syms c0_x c1_x c2_x c3_x
syms c0_y c1_y c2_y c3_y
syms c0_z c1_z c2_z c3_z

syms Xi_x Xi_y Xi_z
syms Xf_x Xf_y Xf_z

syms XA_x XA_y XA_z
syms XB_x XB_y XB_z
syms XC_x XC_y XC_z
syms XD_x XD_y XD_z

syms VA_x VA_y VA_z
syms VB_x VB_y VB_z
syms VC_x VC_y VC_z
syms VD_x VD_y VD_z

traj_AB_x = a0_x + a1_x .* t + a2_x .* t.^2 + a3_x .* t.^3;
traj_AB_y = a0_y + a1_y .* t + a2_y .* t.^2 + a3_y .* t.^3;
traj_AB_z = a0_z + a1_z .* t + a2_z .* t.^2 + a3_z .* t.^3;

traj_BC_x = b0_x + b1_x .* t + b2_x .* t.^2 + b3_x .* t.^3;
traj_BC_y = b0_y + b1_y .* t + b2_y .* t.^2 + b3_y .* t.^3;
traj_BC_z = b0_z + b1_z .* t + b2_z .* t.^2 + b3_z .* t.^3;

traj_CD_x = c0_x + c1_x .* t + c2_x .* t.^2 + c3_x .* t.^3;
traj_CD_y = c0_y + c1_y .* t + c2_y .* t.^2 + c3_y .* t.^3;
traj_CD_z = c0_z + c1_z .* t + c2_z .* t.^2 + c3_z .* t.^3;

velo_AB_x = a1_x + a2_x .* t + a3_x .* t.^2;
velo_AB_y = a1_y + a2_y .* t + a3_y .* t.^2;
velo_AB_z = a1_z + a2_z .* t + a3_z .* t.^2;

velo_BC_x = b1_x + b2_x .* t + b3_x .* t.^2;
velo_BC_y = b1_y + b2_y .* t + b3_y .* t.^2;
velo_BC_z = b1_z + b2_z .* t + b3_z .* t.^2;

velo_CD_x = c1_x + c2_x .* t + c3_x .* t.^2;
velo_CD_y = c1_y + c2_y .* t + c3_y .* t.^2;
velo_CD_z = c1_z + c2_z .* t + c3_z .* t.^2;


% t b/w 2-5 secs
t_sp = 5/3;

t0_x = 0;
t1_x = t_sp;
t2_x = 2 * t_sp;
t3_x = 3 * t_sp;

t0_y = 0;
t1_y = t_sp;
t2_y = 2 * t_sp;
t3_y = 3 * t_sp;

t0_z = 0;
t1_z = t_sp;
t2_z = 2 * t_sp;
t3_z = 3 * t_sp;
