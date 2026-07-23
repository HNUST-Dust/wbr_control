%% RoboMaster 平衡步兵控制复现代码骨架

clear; clc;

%% 1. 符号建模：由式(1.3)(1.4)(1.5)(1.6)(1.7)(1.8)(1.9)求 f(x,u)
% x 轮轴水平位置 m
% θ 腿部（摆杆）相对竖直方向倾角 rad
% φ 机身（上半身）俯仰角 rad
% ẋ 轮轴速度 m/s
% θ̇ 腿部角速度 rad/s
% φ̇ 机身角速度 rad/s
% ẍ 轮轴加速度 m/s²
% θ̈ 腿部角加速度 rad/s²
% φ̈ 机身角加速度 rad/s²
% mw 单个轮子质量 kg
% mp 腿部质量 kg
% M 机身质量 kg
% Iw 轮子转动惯量 kg·m²
% Ip 腿部转动惯量 kg·m²
% IM 机身转动惯量 kg·m²
% R 轮半径 m
% L 轮轴到腿质心距离 m
% LM 腿关节到机身质心距离 m
% l 机身关节到机身质心距离 m
% N 地面对腿的水平作用力 N
% P 地面对腿的竖直作用力 N
% NM 机身对腿的水平作用力 N
% PM 机身对腿的竖直作用力 N
% T 轮毂电机输出力矩 N·m
% Tp 胯关节输出力矩 N·m

syms theta dtheta ddtheta x dx ddx phi dphi ddphi real
syms T Tp real
syms mw Iw R mp Ip L LM M IM l g real
syms N P NM PM real

% 运动学二阶导
dd_x_p = ddx + L*(ddtheta*cos(theta) - dtheta^2*sin(theta));
dd_z_p = -L*(ddtheta*sin(theta) + dtheta^2*cos(theta));

dd_x_M = ddx + (L+LM)*(ddtheta*cos(theta)-dtheta^2*sin(theta)) ...
             - l*(ddphi*cos(phi)-dphi^2*sin(phi));

dd_z_M = -(L+LM)*(ddtheta*sin(theta)+dtheta^2*cos(theta)) ...
         - l*(ddphi*sin(phi)+dphi^2*cos(phi));

% 动力学方程
eq1 = ddx == (T - N*R)/(Iw/R + mw*R);                 % 1.3
eq2 = N - NM == mp*dd_x_p;                            % 1.4
eq3 = P - PM - mp*g == mp*dd_z_p;                     % 1.5
eq4 = Ip*ddtheta == (P*L + PM*LM)*sin(theta) ...
                  - (N*L + NM*LM)*cos(theta) - T + Tp; % 1.6
eq5 = NM == M*dd_x_M;                                 % 1.7
eq6 = PM - M*g == M*dd_z_M;                           % 1.8
eq7 = IM*ddphi == Tp + NM*l*cos(phi) + PM*l*sin(phi); % 1.9

sol = solve([eq1,eq2,eq3,eq4,eq5,eq6,eq7], ...
            [ddx, ddtheta, ddphi, N, P, NM, PM], ...
            'Real', true, 'ReturnConditions', false);

f = [dtheta;
     simplify(sol.ddtheta);
     dx;
     simplify(sol.ddx);
     dphi;
     simplify(sol.ddphi)];

X = [theta; dtheta; x; dx; phi; dphi];
U = [T; Tp];

A_sym = jacobian(f, X);
B_sym = jacobian(f, U);

% 平衡点：theta=0,dtheta=0,dx=0,phi=0,dphi=0,T=0,Tp=0
A_eq = simplify(subs(A_sym, ...
    [theta,dtheta,dx,phi,dphi,T,Tp], ...
    [0,0,0,0,0,0,0]));

B_eq = simplify(subs(B_sym, ...
    [theta,dtheta,dx,phi,dphi,T,Tp], ...
    [0,0,0,0,0,0,0]));

disp('A symbolic ='); pretty(A_eq)
disp('B symbolic ='); pretty(B_eq)

%% 2. 文章给出的 L0 = 0.18 m 数值模型与 LQR

A = [0        1 0 0       0        0;
     265.9556 0 0 0       80.6327  0;
     0        0 0 1       0        0;
    -25.4562  0 0 0       1.8637   0;
     0        0 0 0       0        1;
     156.6952 0 0 0       183.0614 0];

B = [0        0;
    -15.1389 13.8563;
     0        0;
     2.1208  -0.7158;
     0        0;
    -4.2238  16.8001];

Q = diag([1, 1, 500, 100, 5000, 1]);
R = [1 0; 0 0.25];

Co = ctrb(A,B);
fprintf('rank(ctrb) = %d\n', rank(Co));

K = lqr(A,B,Q,R)

% 文章中的控制律：u = K*(xd - x)
x_now = zeros(6,1);
x_ref = [0; 0; 1.0; 0; 0; 0];   % 期望位置示例
u = K * (x_ref - x_now);

%% 3. K(L0) 三次多项式拟合：式(1.11)

% 假设你已经在多个腿长 L0_samples 下算出了 K_samples
% K_samples 的尺寸：2 x 6 x n
L0_samples = [0.15 0.16 0.17 0.18 0.19];  % 示例
K_samples = zeros(2,6,numel(L0_samples)); % 换成你实际算出的 K

Pcoef = zeros(2,6,4); % p3,p2,p1,p0，MATLAB polyval 顺序

for i = 1:2
    for j = 1:6
        y = squeeze(K_samples(i,j,:)).';
        Pcoef(i,j,:) = polyfit(L0_samples, y, 3);
    end
end

% 某一腿长下恢复 K(L0)
L0_now = 0.18;
K_L0 = zeros(2,6);
for i = 1:2
    for j = 1:6
        K_L0(i,j) = polyval(squeeze(Pcoef(i,j,:)), L0_now);
    end
end

%% 4. 五杆机构正运动学：求 phi2

function phi2 = solve_phi2(xB,yB,xD,yD,l2,l3)
    lBD = sqrt((xD-xB)^2 + (yD-yB)^2);

    A0 = 2*l2*(xD-xB);
    B0 = 2*l2*(yD-yB);
    C0 = l2^2 + lBD^2 - l3^2;

    phi2 = 2*atan((B0 + sqrt(A0^2 + B0^2 - C0^2))/(A0 + C0));
end

%% 5. VMC：T_joint = J' * F_task

% x_task = [L0; phi0]
% q = [phi1; phi4]
% F_task = [F; Tp]

function T_joint = vmc_torque(J, F, Tp)
    F_task = [F; Tp];
    T_joint = J.' * F_task;
end

%% 6. 若你有正运动学函数，可用数值差分求 J

function J = numerical_jacobian_fk(fk, q)
    eps0 = 1e-6;
    J = zeros(2,2);

    for k = 1:2
        dq = zeros(2,1);
        dq(k) = eps0;

        xp = fk(q + dq);
        xm = fk(q - dq);

        J(:,k) = (xp - xm)/(2*eps0);
    end
end

%% 7. 离地检测支持力：第 3 节公式

function FN = support_force(F, Tp, theta, L0, mw, g, ddzM, ddL0, dL0, ddtheta, dtheta)

    P = F*cos(theta) + Tp*sin(theta)/L0;

    ddzw = ddzM ...
         - ddL0*cos(theta) ...
         + 2*dL0*dtheta*sin(theta) ...
         + L0*ddtheta*sin(theta) ...
         + L0*dtheta^2*cos(theta);

    FN = P + mw*g + mw*ddzw;
end

% 离地判断
% is_flying = FN < 20;