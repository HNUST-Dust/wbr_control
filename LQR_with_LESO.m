% 本程序用于求解LQR反馈矩阵lqr_k(L0)
% 对于每一个腿长L0，求解一次系统状态空间方程，然后求得反馈矩阵K
% 对于不同的K，对L0进行拟合，得到lqr_k

clear

L0s=0.1:0.01:0.35; % L0变化范围
Ks=zeros(2,6,length(L0s)); % 存放不同L0对应的K
Aes=zeros(8,8,length(L0s));
Bes=zeros(8,2,length(L0s));
Ls=zeros(8,6,length(L0s));

for step=1:length(L0s)

    % 所需符号量
    syms theta theta1 theta2; % theta1=dTheta, theta2=ddTheta
    syms x x1 x2 xd;
    syms phi phi1 phi2;
    syms T Tp N P Nm Pm Nf t;

    % 机器人结构参数
    R=0.06; L=L0s(step)/2; Lm=L0s(step)/2; l=0.15; mw=0.268/1; mp=3.117; M=25.0; Iw = 242e-6; Ip = 0.02*(1 + (L0s(step)/0.35)*0.3); Im = 0.3;
    g=9.8;

    % 进行物理计算
    Nm=M*(x2+(L+Lm)*(theta2*cos(theta)-theta1^2*sin(theta))-l*(phi2*cos(phi)-phi1^2*sin(phi)));
    Pm=M*g+M*((L+Lm)*(-theta1^2*cos(theta)-theta2*sin(theta))-l*(phi1^2*cos(phi)+phi2*sin(phi)));
    N=Nm+mp*(x2+L*(theta2*cos(theta)-theta1^2*sin(theta)));
    P=Pm+mp*g+mp*L*(-theta1^2*cos(theta)-theta2*sin(theta));

    equ1=x2-(T-N*R)/(Iw/R+mw*R);
    equ2=(P*L+Pm*Lm)*sin(theta)-(N*L+Nm*Lm)*cos(theta)-T+Tp-Ip*theta2;
    equ3=Tp+Nm*l*cos(phi)+Pm*l*sin(phi)-Im*phi2;
    [x2,theta2,phi2]=solve(equ1,equ2,equ3,x2,theta2,phi2);

    % 求得雅克比矩阵，然后得到状态空间方程
    Ja=jacobian([theta1;theta2;x1;x2;phi1;phi2],[theta theta1 x x1 phi phi1]);
    Jb=jacobian([theta1;theta2;x1;x2;phi1;phi2],[T Tp]);
    A=vpa(subs(Ja,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));
    B=vpa(subs(Jb,[theta theta1 x x1  phi phi1],[0 0 0 0 0 0]));

    % 离散化
    [G,H]=c2d(eval(A),eval(B),0.001);

    % 定义权重矩阵Q, R
    Q=diag([1000 10 100 200 12000 200]);
    R=diag([2.5 15]);

    % 求解反馈矩阵K
    Ks(:,:,step)=dlqr(G,H,Q,R);


    %% ====================  LESO 观测器增益设计（极点配置法） ====================
    G_design = double(G);   
    H_design = double(H);

    % 系统维度
    n = 6;      % 状态维度
    m = 2;      % 输入维度
    p = 6;      % 输出维度（全状态可测）
    q = 2;      % 扩张状态（扰动）维度，取与输入相同

    % 构造离散扩张系统矩阵 A_e (8x8), B_e (8x2), C_e (6x8)
    A_e = [G_design, H_design;
        zeros(q, n), eye(q)];
    B_e = [H_design;
        zeros(q, m)];
    C_e = [eye(p), zeros(p, q)];
    Aes(:,:,step)=A_e;
    Bes(:,:,step)=B_e;


    % 选择观测器极点（共 n+q = 8 个极点）
    sys_poles = eig(A_e);
    fast_factor = 0.4;   % 倍数越小，观测器越快（但增益可能很大）
    obs_poles_mod = abs(sys_poles) * fast_factor;
    % 对于模已经小于 fast_factor 的极点，不再减小
    obs_poles_mod = max(obs_poles_mod, 0.2);
    % 扰动对应的两个极点（最后两个状态）
    obs_poles_mod(end-1:end) = 0.985;

    obs_poles = obs_poles_mod;   

    % 使用 place 计算对偶系统的反馈增益
    % 对偶系统： (A_e^T, C_e^T)
    K_place = place(A_e', C_e', obs_poles);
    % 观测器增益 L 是 K_place 的转置
    L_obs = K_place';

    Ls(:,:,step)=L_obs;

end

% 对K的每个元素关于L0进行拟合
K=sym('K',[2 6]);
syms L0;
for x=1:2
    for y=1:6
        p=polyfit(L0s,reshape(Ks(x,y,:),1,length(L0s)),3);
        K(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

% 输出到m函数
matlabFunction(K,'File','lqr_k_calc');

% 对Ae的每个元素关于L0进行拟合
Ae=sym('Ae',[8 8]);
for x=1:8
    for y=1:8
        p=polyfit(L0s,reshape(Aes(x,y,:),1,length(L0s)),3);
        Ae(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

% 输出到m函数
matlabFunction(Ae,'File','leso_A_calc');

% 对Be的每个元素关于L0进行拟合
Be=sym('Be',[8 2]);
for x=1:8
    for y=1:2
        p=polyfit(L0s,reshape(Bes(x,y,:),1,length(L0s)),3);
        Be(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

% 输出到m函数
matlabFunction(Be,'File','leso_B_calc');

% 对Ls的每个元素关于L0进行拟合
L=sym('L',[8 6]);
for x=1:8
    for y=1:6
        p=polyfit(L0s,reshape(Ls(x,y,:),1,length(L0s)),3);
        L(x,y)=p(1)*L0^3+p(2)*L0^2+p(3)*L0+p(4);
    end
end

% 输出到m函数
matlabFunction(L,'File','leso_L_calc');


% 代入L0=0.07打印矩阵K
vpa(subs(K,L0,0.28))



% 显示结果
disp(['观测器增益 L (', num2str(n+q), 'x', num2str(p), ') 为:']);
disp(L_obs);

% 验证估计误差矩阵 A_e - L_obs*C_e 的特征值是否在设计位置
eig_obs = eig(A_e - L_obs*C_e);
disp('观测器闭环极点（应接近设计值）：');
disp(eig_obs);

