%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #1                         %
%  EFK-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;

%==== TEST: Setup uncertianty parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;
listmahalanobis = []
listeucleadian= []

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====

tempb = [arr(1),arr(3),arr(5) , arr(7) , arr(9), arr(11)];
tempr = [arr(2),arr(4),arr(6) , arr(8) , arr(10), arr(12)];

tempb = tempb + pose(3);
costempb = cos(tempb);
sintempb = sin(tempb);

temp = pose(1) + tempr.*costempb;
temp1 = pose(2) +  tempr.*sintempb;

landmark = [temp(1),temp1(1),temp(2),temp1(2),temp(3),temp1(3),temp(4),temp1(4),temp(5),temp1(5),temp(6),temp1(6)]
landmark = landmark';

ones = [1;1;1;1;1;1]
zerose = [0;0;0;0;0;0]
firstrowg = [ ones zerose -1.*tempr'.*sintempb' -1*tempr'.*sintempb' costempb'];
secondrowg = [zerose ones 1.*tempr'.*costempb' 1*tempr'.*costempb' sintempb'];



sigmacov1 = diag(([0.02^2, 0.02^2, 0.1^2, sig_beta2, sig_alpha2]));
sigmacov2 = diag(([0.02^2, 0.02^2, 0.1^2, sig_beta2, sig_alpha2]));
sigmacov3 = diag(([0.02^2, 0.02^2, 0.1^2, sig_beta2,sig_alpha2]));
sigmacov4 = diag(([0.02^2, 0.02^2, 0.1^2, sig_beta2, sig_alpha2]));
sigmacov5 = diag(([0.02^2, 0.02^2, 0.1^2, sig_beta2, sig_alpha2]));
sigmacov6 = diag(([0.02^2, 0.02^2, 0.1^2, sig_beta2, sig_alpha2]));

firstmat = [firstrowg(1,:);secondrowg(1,:)]*sigmacov1*[firstrowg(1,:);secondrowg(1,:)]';
secmat  = [firstrowg(2,:);secondrowg(2,:)]*sigmacov2*[firstrowg(2,:);secondrowg(2,:)]';
thirdmat = [firstrowg(3,:);secondrowg(3,:)]*sigmacov3*[firstrowg(3,:);secondrowg(3,:)]';
fourthmat = [firstrowg(4,:);secondrowg(4,:)]*sigmacov4*[firstrowg(4,:);secondrowg(4,:)]';
fifthmat = [firstrowg(5,:);secondrowg(5,:)]*sigmacov5*[firstrowg(5,:);secondrowg(5,:)]';
sixmat =[firstrowg(6,:);secondrowg(6,:)]*sigmacov6*[firstrowg(6,:);secondrowg(6,:)]';


A = eye(6);
B = [1 1;1 1];
Ctri = kron(A,B);


 Ctri(1:2,1:2)= Ctri(1:2,1:2).*firstmat;
 Ctri(3:4,3:4) = Ctri(3:4,3:4).*secmat;
 Ctri(5:6,5:6)= Ctri(5:6,5:6).*thirdmat;
 Ctri(7:8,7:8) = Ctri(7:8,7:8).*fourthmat;
Ctri(9:10,9:10) = Ctri(9:10,9:10).*fifthmat;
Ctri(11:12,11:12)= Ctri(11:12,11:12).*sixmat;




landmark_cov = Ctri;



% Write your code here...


%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];
k = 6;

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
I = [d*cos(x(3)); d*sin(x(3)); alpha]; % g(ut, u(t-1)
x_pre = x; 
if ( length(arr) == 2)
x_pre(1:3) = x_pre(1:3) + I;        % updating the pose

end

 Bmat = [cos(x(3)) -1*sin(x(3)) 0 ; sin(x(3)) cos(x(3)) 0; 0 0 1];
 Gmat = [1 0 (-d*sin(x(3))); 0 1 (d*cos(x(3))) ; 0 0 1 ];
 
 
 ProcessN = Bmat*control_cov*Bmat';
 TranN = Gmat*P(1:3,1:3)*Gmat';
 P_pre = P;
 P_pre(1:3,(1:3)) =  ProcessN + TranN;   % updading the covariance for pose
 
 % setting the landmark to before
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ==== 

    % Write your code here...
    
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    % Write your code here..

xd = [x_pre(4),x_pre(6),x_pre(8),x_pre(10),x_pre(12),x_pre(14)];
yd = [x_pre(5) ,x_pre(7),x_pre(9),x_pre(11),x_pre(13),x_pre(15)];

xdiff = xd-x_pre(1);
ydiff = yd -x_pre(2);

r_pred = (xdiff.*xdiff + ydiff.*ydiff).^0.5;


 tempb = [arr(1),arr(3),arr(5) , arr(7) , arr(9), arr(11)];
tempr = [arr(2),arr(4),arr(6) , arr(8) , arr(10), arr(12)];

tempbr = tempb + x_pre(3);
costempb = cos(tempbr);
sintempb = sin(tempbr);

temp = x_pre(1) + tempr.*costempb;
temp1 = x_pre(2) +  tempr.*sintempb;

landmark = [temp(1),temp1(1),temp(2),temp1(2),temp(3),temp1(3),temp(4),temp1(4),temp(5),temp1(5),temp(6),temp1(6)] % LXLY
landmark = landmark';




%x_pre(4:15) = landmark; % not sure
Hpmatrix2 = [(x_pre(1)-xd'), (x_pre(2)-yd') ,(temp1'-temp1')]
Hpmatrix1 = [(yd'-x_pre(2)), -xd'+x_pre(1), (temp1'-temp1'-1)]  % LEFT TO DIVIDE





Hpmatrix = [Hpmatrix1(1,:); Hpmatrix2(1,:); Hpmatrix1(2,:);Hpmatrix2(2,:);Hpmatrix1(3,:);Hpmatrix2(3,:);Hpmatrix1(4,:);Hpmatrix2(4,:);Hpmatrix1(5,:);Hpmatrix2(5,:);Hpmatrix1(6,:);Hpmatrix2(6,:)]




Hpmatrix(1,:) = Hpmatrix(1,:)./r_pred(1).^2;
Hpmatrix(3,:) = Hpmatrix(3,:)./r_pred(2).^2;
Hpmatrix(5,:) = Hpmatrix(5,:)./r_pred(3).^2;
Hpmatrix(7,:) = Hpmatrix(7,:)./r_pred(4).^2;
Hpmatrix(9,:) = Hpmatrix(9,:)./r_pred(5).^2;
Hpmatrix(11,:) = Hpmatrix(11,:)./r_pred(6).^2;

Hpmatrix(2,:) = Hpmatrix(2,:)./r_pred(1);
Hpmatrix(4,:) = Hpmatrix(4,:)/r_pred(2);
Hpmatrix(6,:) = Hpmatrix(6,:)/(r_pred(3));
Hpmatrix(8,:) = Hpmatrix(8,:)/(r_pred(4));
Hpmatrix(10,:) = Hpmatrix(10,:)/(r_pred(5));
Hpmatrix(12,:) = Hpmatrix(12,:)/(r_pred(6));

Hpmatrix(1,3) = -1;
Hpmatrix(3,3) = -1;
Hpmatrix(5,3)= -1;
Hpmatrix(7,3)= -1;
Hpmatrix(9,3) = -1;
Hpmatrix(11,3) = -1;



Hlmatrix = -1*Hpmatrix(:,1:2)

A = eye(6);
B = [1 1;1 1];
Ctr = kron(A,B);


 Ctr(:,1:2)= Ctr(:,1:2).*Hlmatrix;
 Ctr(:,3:4) = Ctr(:,3:4).*Hlmatrix;
 Ctr(:,5:6)= Ctr(:,5:6).*Hlmatrix;
 Ctr(:,7:8) = Ctr(:,7:8).*Hlmatrix;
Ctr(:,9:10) = Ctr(:,9:10).*Hlmatrix;
Ctr(:,11:12)= Ctr(:,11:12).*Hlmatrix;
Hmatrix = [Hpmatrix,Ctr];


A = eye(6);
B = [1 1;1 1];
Ctrl = kron(A,B);

 Ctrl(1:2,1:2)= Ctrl(1:2,1:2).*measure_cov;
 Ctrl(3:4,3:4) = Ctrl(3:4,3:4).*measure_cov;
 Ctrl(5:6,5:6)= Ctrl(5:6,5:6).*measure_cov;
 Ctrl(7:8,7:8) = Ctrl(7:8,7:8).*measure_cov;
Ctrl(9:10,9:10) = Ctrl(9:10,9:10).*measure_cov;
Ctrl(11:12,11:12)= Ctrl(11:12,11:12).*measure_cov;


K = (P_pre*Hmatrix')/((Hmatrix*P_pre*Hmatrix')+Ctrl);% ctrl delete


xd = [x_pre(4),x_pre(6),x_pre(8),x_pre(10),x_pre(12),x_pre(14)];
yd = [x_pre(5) ,x_pre(7),x_pre(9),x_pre(11),x_pre(13),x_pre(15)];

xdiff = xd-x_pre(1);
ydiff = yd -x_pre(2);

r_pred = (xdiff.*xdiff + ydiff.*ydiff).^0.5;

b_pred(1)  = atan2(ydiff(1),xdiff(1));
b_pred(2)  = atan2(ydiff(2),xdiff(2));
b_pred(3)  = atan2(ydiff(3),xdiff(3));
b_pred(4)  = atan2(ydiff(4),xdiff(4));
b_pred(5)  = atan2(ydiff(5),xdiff(5));
b_pred(6)  = atan2(ydiff(6),xdiff(6));
b_pred  = wrapTo2Pi(b_pred - x_pre(3)) ;


tempb = [arr(1),arr(3),arr(5) , arr(7) , arr(9), arr(11)];
tempr = [arr(2),arr(4),arr(6) , arr(8) , arr(10), arr(12)];

hu = [b_pred(1),r_pred(1),b_pred(2),r_pred(2),b_pred(3),r_pred(3),b_pred(4),r_pred(4),b_pred(5),r_pred(5),b_pred(6),r_pred(6)];
z = [tempb(1),tempr(1),tempb(2),tempr(2),tempb(3),tempr(3),tempb(4),tempr(4),tempb(5),tempr(5),tempb(6),tempr(6)]

x = x_pre + K*( z'-hu');
P = (eye(15)-K*Hmatrix)*P_pre






    
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
    gt = [ 6 6 6 12 10 6 10 12 14 6 14 12 ]

   

    
    
    
end
i = 1;
Pinv = inv(P);
while i<12  
     error =  ((x(i+3:i+4) - gt(i:i+1)')'*((Pinv((i+3:i+4),(i+3:i+4))))*(x(i+3:i+4) - gt(i:i+1)'))^0.5
    error1 =  ((x(i+3:i+4) - gt(i:i+1)')'*(x(i+3:i+4) - gt(i:i+1)'))^0.5
   listmahalanobis = cat(1,listmahalanobis,error);
     listeucleadian  = cat(1,listeucleadian, error1);
    i = i + 2;
end
print listmahalanobis
%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
    a = [6,6,10,10,14,14];
b = [6,12,6,12,6,12];
plot(a, b, '*k');


%==== Close data file ====
fclose(fid);
