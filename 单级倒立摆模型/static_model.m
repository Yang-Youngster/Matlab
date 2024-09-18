clear all;
mp = 0.4;
mw = 0.1;
%质量
l = 0.12;
%长度
g = 9.8;
%重力加速度
J = mp*l*l;
%转动惯量
a0 = 10 / 180 * pi;
da0 = 0 / 180 * pi;
x0 = 0;
dx0 = 0;


%初始状态
T = 0.01;
%仿真周期
n = 1000;
%仿真步数
a = zeros(n,1);
da = zeros(n,1);
dda = zeros(n,1);
x = zeros(n,1);
dx = zeros(n,1);
time = zeros(n,1);
coef = zeros(n,1);
a(1) = a0;
da(1) = da0;
k = 0;
%%展示车轮移动的位移大小

%%车轮位移加速度
ddxw=zeros(n,1);

dis=zeros(n,1);


for i = 1:1:n-1
    coef(i) = (J - (mp*mp * l*l * cos(a(i))*cos(a(i))) / (mp+mw) );
    
    dda(i+1) = (mp*g*l*sin(a(i)) - ((mp*mp * l*l *sin(a(i))*cos(a(i)))/(mp+mw))  * da(i)*da(i)) / coef(i);

    %角加速度
    da(i+1) = da(i) + dda(i)*T;
    %角速度
    a(i+1) = a(i) + da(i)*T;
    
    time(i+1) = i * T;
    ddxw(i+1)=mp*l*(dda(i)*cos(a(i))-da(i)*da(i)*sin(a(i)));
 
    if a(i+1) >= pi / 2
        k = i;
        break
    end
end


plot(time(1:k),a(1:k)/pi*180,'r');
%title("show polt balance status");

hold on

plot(time(1:k),ddxw(1:k),'green--');

legend('polt-a',"wheel-a");
xlabel('time/s');
ylabel('°/s')

%title("show wheel balance status");




