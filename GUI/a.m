%{
x = [-2:0.1:0];
%y = (2*x*plusfun(x-2))+(((-1)/2*x+5)*plusfun(2-x));
%y = plusfun(x-2)*x
plusfun = @(x) max(x,0);
y = -2*(plusfun(2-x)) + (-0.5)*(plusfun(x-2)) -4;


plot(x,y)
%}


plusfun = @(x) max(x,0);

x = [-2:0.2:-1];
y = x +2 + randn(size(x))/20;
x1 = [0:0.1:2];
y1 = -1*x1 +2 +randn(size(x1))/20;
x = cat(2,x,x1);
y = cat(2,y,y1);
x1 = [1.01 1.02 1.03 1.04 1.05]
y1 = [0.25 0.25 0.25 0.25 0.25]
x = cat(2, x, x1)
y = cat(2, y, y1)

plot(x, y, 'o');

%P0 = [-5.0 -5.0 -5.0];
P0 = [0.1 0 0.0]
%P0 = [2 0 5 0 0];
lb = [-1 -10 -10];
ub = [1 10 10];

model = @(P,x) (-P(1))*plusfun(P(2)-x) + ((-1)/P(1))*plusfun(x-P(2)) + P(3);
% Pfit = lsqcurvefit(model,P0,x,y,lb,ub)
Pfit = lsqcurvefit(model,P0,x,y, lb, ub)
%x1 = sort(x)
%x1 = [0:0.1:4];
%y2 = model(Pfit, x1);
%y2 = -2*plusfun(2-x) + ((-1)/2)*plusfun(x-2)+4;
%y2 = -2*(plusfun(2-x1)) + (-0.5)*(plusfun(x1-2)) -4;
%Pfit = [1 0 2];
modelpred = model(Pfit,sort(x));
%plot(x,y,'o',sort(x),modelpred,'r-')

opts = statset('nlinfit');
opts.RobustWgtFun = 'bisquare';
beta = nlinfit(x,y,model,P0,opts)
new_beta = model(beta, sort(x));
plot(x, y, 'o', sort(x), new_beta,'r-');



%{

x = 3*(rand(100,1) - .5);
y = sin(x) + randn(size(x))/20;
plot(x,y,'o')

%plusfun = @(x) mymax(x, 0);

s2 = 1;

%plusfun = @(x) max(x,0);
model1 = @(P,x) P(1) + s2*x + (s2-P(2))*plusfun(P(4)-x) + (P(3)-s2)*plusfun(x-(P(4)+P(5)));
%model = mypoy(P, x);
Pfit = lsqcurvefit(model1,P0,x,y,lb,ub)
modelpred = model(Pfit,sort(x));
plot(x,y,'o',sort(x),modelpred,'r-')

function ccc = model(P, x)

s2 = 1;

ccc = P(1) + s2*x + (s2-P(2))*plusfun(P(4)-x) + (P(3)-s2)*plusfun(x-(P(4)+P(5)));
end

function bbb = plusfun(x)
bbb = max(x, 0);
end

function aaa = mymax(a,b)
if (a > b)
    aaa = a;
else
    aaa = b;
end
end

%}