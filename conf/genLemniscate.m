function points=genLemniscate

% number of points
N=100;

% compute the length L of quarter of the curve
syms t;

x=sqrt(cos(2*t))*cos(t);
y=sqrt(cos(2*t))*sin(t);

Dx=diff(x);
Dy=diff(y);

f=sqrt(Dx^2+Dy^2);
L=quad(@(theta)(subs(f,t,theta)),-pi/4,0);
fprintf('length of quarter of the curve L=%g\n',L);

% go over the first quarter seeking for theta(:)
fun1=@(theta)(quad(@(tau)(subs(f,t,tau)),-pi/4,theta));
theta=-pi/4*ones(N/4,1);
ds=4*L/N;
s=ds;
for i=2:length(theta)
    fprintf('computing theta for s=%g*L ... ',s/L);
    fun2=@(theta)(fun1(theta)-s);
    theta(i)=fzero(fun2,theta(i-1));    
    fprintf('theta(%d)=%g\n',i,theta(i));
    s=s+ds;
    
    if s>L
        break;
    end
end

fprintf('computing points of the first quarter ...\n');
x1=subs(x,t,theta);
y1=subs(y,t,theta);

fprintf('second quarter: mirroring ...\n');
x2=flipud(x1);
y2=flipud(-y1);

fprintf('merging results ...\n');
x1=[x1; x2];
y1=[y1; y2];

fprintf('second half: mirroring ...\n');
x2=-x1;
y2=flipud(-y1);

fprintf('merging results ...\n');
x=[x1; x2];
y=[y1; y2];

fprintf('plotting results...\n');
plot(x,y,'o-');

points=[x y];

end

