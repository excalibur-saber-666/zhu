% LINECOLORS 

N = 13;
C = rand(N,3);
X =linspace(0,pi*3,1000); 
Y =bsxfun(@(x,n)sin(x+2*n*pi/N), X.',1:N); 

for i = 1:N
    plot(X',Y(:,i),'color',C(i,:));
    hold on
end