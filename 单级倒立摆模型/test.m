x = linspace(-2,2,20);
y = x'; %转置
z = x .* exp(-x.^2 - y.^2);
surf(x,y,z)