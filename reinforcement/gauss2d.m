function val = gauss2d(x)
%SIN2D 
    % sample the value at [x,y] of a 2-d gaussian function
    sigma = 0.1;
    mu = [0.5 0.5];
    val = (1/(sigma*sqrt(2*pi)))*exp(-((dot(x-mu,x-mu))/sigma)^2/2);
end

