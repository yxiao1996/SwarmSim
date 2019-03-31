function noisy = AdditiveGaussian(data,Sigma)
%ADDITIVEGAUSSIAN 
    % Add gaussian noise to input data according to Covariance
    % matrix(Sigma)
    R = chol(Sigma);
    noise = randn(size(data))*R;
    noisy = data + noise;
end

