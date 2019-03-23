clear all
close all
numSamples = 400;
samples = zeros(numSamples,3);
for i = 1:numSamples
    x = rand(1,2);
    val_g = gauss2d(x);
    samples(i,:) = [x val_g];
end
reso = 100;
surface1 = zeros(reso,reso);
for i = 1:numSamples
    x = ceil(samples(i,1)*reso);
    y = ceil(samples(i,2)*reso); 
    surface1(x,y) = samples(i,3);
end
mesh(surface1);
heg = HEDGER(samples(1:100,:),0.5,0.1,7);
for j = 1:100
    surface2 = zeros(reso,reso);
    predicts = zeros(numSamples,3);
    for i = 1:numSamples
        q = rand(1,2);
        pred = heg.predict(q(1),q(2),1);
        predicts(i,:) = [q pred];
        x = ceil(q(1)*reso);
        y = ceil(q(2)*reso);
        surface2(x,y) = pred;
    end
    figure(2);
    mesh(surface2);
    t = samples(100+j,:);
    heg = heg.training(t(1),t(2),t(3),t(1)+rand*0.01,0.8,0.8,0.1);
    disp("*");
end