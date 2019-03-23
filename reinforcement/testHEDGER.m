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
%mesh(surface1);
heg = HEDGER(samples,0.5,0.1,10);

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
%figure(2);
%mesh(surface2);

%quad3dcovmat(1,1,1,samples(1,:),samples(2,:),samples(3,:))
sf = fit(samples(:,1:2),samples(:,3),'poly22');
plot(sf,samples(:,1:2),samples(:,3));
figure(2);
sf = fit(predicts(:,1:2),predicts(:,3),'poly22');
plot(sf,predicts(:,1:2),predicts(:,3));
c = coeffvalues(sf);
p_x = [c(4) c(2)+c(5) c(1)+c(6)+c(3)];
p_y = [c(6) c(3)+c(5) c(1)+c(2)+c(4)];
cd_x = polyder(p_x);
cd_y = polyder(p_y);
roots(cd_x)
roots(cd_y)
% test greedy sample stategy
steps = 20;
q = [0.2 0.2];
for i = 1:steps
    q = heg.greedy(q(1),q(2),0.1,0.1,0.1);
    val = gauss2d(q);
    disp(val);
end