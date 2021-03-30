theta1 = 0;
theta2 = 0;
theta3 = 0;
%assume wrist facing vertically down
minang = 15;
maxang = 75;
step = 5;
val = (maxang-minang)/step;
x = zeros(val*val*val);
y = zeros(val*val*val);
z = zeros(val*val*val);
index = 1;
d4 = 0;
d3 = 0;
d2 = 0;
d1 = 0.05;
a2 = 0.6;
a3 = 0.5;
%i is the swivel of shoulder and can be either positive or negative
%j must positive so that the shoulder goes
% for i = minang:step:maxang
%     for j = minang:step:maxang
%         for k = minang:step:maxang
%             x(index) = d2*sind(i) - a3*(cosd(i)*sind(j)*sind(-k) - cosd(i)*cosd(j)*cosd(-k)) + d3*sind(i) + d4*sind(i) + a2*cosd(i)*cosd(j);
%             y(index) = a2*cosd(j)*sind(i) - d2*cosd(i) - d3*cosd(i) - d4*cosd(i) - a3*(sind(i)*sind(j)*sind(-k)-cosd(j)*cosd(-k)*sind(i));
%             z(index) = d1 + a3*sind(j-k) + a2*sind(-j);
%             index = index + 1;
%         end
%     end
% end



for j = minang:step:maxang
    for i = minang:step:maxang
        for k = minang:step:maxang
            x(index) = d2*sind(i) - a3*(cosd(i)*sind(j)*sind(-k) - cosd(i)*cosd(j)*cosd(-k)) + d3*sind(i) + d4*sind(i) + a2*cosd(i)*cosd(j);
            y(index) = a2*cosd(j)*sind(i) - d2*cosd(i) - d3*cosd(i) - d4*cosd(i) - a3*(sind(i)*sind(j)*sind(-k)-cosd(j)*cosd(-k)*sind(i));
            z(index) = d1 + a3*sind(j-k) + a2*sind(-j);
            
            index = index + 1;
        end
    end
end

figure
% scatter(x,z)
plot(x,z,'o')
title('X vs Z (measuring reachable height)')

figure
% scatter(x,y)
plot(x,y,'o')
title('X vs Y(measuring top down 2D reachable area)')