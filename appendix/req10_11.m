% Requirement 10 & 11:
% QR CODE (0, 1000):

px1 = [-22, -22, -20, -16, -13, -23, -14, -18, -15, -16, -14, -17, -15, -18, -21, -18, -13, -15, -19, -14, -19, -16, -19, -17, -9, -21, -10, -8, -23, -16];
py1 = [-41, -51, -53, -51, -27, -52, -48, -56, -54, -45, -45, -26, -47, -45, -41, -56, -46, -49, -53, -55, -56, -48, -55, -47, -51, -46, -43, -46, -51, -43];

figure(1)
scatter(px1, py1, 'filled', 'b')
axis equal
xlim([-30 40])
ylim([-60 20])
hold on

title('Accuracy and repeatability test - 1 m')
xlabel('distance (mm)')
ylabel('distance (mm)')
plot(0, 0, 'r*', 'MarkerSize', 9)
legend('Chalk mark center', 'QR code center')

x1_std = std(px1)
y1_std = std(py1)

x1_mean = mean(px1)
y1_mean = mean(py1)

accuracy = sqrt(x1_mean*x1_mean + y1_mean*y1_mean)

dist1 = []

for i = 1:30
    dist1 = [dist1 sqrt(px1(i)*px1(i)+py1(i)*py1(i))];
end

p1_std = std(dist1);
p1_mean = mean(dist1);

% QR CODE (0, 1500):
px2 = [-5 -18 -8 -4 -8 8 -6 -7 -8 -17 6 -13 10 22 25 8 -7 22 32 9 2 9 8 6 7 15 3 9 9 9];
py2 = [-32 -32 -31 -50 -34 -35 -43 -40 -38 -36 -40 -35 -44 -42 -42 -41 -36 -40 -51 -52 -53 -48 -35 -42 -27 -27 -32 -42 -41 -39];

figure(2)
scatter(px2, py2, 'filled', 'b')
axis equal
xlim([-30 40])
ylim([-60 20])
hold on

title('Accuracy and repeatability test - 1.5 m')
xlabel('distance (mm)')
ylabel('distance (mm)')
plot(0, 0, 'r*', 'MarkerSize', 9)
legend('Chalk mark center', 'QR code center')

x2_std = std(px2);
y2_std = std(py2);

x2_mean = mean(px2);
y2_mean = mean(py2);

accuracy_1 = sqrt(x2_mean*x2_mean + y2_mean*y2_mean);

dist2 = [];

for i = 1:30
    dist2 = [dist2 sqrt(px2(i)*px2(i)+py2(i)*py2(i))];
end
p2_std = std(dist2);
p2_mean = mean(dist2);

%% Discussion - Throw with random position:

rx = [-5 -35 7 8 7 7 36 27 -38 -4 -15 -11 -35 -7 20 -7 -34 11 -45 11]
ry = [-89 -25 -20 -112 7 -29 -95 -128 24 6 -105 -36 -78 -10 -129 -68 -80 -59 -152 4]

figure(3)
hold on
scatter(px1, py1, 'filled', 'b')
scatter(rx, ry, 'filled', 'r')
axis equal
xlim([-100 100])
ylim([-180 50])
axis equal

title('Fixture vs. random position - 1 meter')
xlabel('distance (mm)')
ylabel('distance (mm)')
plot(0, 0, 'r*', 'MarkerSize', 14)
legend('Chalk mark center - fixture', 'Chalk mark center - random position', 'QR code center', 'Location', 'southeast')
