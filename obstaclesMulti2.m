% parameters
N = 120; %total number of steps
h = 0.05; %scaling factor for discretization

d = 2; %number of dimensions in our space

pInitial = [0, 0]; %position X1,X2 of robot at start
pFinal = [30,8]; %position of goal on ground

pObs = [5, 2; 12, 6; 12,1; 20,3; 26,6; 26,1]; %position X1,X2 of each obstacle (six total)

rObs = [2.06; 1.78; 1.78; 3.81; 1.66; 1.56]; %radius of each obstacle hitbox

obstacles = length(rObs); %number of obstacles

%%% FEEL FREE TO COPY THE FOLLOWING PLOTTING CODE ### 
% fig = figure(1); 
% x = (1 : N) * h;
% xlim([x(1), x(N)]);
% plot([S * h, S * h], [-1, 1], 'k--'); hold on;
% plot(x, P(:, 1), 'r', 'Linewidth', 2);
% plot(x, P(:, 2), 'b', 'LineWidth', 2);
% plot(x, P(:, 3), 'g', 'LineWidth', 2);
% plot([O * h, O * h], [0.5, 1], 'r--');
% plot([O * h, O * h], [-1, -0.5], 'b--');
% legend('obstacle revealed', 'obstacle left', ...
%     'obstacle right', 'obstacle clear', ...
%     'left safe region', 'right safe region');
% hold off;
% print(fig, '-depsc', 'path_plan_contingencies.eps');