clc;
clear all;
close all;

%%
%carga de datos

run('obstacles0');

%%
%generacion de hiperparametros
N = 30; %number of steps in path

n = 1; %number of generated trajectories

epsilon = 1e-7; %control of overarching while loop

pnt = ones(N,d,n);%generation of original values

v = -1:0.01:1;  % plotting range
[x y] = meshgrid(v);  % get 2-D mesh for x and y
cond1 = x+y+x.^2 < 3;  % check conditions for these values
cond2 = y+x+y.^2 < 3;
cond1 = double(cond1);  % convert to double for plotting
cond2 = double(cond2);
cond1(cond1 == 0) = NaN;  % set the 0s to NaN so they are not plotted
cond2(cond2 == 0) = NaN;
cond = cond1.*cond2;  % multiply the two condaces to keep only the common points
surf(x,y,cond)
view(0,90)    % change to top view

for k=1:n
    for i=2:N-1
        pnt(i,:,n) = pInitial+((pFinal-pInitial)/N)*(i-1); %interpolation to start off with a close solution
    end
    pnt(1,:,n) = pInitial;
    pnt(N,:,n) = pFinal;
end

iter=1;

%pnt(2:N-1,:,:) = randn(N-2,d,n);

diff = inf; %initialize to unreasonable value
%%
%optimizacion

%while abs(diff) > epsilon
for it=1:iter
    
    prevGas = 0; %objective function evaluation at current iteration
    for k=1:n
        for i=1:(N-1)
            %prevGas = prevGas + norm(pnt(i-1,:,k)-2*pnt(i,:,k)+pnt(i+1,:,k),2)^2;
            prevGas = prevGas + norm(pnt(i,:,k)-pnt(i+1,:,k),2)^2;
        end
    end
    
    prevGas %print for monitoring

    cvx_begin quiet

        variable p(N,d,n) %number of steps, number of dimensions, number of trajectories

        expression gasolina

        gasolina = 0;
        for k=1:n %two trajectories
            for i=1:N-1 %steps
               %gasolina = gasolina + pow_pos(norm(p(i-1,:,k)-2*p(i,:,k)+p(i+1,:,k),2),2);
               gasolina = gasolina + pow_pos(norm(p(i,:,k)-p(i+1,:,k),2),2);
            end
        end

        minimize(gasolina)

        subject to

        for k=1:n
            norm(p(1,:,k)-pInitial,2) <= 1;
            norm(p(N,:,k)-pFinal,2) <= 1;
        end
            
            %affine approximations of nonconvex constraints for collision
            %avoidance
            
            for k=1:n %for each generated trajectory
                for o=1:obstacles %for each obstacle
                    for i=2:N-1 %for each step in the trajectory
                        if pnt(i,:,k)==pObs(o,:) %collision even with radius zero
                            rObs(o) - norm(pnt(i,:,k)-pObs(o,:)) <= 0;
                        else%if norm(pnt(i,:,k)-pObs(o,:)) <= 2*rObs(o) %the two positions are not identical
                            rObs(o) - norm(pnt(i,:,k)-pObs(o,:)) - ((pnt(i,:,k)-pObs(o,:))/(norm(pnt(i,:,k)-pObs(o,:))))'*(p(i,:,k)-pnt(i,:,k)) <= 0;
                        end
                    end
                end
            end

    cvx_end

    if cvx_status ~= 'Infeasible'
        pnt = p;
        diff = prevGas - cvx_optval;
    end 
end

%%

dist = (1:N)*h;

plot(p(:,1,1),p(:,2,1));
hold on
%figure;
viscircles(pObs,rObs);