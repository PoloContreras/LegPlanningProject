clc;
clear all;
close all;

%%
%carga de datos

run('obstacles0');

%%

cvx_begin quiet
    
    variable p(N,d,2) %number of steps, number of dimensions, number of trajectories
    
    expression gasolina
    
    gasolina = 0;
    for k=1:2
        for i=2:N-1
           gasolina = gasolina + pow_pos(norm(p(i-1,:,k)-2*p(i,:,k)+p(i+1,:,k),2),2);
        end
    end
    
    minimize(gasolina)
    
    subject to
    
    for k=1:2
        
        p(1,:,k) == pInitial
        p(N,:,k) == pFinal
       
        %collision avoidance for one obstacle, two trajectories (we assume
        %the goal is in the upper right quadrant in terms of the obstacle, and initial position is
        %not)
        if (pInitial(1) <= pObs(1,1)+rObs) && (pInitial(2) <= pObs(1,2)+rObs)
            if k==1
                p(N/2,1,k) <= pObs(1,1)-(rObs)
                p(N/2,2,k) >= pObs(1,2)+(rObs)
            elseif k==2
                p(N/2,1,k) >= pObs(1,1)+(rObs)
                p(N/2,2,k) <= pObs(1,2)-(rObs)
            end
        end
    end
    
cvx_end

%%

dist = (1:N)*h;

plot(p(:,1,1),p(:,2,1));
hold on
plot(p(:,1,2),p(:,2,2));
%figure;
