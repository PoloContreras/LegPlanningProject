clc;
clear all;
close all;

%%
%carga de datos

run('obstacles0');

%%
%optimizacion

cvx_begin quiet
    
    variable p(N,d,2) %number of steps, number of dimensions, number of trajectories
    
    expression gasolina
    
    gasolina = 0;
    for k=1:2 %two trajectories
        for i=2:N-1 %steps
           gasolina = gasolina + pow_pos(norm(p(i-1,:,k)-2*p(i,:,k)+p(i+1,:,k),2),2);
        end
    end
    
    minimize(gasolina)
    
    subject to
    
    for k=1:2
        
        p(1,:,k) == pInitial
        p(N,:,k) == pFinal
       
        %collision avoidance for multiple obstacles, two trajectories (we assume
        %the goal is in the upper right quadrant in terms of the obstacle, and initial position is
        %not)
        for o=1:obstacles %for each obstacle
            if (pInitial(1) <= pObs(o,1)+rObs(o)) && (pInitial(2) <= pObs(o,2)+rObs(o))
                if k==1
                    p(N/2,1,k) <= pObs(o,1)-(rObs(o)) %passing on the left
                    p(N/2,2,k) >= pObs(o,2)+(rObs(o)) %passing above
                elseif k==2
                    p(N/2,1,k) >= pObs(o,1)+(rObs(o)) %passing on the right
                    p(N/2,2,k) <= pObs(o,2)-(rObs(o)) %passing below
                end
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
viscircles(pObs,rObs);
