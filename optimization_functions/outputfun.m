function [state,options,optchanged] = outputfun(options,state,flag)%,seed)
    persistent data;
    if flag == 'init'        
        data = [state];
        data(1:options.MaxGenerations) = state;
    elseif flag == 'iter'
        data(state.Generation) = state;
    elseif flag == 'done'
        data(state.Generation) = state;
        data = data(1:state.Generation);
%         save(['data' num2str(seed) '.mat'], 'data');
        save('data.mat', 'data');
        clear data;
    end
    
    optchanged = false;
    
end