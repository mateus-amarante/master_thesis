function state = plot_opt_vars(options,state,flag)

persistent x;

[~,best_idx] = min(state.Score); 
best = state.Population(best_idx,:);

if flag == 'init'        
    x = best;
    h = plot(0,x);
%     set(h,'Tag','bestplot');
    set(gca,'xlim',[0,options.MaxGenerations]);
    legend;
elseif flag == 'iter'
    x = [x; best];
%     h = findobj(get(gca,'Children'),'Tag','bestplot'); 
    plot((0:state.Generation)',x); 
end

end

