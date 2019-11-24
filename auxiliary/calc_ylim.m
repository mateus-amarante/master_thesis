function ylim_values = calc_ylim(values, relative_offset)

ylim_max = max(values(:));
ylim_min = min(values(:));
ylim_offset = (ylim_max - ylim_min)*relative_offset;
ylim_values = [ylim_min, ylim_max] + ylim_offset*[-1,1];

end

