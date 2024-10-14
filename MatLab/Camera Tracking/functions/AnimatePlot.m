function [] = AnimatePlot(x, y, plot_title, xname, yname, xlims, ylims, lgnd, file_video)

[n_row,n_col] = size(y);

open(file_video)
figure()
for i=1:1:length(y)
    plot(x(1:i), y(1:i,:), 'LineWidth', 2)
    title(plot_title)
    grid on
    xlim(xlims);
    ylim(ylims);
    xlabel(xname);
    ylabel(yname);
    
    if n_col ~= 1
            legend(lgnd);
    end
    pause(0.0001) %Pause and grab frame
    frame = getframe(gcf); %get frame
    writeVideo(file_video, frame);
end
close(file_video)

end