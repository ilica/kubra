function [ X, Y ] = letter_maker()

    plot([], [], 'o');
    axis([-3 3 -3 3]);
    set(gca,'XLimMode','manual','YLimMode','manual');  % Fix axes limits
    X = [];
    Y = [];
    x = 0
    while x < 3 
        [x y] = ginput(1);
        X(end+1) = x;
        Y(end+1)= y;
        plot(X, Y, 'o');
        axis([-3 3 -3 3]);

        X
        Y
    end
    


end

