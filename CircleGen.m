function CircleGen(Cen_x, Cen_y, R)
    idx = 1;
    for theta = 0.3:(2*pi-0.1)/1000:2*pi
        x(idx) = R*cos(theta);
        y(idx) = R*sin(theta);
        idx = idx+1;
    end
    x = x + Cen_x;
    y = y + Cen_y;

    plot(x, y);
    axis equal;

    path = [x', y'];
    save path_Circle.mat path

end