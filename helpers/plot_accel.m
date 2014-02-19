function [z_pos x_pos z_raw x_raw] = plot_accel
    Y = [
-0.138428 0.066406 -2.664062 -1.212000 -43.548000 -11.792000
-0.145996 0.115967 -1.875732 -6.132000 -40.383998 -3.628000
-0.200439 0.071045 -1.657227 -10.516000 -65.255996 0.344000
-0.236084 -0.144531 -2.905029 -6.848000 -53.111999 -2.472000
-0.287354 0.223633 -2.019775 -7.140000 -40.631999 6.460000
-0.334229 -0.026367 -1.470947 -7.552000 -61.155998 14.860000
-0.449707 0.085449 -2.192627 -4.764000 -88.195999 13.940000
-0.467041 0.131104 -2.321533 -12.472001 -40.824001 15.496001
-0.474609 0.174072 -1.509033 -11.692001 -52.819999 22.704000
-0.548828 0.270020 -1.410645 -14.536001 -82.779998 26.983999
-0.562256 0.115723 -2.879150 -15.092000 -55.548000 33.183998
-0.613525 0.092529 -2.076660 -11.272001 -33.888000 41.183998
-0.596191 0.147705 -1.677002 -6.644000 -54.411998 42.212001
-0.669922 0.234375 -2.208740 -6.624000 -63.035999 50.979999
-0.702881 0.270996 -2.178711 -7.168000 -29.176000 52.819999
-0.679199 0.354004 -1.736572 -1.824000 -27.096000 66.024002
-0.750732 0.255127 -1.854736 1.040000 -26.343999 73.139999
-0.771728 0.047119 -1.989746 6.532000 -8.276000 60.375999
-0.762451 0.244873 -2.053955 13.108000 3.400000 53.411998
-0.747314 0.028564 -1.945068 20.799999 11.444001 35.400001
-0.722412 0.271729 -2.137939 23.815999 29.472000 18.011999
-0.630859 0.370117 -1.974365 25.243999 31.187999 17.347999
-0.581543 0.289307 -1.881592 16.503999 41.276000 17.035999
-0.528809 0.148438 -1.835693 12.780000 39.291999 8.704001
-0.505859 0.087402 -2.239990 20.704000 39.931999 -2.244000
-0.530029 0.449463 -2.505859 34.231998 72.251998 -13.828001
-0.556396 0.441650 -2.065430 27.555999 97.148002 1.268000
-0.512451 0.078613 -1.343750 24.184000 82.435997 -10.440000
-0.376953 0.121582 -2.427734 31.252000 73.627998 -22.555999
-0.375244 0.346924 -2.574951 30.892000 90.587997 -31.604000
-0.288086 0.243164 -1.690430 13.724000 80.487998 -1.724000
-0.160645 -0.013184 -2.122559 17.691999 73.975997 -15.984001
-0.100830 0.135010 -1.844971 17.511999 68.603996 -10.704001
-0.027588 0.428223 -1.813232 14.380001 59.416000 -15.512001
0.003906 0.137695 -1.812988 4.584000 48.571998 -1.848000
0.051270 0.125000 -2.424316 10.604001 53.580001 -9.168001
0.048584 0.087158 -2.201904 9.476000 75.839996 -9.496001
0.155029 0.247070 -1.384521 6.980000 53.124000 -3.940000
0.244141 0.110352 -2.306885 12.772001 42.655998 -4.680000
0.278564 0.033447 -2.048096 17.540000 50.659999 -7.912000
0.282715 0.049072 -2.173096 13.516000 58.596000 -7.696000
0.356689 0.147705 -1.641602 10.752000 34.611999 -1.228000
0.361084 -0.024902 -1.994629 15.276000 20.972000 -1.336000
0.395508 0.047363 -2.090332 10.956001 33.616001 -3.348000
0.421387 0.075928 -1.619141 9.220001 21.431999 -3.080000
0.462647 0.029297 -1.637207 6.168000 -1.360000 -2.796000
0.433838 0.028564 -2.343506 10.028000 -6.036000 -2.440000
0.414551 -0.029785 -2.122314 4.548000 7.436000 -2.744000
];

    dt = 0.025;
    finger_length = 10;

  
    
    function [x_int y_int] = position_from_accel(y_accel)
        data_length = length(y_accel);
        x = 1:data_length;
        x_int = 1:0.25:data_length;

        accel_spline = spline(x,y_accel);
        vel = fnval(fnint(accel_spline), x);

        vel_spline = spline(x, vel);
        y_int = fnval(fnint(vel_spline), x_int); 
    end

    function [x_int y_int] = linear_position_from_accel(y_accel)
        data_length = length(y_accel);
        x_int = 1:data_length;
        y_int = cumsum(cumsum(y_accel));
    end

    function [y] = gyro_int(gyro)
        y = cumsum(gyro * dt)
    end

    function pos_at_time = position_from_gyro(gyro)
        radians_at_time = gyro_int(gyro) * (pi / 180);
        pos_at_time = sin(radians_at_time) * finger_length;
    end
    x_raw = gyro_int(Y(:, 4));
    z_raw = gyro_int(Y(:, 6));
    
    x_pos = position_from_gyro(Y(:, 4))
    %y_pos = position_from_gyro(Y(:, 5));
    z_pos = position_from_gyro(Y(:, 6))

      
        set(gcf, 'color', [0 0 0]);
        set(gca, 'color', [0 0 0]);
        
        plot([-1]);
         set(gcf, 'color', [0 0 0]);
        set(gca, 'color', [0 0 0]);
        
        pause(10);
    
        
    for i = 1:length(z_pos)
        plotted = plot(x_pos(1:i), -z_pos(1:i), 'o');
        axis([-3 3 -3 3]);
        
        set(gcf, 'color', [0 0 0]);
        set(gca, 'color', [0 0 0]);

        set(plotted, ...
            'Marker'          , 'o', ...
            'MarkerSize'      , 10, ...
            'MarkerEdgeColor' , 'none', ...
            'MarkerFaceColor' , [134/255 205/255 77/255]);
        
        pause(0.025);

        
    end
    
    
    
    %plot(y_pos, z_pos, 'o');
end

