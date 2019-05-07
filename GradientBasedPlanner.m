function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
route = start_coords;
iter = 1;
err = Inf;
min_err = 2;
while err > min_err
    if iter >= max_its
        return
    else
        next_pos = round(route(iter,:));
        gradient_temp = [gx(next_pos(2),next_pos(1)) gy(next_pos(2),next_pos(1))]; %switch x and y indices
        gradient_temp = gradient_temp/norm(gradient_temp);
        next_pos = [route(iter,1) + gradient_temp(1) route(iter,2) + gradient_temp(2)];
        if  next_pos(1) < 1
            next_pos(1) = 1;
        elseif next_pos(1) > size(gx,1)
            next_pos(1)  = size(gx,1);
        end
        if  next_pos(2) < 1
            next_pos(2) = 1;
        elseif next_pos(1) > size(gx,2)
            next_pos(2) = size(gx,2);
        end
        route = [route ; next_pos];
        err = norm([(route(iter,1) - end_coords(1)) (route(iter,2) - end_coords(2))]);
        iter = iter + 1;
    end
end
route = double(route);

% *******************************************************************
end
