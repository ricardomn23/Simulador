function check_distance()
	final_point = [10 15 20];
    
    data = load("BUV1_Sim.log");
    
    cur_point = [data(1, 2) data(1, 3) data(1, 4)];
    min_distance = distance(final_point, cur_point);
    min_index = 1;
    
    for i = 2:size(data,1)        
        cur_point = [data(i, 2) data(i, 3) data(i, 4)];
        d = distance(final_point, cur_point);
        
        if d < min_distance
            min_distance = d;
            min_index = i;
        end
        
    end
    
    fprintf('data size: %d\n', size(data,1));
    fprintf('min_index: %d\n', min_index);
    fprintf('min_distance: %f\n', min_distance);
    fprintf('point with min_distance: (%f, %f, %f)\n', data(min_index, 2), data(min_index, 3), data(min_index, 4));
    
end
 
function dist = distance(p1, p2)
    dist = sqrt((p1(1) - p2(1))^2 + (p1(2) - p2(2))^2 + (p1(3) - p2(3))^2);
end
