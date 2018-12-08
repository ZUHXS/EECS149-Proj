    x = double(positionAll(1,:));
    y = double(positionAll(2,:));
    disp(x);
    disp(y);
    total_array = zeros(1200,600,100);
    
    for i=1:length(x)
        total_array(int32(x(i)*100)+600,int32(y(i)*100),1) = total_array(int32(x(i)*100)+600,int32(y(i)*100),1) + 1;
        current_index = total_array(int32(x(i)*100)+600,int32(y(i)*100),1);
        if (current_index >= 98)
            fprintf("out of range!!!");
        end
        total_array(int32(x(i)*100)+600,int32(y(i)*100),current_index+1) = i;
    end
    
    total_index = zeros(1200*600);
    max_index = 1;
    for i=2:599
        for j=2:1199
            if (total_array(i, j, 1) ~= 0) % if current index has dots
                if (total_array(i,j-1,100)~=0)
                     current_group_number = total_array(i,j-1,100);
                    if (total_array(i-1,j+1,100)~=0)
                        total_index(total_array(i-1,j+1,100))=0;
                        total_array(i-1,j+1,100) = current_group_number;
                        total_array(i,j,100) = current_group_number;
                    else
                        total_array(i,j,100) = current_group_number;
                    end
                elseif (total_array(i-1,j-1,100)~=0)
                    current_group_number = total_array(i-1,j-1,100);
                    if (total_array(i-1,j+1,100)~=0)
                        total_index(total_array(i-1,j+1,100))=0;
                        total_array(i-1,j+1,100) = current_group_number;
                        total_array(i,j,100) = current_group_number;
                    else
                        total_array(i,j,100) = current_group_number;
                    end
                elseif (total_array(i-1,j,100)~=0)
                    current_group_number = total_array(i-1,j,100);
                    total_array(i,j,100) = current_group_number;
                elseif (total_array(i-1,j+1,100)~=0)
                    current_group_number = total_array(i-1,j+1,100);
                    total_array(i,j,100) = current_group_number;
                else % arround no existing points
                    current_group_number = max_index;
                    max_index = max_index + 1;
                    total_array(i,j,100) = current_group_number;
                end
            end
        end
    end
    disp(max_index);