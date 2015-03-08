idx1 = [];
idx2 = [];
idx3 = [];
idx4 = [];
idx5 = [];

for i = 1:size(rgbdclabel,2)
    if rgbdclabel(i) == 1
        idx1 = [idx1 i];
    end
    if rgbdclabel(i) == 2
        idx2 = [idx2 i];
    end
    if rgbdclabel(i) == 3
        idx3 = [idx3 i];
    end
    if rgbdclabel(i) == 4
         idx4 = [idx4 i];
    end
    if rgbdclabel(i) == 5
         idx5 = [idx5 i];
    end    
end

% sum = 0;
% count = 0;
% for i = 1:size(idx1,2)-1
%    for j = i+1: size(idx1,2)
%        %rgbdfea(:,idx1(i))' * rgbdfea(:, idx1(j))
%        %sum = sum + rgbdfea(:,idx2(i))' * rgbdfea(:, idx2(j))/4.0;
%        sum = sum + exp(- 0.5*norm(rgbdfea(:,idx1(i))/2.0 - rgbdfea(:, idx1(j))/2.0));
%        count = count + 1;
%    end
% end
% sum/count

sum = 0;
count = 0;
for i = 1:400
   for j = 1: 400
       %rgbdfea(:,idx1(i))' * rgbdfea(:, idx1(j))
       %sum = sum + rgbdfea(:,idx1(i))' * rgbdfea(:, idx2(j))/4.0;

       var = histogram_intersection(rgbdfea(:,idx2(i)), rgbdfea(:, idx3(j)));
       sum = sum + var;
        
       count = count + 1;
   end
end
sum/count

sum = 0;
count = 0;
for i = 1:200
   for j = 1: 200
       %rgbdfea(:,idx1(i))' * rgbdfea(:, idx1(j))
       %sum = sum + rgbdfea(:,idx1(i))' * rgbdfea(:, idx2(j))/4.0;
       var = exp(- 0.7*norm(rgbdfea(:,idx1(i))/2.0 - rgbdfea(:, idx2(j))/2.0));
       sum = sum + var;
       count = count + 1;
   end
end
sum/count
    
sum = 0;
count = 0;
for i = 1:200
   for j = 1: 200
       %rgbdfea(:,idx1(i))' * rgbdfea(:, idx1(j))
       %sum = sum + rgbdfea(:,idx1(i))' * rgbdfea(:, idx2(j))/4.0;
       sum = sum + exp(- 0.7*norm(rgbdfea(:,idx1(i))/2.0 - rgbdfea(:, idx3(j))/2.0));
       count = count + 1;
   end
end
sum/count
    
sum = 0;
count = 0;
for i = 1:200
   for j = 1: 200
       %rgbdfea(:,idx1(i))' * rgbdfea(:, idx1(j))
       %sum = sum + rgbdfea(:,idx1(i))' * rgbdfea(:, idx2(j))/4.0;
       sum = sum + exp(- 0.7*norm(rgbdfea(:,idx1(i))/2.0 - rgbdfea(:, idx4(j))/2.0));
       count = count + 1;
   end
end
sum/count
    
sum = 0;
count = 0;
for i = 1:200
   for j = 1: 200
       %rgbdfea(:,idx1(i))' * rgbdfea(:, idx1(j))
       %sum = sum + rgbdfea(:,idx1(i))' * rgbdfea(:, idx2(j))/4.0;
       sum = sum + exp(- 0.7*norm(rgbdfea(:,idx1(i))/2.0 - rgbdfea(:, idx5(j))/2.0));
       count = count + 1;
   end
end
sum/count
    
    