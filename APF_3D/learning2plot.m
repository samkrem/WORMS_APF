%v1=[0.2,0.3 4 ],
%v2=[-0.3,0.3,0.1],
%v=[v1;v2];
%disp(v)
%disp(v(:,1))
%disp(v(1))
%plot3(v(:,1),v(:,2),v(:,3),'r')

% Example matrix
matrix = [3 2 1 5 6];

% Sort the matrix in descending order and get the sorted indexes
[sortedMatrix, sortedIndexes] = sort(matrix, 'descend');

% Display the result
disp(sortedMatrix);    % Maximum values in descending order
disp(sortedIndexes);   % Indexes of the maximum values