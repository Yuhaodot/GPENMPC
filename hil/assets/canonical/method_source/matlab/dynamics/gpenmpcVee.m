function value = gpenmpcVee(matrix)
%GPENMPCVEE Vee map for a 3-by-3 skew-symmetric matrix.

value = [matrix(3, 2); matrix(1, 3); matrix(2, 1)];
end
