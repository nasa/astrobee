% print blas

blas

Gflops_max = A(1)*A(2);

f1 = figure();
plot(B(:,1), B(:,2), 'b');
hold on
plot(B(:,1), B(:,4), 'g');
plot(B(:,1), B(:,6), 'r');
hold off

axis([0 300 0 Gflops_max]);
legend('blas', 'blas_pack', 'blasfeo', 'Location', 'SouthEast');
xlabel('matrix size n')
ylabel('Gflops')
grid on

file_name = ['blas.eps'];
print(f1, file_name, '-depsc') 


