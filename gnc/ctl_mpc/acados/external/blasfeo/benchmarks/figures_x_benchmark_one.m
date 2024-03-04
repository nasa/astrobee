% routine to plot figures from data in the folder

%routine = 'dgemm';
routine = 'dtrmm';

routines = dir('*.m');

routines.name;

num_all = length(routines);

f1 = figure();

Gflops_max = 0.0;

for ii=1:num_all
	
	name = routines(ii).name;
	tokens = strsplit(name, '_');

	if (strcmp(tokens{1}, routine))
		
		run(name);

		tokens2 = strsplit(tokens{2}, '.');

		hold all
		plot(B(:,1), B(:,2), 'DisplayName', tokens2{1});
		hold off

		if A(1)*A(2)>Gflops_max
			Gflops_max = A(1)*A(2);
		end
	
	end

end

axis([0 300 0 Gflops_max]);
hlegend = legend(gca, 'show', 'Location', 'SouthEast');
title(routine);
xlabel('matrix size n')
ylabel('Gflops')
grid on
ytick = get(gca, 'ytick');
ytick = [ytick, Gflops_max];
set(gca, 'ytick', ytick);

file_name = [routine];
file_name_eps = [file_name, '.eps'];
file_name_pdf = [file_name, '.pdf'];
print(f1, file_name_eps, '-depsc') 
system(['epstopdf ', file_name_eps, ' -out ', file_name_pdf]);
system(['rm ', file_name_eps]);


