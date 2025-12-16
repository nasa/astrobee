function py_list = m2py_cell(m_cell, np)
    n = length(m_cell);
    py_cell = cell(n, 1);
    for i = 1:n
        M = m_cell{i};
        np_array = hpipm_matlab.m2py(M, np);
        py_cell{i} = np_array;
    end
    py_list = py.list(py_cell.');
end