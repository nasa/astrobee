function py_obj = m2py_obj(m_obj, py_obj) 
    np = py.importlib.import_module('numpy');
    import hpipm_matlab.*
    py_props = fieldnames(py_obj);

    for iprop = 1:length(py_props)
        py_thisprop = py_props{iprop};
        py_thisprop_value = py_obj.(py_thisprop);
        
        % TODO(andrea): py.NoneType not supported in Octave
        if ~isOctave()
            if py_thisprop_value ~= py.NoneType
                error(['Trying to assign to non-None attribute "', py_thisprop, '" in Python object.']);
            end
        end
        eval(['isempty_attr = isempty(m_obj.', py_thisprop, ');']);
        if ~isempty_attr    
            eval(['attr_type = class(m_obj.', py_thisprop, ');']);
            switch attr_type
                case 'cell'
                    try eval(['py_obj.', py_thisprop, '=', 'hpipm_matlab.m2py_cell(m_obj.', py_thisprop, ', np);']);
                        
                    catch
                        error(['Cannot assign "', py_thisprop, '" to Pyton object.']);
                    end
                case 'double'
                    eval(['attr_size = sum(size(m_obj.', py_thisprop,'));']);
                    if attr_size > 2
                        try eval(['py_obj.', py_thisprop, '=', 'hpipm_matlab.m2py(m_obj.', py_thisprop, ', np);']);
                            
                        catch
                            error(['Cannot assign "', py_thisprop, '" to Pyton object.']);
                        end
                    else
                        try eval(['py_obj.', py_thisprop, '=', 'int32(m_obj.', py_thisprop, ');']);
                            
                        catch
                            error(['Cannot assign "', py_thisprop, '" to Pyton object.']);
                        end
                    end
            end
        end
    eval(['m_thisprop_value =  m_obj.', py_thisprop, ';'])
    end   
end
