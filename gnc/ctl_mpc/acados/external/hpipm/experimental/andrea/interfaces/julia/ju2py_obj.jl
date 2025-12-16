function ju2py_obj(ju_obj, py_obj)
    py_attr = keys(py_obj)
    n_attr = length(py_attr)
    for i = 1:n_attr
        # skip base class attributes
        if !contains(String(py_attr[i]), "__") 
            if py_obj[py_attr[i]] != nothing
                error("Cannot assign to non-None attribute $(py_obj[py_attr[i]]) in Python object.")
            end
            py_attr_name = py_attr[i]
            ju_key_name = String(py_attr_name)
            if ju_obj[ju_key_name] != nothing
                ju_eltype = eltype(ju_obj[ju_key_name])
                # assign Julia array of arrays to Python list of numpy arrays
                if ju_eltype == Array{Int64, 1} || ju_eltype == Array{Float64,1} || ju_eltype == Array{Int64,2} || ju_eltype == Array{Float64, 2}
                    try py_obj[py_attr_name] = PyVector(ju_obj[ju_key_name])
                    catch
                        error("Could not assign attribute $(py_attr[i]).")
                    end
                # assign Julia array to numpy array
                elseif (ju_eltype == Int64 || ju_eltype == Float64) && size(ju_obj[ju_key_name]) != ()
                    try py_obj[py_attr_name] = np.array(ju_obj[ju_key_name])
                    catch
                        error("Could not assign attribute $(py_attr[i]).")
                    end
                # assign scalar
                elseif size(ju_obj[ju_key_name]) == ()
                    try py_obj[py_attr_name] = ju_obj[ju_key_name]
                    catch
                        error("Could not assign attribute $(py_attr[i]).")
                    end
                # the Julia Dictionary containts something that it should not contain
                else
                    error("Julia dictionary contains something not an Array{Int/Float64} or Int/Float64.")
                end
            end
        end
    end
    return py_obj 
end
