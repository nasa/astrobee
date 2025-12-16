function py2ju_obj(py_obj)
    py_attr = keys(py_obj)
    n_attr = length(py_attr)
    ju_obj = Dict()
    for i = 1:n_attr
        # skip base class attributes
        if !contains(String(py_attr[i]), "__") 
            if py_obj[py_attr[i]] != nothing
                error("Cannot convert Python object with non-None attribute value $(prop[i]).")
            end
            attr_name = String(py_attr[i])
            ju_obj[attr_name] = nothing
        end
    end
    return ju_obj
end
