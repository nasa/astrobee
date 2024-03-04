+++
title = "Routines List"
order = 2
+++


List on routines implemented in BLASFEO for a reference about the namining
conventions adopted see [Naming Conventions](/docs/naming)

{% for rtn_class_name, rtn_class in routines.items() %}

## {{rtn_class_name}}
{% for rtn_name, rtn_meta in rtn_class.items() %} - {{ rtn_name }}
{% endfor %}

{% endfor %}
