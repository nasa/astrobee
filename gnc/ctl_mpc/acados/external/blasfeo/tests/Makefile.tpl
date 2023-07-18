TESTS_DIR=$(BLASFEO_PATH)/tests
ABS_BINARY_PATH=$(TESTS_DIR)/$(BINARY_DIR)

include $(BLASFEO_PATH)/Makefile.rule

LIBS =
SHARED_LIBS =

LIBS += $(ABS_BINARY_PATH)/libblasfeo.a
SHARED_LIBS += -Wl,-rpath=$(ABS_BINARY_PATH) -L $(ABS_BINARY_PATH) -lblasfeo

LIBS += -lm
SHARED_LIBS += -lm

LIBS += $(LIBS_EXTERNAL_BLAS)
SHARED_LIBS += $(SHARED_LIBS_EXTERNAL_BLAS)

{% for flag, value in test_macros.items() %}
{%- if value -%}
	CFLAGS += -D{{flag | upper}}={{value}}
{% else -%}
	CFLAGS += -D{{flag | upper}}
{% endif -%}
{%- endfor -%}

{% if TEST_BLAS_API in test_macros -%}
ifeq ($(EXTERNAL_BLAS), 0)
$(error No EXTERNAL_BLAS specified, install specify one reference blas implementation i.e. OPENBLAS)
{%- endif %}

test.o:
	# build executable obj $(ABS_BINARY_PATH)
	$(CC) $(CFLAGS) -c $(TESTS_DIR)/test.c -o $(ABS_BINARY_PATH)/test.o
	$(CC) $(CFLAGS) $(ABS_BINARY_PATH)/test.o -o $(ABS_BINARY_PATH)/test.out $(LIBS)
	$(ABS_BINARY_PATH)/test.out
#	~/sde/sde64 -- $(ABS_BINARY_PATH)/test.out


run: test.o

update: run
full: update_lib run
