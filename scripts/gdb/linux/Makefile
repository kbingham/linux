always := gdb-scripts

SRCTREE := $(shell cd $(srctree) && /bin/pwd)

$(obj)/gdb-scripts: $(obj)/constants.py
ifneq ($(KBUILD_SRC),)
	$(Q)ln -fsn $(SRCTREE)/$(obj)/*.py $(objtree)/$(obj)
endif
	@:

$(obj)/constants.py: $(SRCTREE)/$(obj)/constants.py.in
	@echo "  GDB PP  $@"
	@$(CPP) -E -x c -P $(c_flags) $< > $@
	@sed -i '1,/<!-- end-c-headers -->/d;' $@

clean-files := *.pyc *.pyo $(if $(KBUILD_SRC),*.py) $(obj)/constants.py
