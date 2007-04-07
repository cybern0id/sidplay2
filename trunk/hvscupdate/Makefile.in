
srcdirs 		= src

.PHONY: all
all:
		@for subdir in $(srcdirs); do \
			(cd $$subdir && $(MAKE) all) || exit 1; \
		done

Update.static:
		@for subdir in $(srcdirs); do \
			(cd $$subdir && $(MAKE) Update.static) || exit 1; \
		done

.PHONY: clean
clean:
		@for subdir in $(srcdirs); do \
			(cd $$subdir && $(MAKE) clean) || exit 1; \
		done

.PHONY: distclean
distclean:
		$(RM) -f config.cache config.status config.log
		$(RM) -f Makefile *.d *.o *.bak *~
		@for subdir in $(srcdirs); do \
			(cd $$subdir && $(MAKE) distclean) || exit 1; \
		done

.PHONY: depend
depend:
		@for subdir in $(srcdirs); do \
			(cd $$subdir && $(MAKE) depend) || exit 1; \
		done

