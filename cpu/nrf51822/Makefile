MODULE =cpu

DIRS = $(RIOTCPU)/libopencm3

all: $(BINDIR)$(MODULE).a
	@for i in $(DIRS) ; do "$(MAKE)" -C $$i || exit 1; done ;
	@cp $(RIOTCPU)/libopencm3/lib/libopencm3_nrf51822.a $(BINDIR)libopencm3_nrf51822.a

include $(RIOTBASE)/Makefile.base
