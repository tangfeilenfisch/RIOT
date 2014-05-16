MODULE =cpu

DIRS = $(RIOTCPU)/libopencm3

all: $(BINDIR)$(MODULE).a
	@for i in $(DIRS) ; do "$(MAKE)" -C $$i || exit 1; done ;
	@cp $(RIOTCPU)/libopencm3/lib/libopencm3_stm32f0.a $(BINDIR)libopencm3_stm32f0.a

include $(RIOTBASE)/Makefile.base
