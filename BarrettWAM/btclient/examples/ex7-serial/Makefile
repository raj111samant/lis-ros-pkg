### Makefile for Barrett Software (examples)

# Be sure to edit config.mk to match your installation
include ../../config.mk
include ../../common.mk

OBJS = main.o
TARG = ex7

# Done with defines, now for the real work
default: ex7

ex7: ${OBJS}
	$(CC) -o ${TARG} ${OBJS} ${LDFLAGS}
  
clean:
	rm *.o ${TARG} 

system:
	$(MAKE) -C ../../src/btsystem lib
	$(MAKE) -C ../../src/btsystem install

wam:
	$(MAKE) -C ../../src/btwam lib
	$(MAKE) -C ../../src/btwam install
   
all: system wam ex7

