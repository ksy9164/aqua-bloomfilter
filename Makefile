LIBPATH =../../
BOARD=vc707
BUILDTOOLS=$(LIBPATH)/buildtools/

BLIBPATH=$(LIBPATH)/../bluelib/src/

CUSTOMBSV= -p +:$(BLIBPATH)/:./lib:/mnt/hdd0/Bluespec/bsc/inst/BSVSource/Misc:$(LIBPATH)/dram/src:$(LIBPATH)/dram/vc707
CUSTOMCPP_BSIM= $(BLIBPATH)/bdpi.cpp

include $(BUILDTOOLS)/Makefile.base


