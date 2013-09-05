ifndef ARCH
ARCH=native
endif

BINARY = ser2sock
CXX = 
DEFAULT_CXX = gcc
CROSS_CXX = arm-linux-gcc

DYNAMIC_CXXFLAGS = -DNDEBUG -O3 -ffunction-sections -fdata-sections -pipe -Wall -DSSL_SUPPORT -march=$(ARCH)
DYNAMIC_LDFLAGS = -lssl
STATIC_CXXFLAGS += $(DYNAMIC_CXXFLAGS)
STATIC_LDFLAGS += -static-libgcc -static -Wl,--gc-sections -Xlinker --gc-sections -s -march=$(ARCH) -lssl

ifndef CXXFLAGS

.PHONY: default all clean dynamic static arm mips

default all: dynamic

dynamic: export CXX := $(DEFAULT_CXX)
dynamic: export CXXFLAGS := $(DYNAMIC_CXXFLAGS)
dynamic: export LDFLAGS := $(DYNAMIC_LDFLAGS)
static: export CXX := $(DEFAULT_CXX)
static: export CXXFLAGS := $(STATIC_CXXFLAGS)
static: export LDFLAGS := $(STATIC_LDFLAGS)
arm: export CXX := $(CROSS_CXX)
arm: export ARCH := armv4
arm: export CXX_FLAGS := $(STATIC_CXXFLAGS)
arm: export LDFLAGS := $(STATIC_LDFLAGS)
mips: export CXX := $(CROSS_CXX)
mips: export ARCH := mips32
mips: export CXX_FLAGS := $(STATIC_CXXFLAGS)
mips: export LDFLAGS := $(STATIC_LDFLAGS)

dynamic static arm mips:
	@$(MAKE) CXX=$(CXX)

clean:
	$(RM) *.o $(BINARY)

else

CP = cp
MKDIR = mkdir
LIBPATH = 

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

dynamic static arm mips: clean $(OBJS)
	$(CXX) -o $(BINARY) $(OBJS) $(LDFLAGS) $(LDLIBS)

%.o : %.c
	$(CXX) $(CXXFLAGS) -c $<

clean:
	$(RM) *.o $(BINARY)

endif
