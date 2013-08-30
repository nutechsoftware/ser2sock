CXXFLAGS = -DNDEBUG -O3 -ffunction-sections -fdata-sections -pipe -Wall
#LDLIBS = -lssl
#CXXFLAGS = -g
# -fPIC is supported. Please report any breakage of -fPIC as a bug.
# CXXFLAGS += -fPIC
# the following options reduce code size, but breaks link or makes link very slow on some systems
# CXXFLAGS += -ffunction-sections -fdata-sections
LDFLAGS +=
CP = cp
MKDIR = mkdir
LIBPATH = 
ARM_CXX = arm-linux-gnueabi-gcc
ARM_CXXFLAGS += $(CXXFLAGS) -march=armv4
ARM_LDFLAGS += -static-libgcc -static -Wl,--gc-sections
ARM_LDFLAGS += -Xlinker --gc-sections -s
CXX = gcc
LDFLAGS += 
BINARY = ser2sock

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

all: ser2sock

ser2sock: clean $(OBJS)
	$(CXX) $(LDFLAGS) $(LDLIBS) -o $(BINARY) $(OBJS) 

ddwrt: CXX = $(ARM_CXX)
ddwrt: clean $(OBJS)
	$(CXX) $(ARM_LDFLAGS) $(LDLIBS) -o $(BINARY) $(OBJS) 

%.o : %.c
	$(CXX) $(CXXFLAGS) -c $<

clean:
	$(RM) *.o ser2sock
