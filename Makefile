CXXFLAGS = -DNDEBUG -O3 -ffunction-sections -fdata-sections
LDLIBS = -lssl
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
ARM_CXXFLAGS += -march=armv4 -pipe -Wall
ARM_LDFLAGS += -static-libgcc -static -Wl,--gc-sections
ARM_LDFLAGS += -Xlinker --gc-sections -s

CXX = gcc
CXXFLAGS += -pipe -Wall
LDFLAGS += 

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

all: ser2sock

ser2sock: clean $(OBJS)
	$(CXX) $(LDFLAGS) $(LDLIBS) -o $@ $(OBJS) 

ser2sockmips: CXX = $(ARM_CXX)
ser2sockmips: clean $(OBJS)
	$(CXX) $(ARM_LDFLAGS) $(LDLIBS) -o $@ $(OBJS) 

%.o : %.c
	$(CXX) $(CXXFLAGS) -c $<

clean:
	$(RM) *.o ser2sockmips ser2sock
