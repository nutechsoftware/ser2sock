CXXFLAGS = -DNDEBUG -O3 -ffunction-sections -fdata-sections
#CXXFLAGS = -g
# -fPIC is supported. Please report any breakage of -fPIC as a bug.
# CXXFLAGS += -fPIC
# the following options reduce code size, but breaks link or makes link very slow on some systems
# CXXFLAGS += -ffunction-sections -fdata-sections
LDFLAGS +=
CP = cp
MKDIR = mkdir
LIBPATH = 
CXX = arm-linux-gcc
CXXFLAGS += -march=armv4 -pipe -Wall
LDFLAGS += -static-libgcc -static -Wl,--gc-sections
LDFLAGS += -Xlinker --gc-sections -s

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

all: ser2sock

ser2sock: $(OBJS)
	$(CXX) $(LDFLAGS) $(LDLIBS) -o $@ $(OBJS) 

%.o : %.c
	$(CXX) $(CXXFLAGS) -c $<

clean:
	$(RM) *.o ser2sock
