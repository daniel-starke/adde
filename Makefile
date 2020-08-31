PREFIX = 
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++

NO_GPL = 1
UNICODE = 1
#DEBUG = 1
USR = usr

BL := \(
BR := \)
override USR := $(USR:/=)
SRC = $(wildcard src/*.c) $(wildcard src/*.cpp) $(wildcard src/utility/*.c) $(shell find $(USR) -type f $(BL) -name "*.c" -or -name "*.cpp" -or -name "*.ino" $(BR))

COMMON_CFLAGS = -DADDE=1,0,2 -Wall -Wextra -Wshadow -Wformat -Wconversion -flto -mstackrealign -fno-ident
ifeq (1, $(NO_GPL))
 COMMON_CFLAGS += -DNO_GPL
endif
ifeq (1, $(DEBUG))
 COMMON_CFLAGS += -Og -g3 -ggdb -gdwarf-3 -fvar-tracking-assignments -fbounds-check -fstack-protector-strong
else
 COMMON_CFLAGS += -O2 -DNDEBUG
endif
PATHS = -Isrc
SYS := $(shell $(CC) -dumpmachine)
ifneq (, $(findstring linux, $(SYS)))
 COMMON_CFLAGS += -D_BSD_SOURCE -D_POSIX_C_SOURCE=200112L -D_XOPEN_SOURCE -D_XOPEN_SOURCE_EXTENDED -D_LARGEFILE64_SOURCE
 CFLAGS = -std=c99 $(COMMON_CFLAGS)
 CXXFLAGS = -std=c++11 -nostdlib -fno-exceptions -fno-rtti $(COMMON_CFLAGS)
 LDFLAGS = -fno-ident
 LIBS = -lpthread
 OBJEXT = .o
 BINEXT = 
else
 ifneq (, $(findstring mingw, $(SYS))$(findstring windows, $(SYS)))
  COMMON_CFLAGS += -D__USE_MINGW_ANSI_STDIO
  CFLAGS = -std=c99 $(COMMON_CFLAGS)
  CXXFLAGS = -std=c++11 -nostdlib -fno-exceptions -fno-rtti $(COMMON_CFLAGS)
  LDFLAGS = -static -fno-ident
  ifeq (1, $(UNICODE))
   CFLAGS += -municode
   CXXFLAGS += -municode
   LDFLAGS += -municode
  endif
  LIBS = -lwinmm
  OBJEXT = .o
  BINEXT = .exe
  ifeq (, $(findstring __MINGW64__, $(shell $(CC) -dM -E - </dev/null 2>/dev/null)))
   # patch to handle missing symbols in mingw32 correctly
   CFLAGS += -D__MINGW64__=1
   CXXFLAGS += -D__MINGW64__=1
  endif
 else
  COMMON_CFLAGS += -D_BSD_SOURCE -D_POSIX_C_SOURCE=200112L -D_XOPEN_SOURCE -D_XOPEN_SOURCE_EXTENDED -D_LARGEFILE64_SOURCE
  CFLAGS = -std=c99 $(COMMON_CFLAGS)
  CXXFLAGS = -std=c++11 -nostdlib -fno-exceptions -fno-rtti $(COMMON_CFLAGS)
  LDFLAGS = -fno-ident
  LIBS = -lpthread
  OBJEXT = .o
  BINEXT = 
 endif
endif

OBJ = $(patsubst $(USR)%,bin/usr%,$(patsubst src%,bin%,$(patsubst %.c,%$(OBJEXT),$(patsubst %.cpp,%$(OBJEXT),$(patsubst %.ino,%$(OBJEXT),$(SRC))))))

all: bin bin/usr bin/utility bin/adde$(BINEXT)

.PHONY: clean
clean:
	rm -rf bin/*

bin:
	mkdir bin

bin/usr:
	mkdir -p $(patsubst $(USR)%,bin/usr%,$(shell find $(USR) -type d))

bin/utility:
	mkdir bin/utility

src/Arduino.cpp: src/License.i
src/License.i: doc/COPYING script/convert-license.sh
	script/convert-license.sh doc/COPYING $@

bin/%.o: src/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

bin/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

bin/usr/%.o: $(USR)/%.c
	$(CC) $(CFLAGS) $(PATHS) -c -o $@ $<

bin/usr/%.o: $(USR)/%.cpp
	$(CXX) $(CXXFLAGS) $(PATHS) -c -o $@ $<

bin/usr/%.o: $(USR)/%.ino
	$(CXX) -x c++ $(CXXFLAGS) $(PATHS) -include src/Arduino.h -c -o $@ $<

bin/utility/%.o: src/utility/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: bin/adde$(BINEXT)
bin/adde$(BINEXT): $(OBJ)
	rm -f $@
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $+ $(LIBS)
