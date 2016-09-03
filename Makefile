include buildinfo.inc
BUILD := $(shell expr $(BUILD) + 1)
DEBUG := 1

ifeq ($(DEBUG),1)
	BUILDFLAGS=-g3
	BUILDNAME=Debug
	BUILDDIR=dbg
else
	BUILDFLAGS=-O3
	BUILDNAME=Release
	BUILDDIR=rls
endif
DEFINES := -DMAJORVER=\"$(MAJOR)\" -DMINORVER=\"$(MINOR)\" -DPATCHVER=\"$(PATCH)\" -DBUILDVER=\"$(BUILD)\" -DBUILDTYPE=\"$(BUILDNAME)\"
INCLUDES := -Iinc
AR 	:= ar
CXX      := g++
CXXFLAGS := -Wall -pedantic -std=c++11 $(BUILDFLAGS) $(DEFINES) $(INCLUDES)

SOURCES := $(wildcard src/*.cpp)
OBJECTS := $(SOURCES:src/%.cpp=obj/$(BUILDDIR)/%.o)
LIBS=-lpthread  -L. -lArducam 
LIBNAME := libArducam.a
all:  libArducam.a examples

libArducam.a: $(OBJECTS) 
	@echo -n " libArducam"
	@$(AR) rcs $(LIBNAME) $(OBJECTS)

obj/$(BUILDDIR)/%.o : src/%.cpp
	@echo -n "$* "
	@$(CXX) $(CXXFLAGS) -MMD -c -o $@  $<


-include $(SOURCES:src/%.cpp=obj/$(BUILDDIR)/%.d)


examples:  libArducam.a simpleshooter 


simpleshooter: obj/$(BUILDDIR)/simpleshooter.o
	@$(CXX) $(CXXFLAGS) -MMD -o simpleshooter obj/$(BUILDDIR)/simpleshooter.o $(LIBS)

obj/$(BUILDDIR)/simpleshooter.o: examples/simpleshooter.cpp
	@$(CXX) $(CXXFLAGS) -c  examples/simpleshooter.cpp -o $@ 

buildinfo.inc : $(OBJECTS)
	@echo "MAJOR :=$(MAJOR)" > buildinfo.inc
	@echo "MINOR :=$(MINOR)" >> buildinfo.inc
	@echo "PATCH :=$(PATCH)" >> buildinfo.inc
	@echo "BUILD :=$(BUILD)" >> buildinfo.inc

.PHONY: clean all
clean:
	rm -fr obj
	rm libArducam.a
	rm simpleshooter
	mkdir -p obj/dbg
	mkdir -p obj/rls



