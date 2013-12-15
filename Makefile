MODE=DEVELOP    #or DEBUG

CXX=g++
ifeq ($(MODE),DEBUG)
CXXFLAGS=-O0 -g3 -Wall -DRECON_DEBUG
else
CXXFLAGS=-O3
endif
RECON_CXXFLAGS=-I${shell pwd}
GIT_REV=${shell git log -1 --format="%h"}

prefix=/usr/local
exec_prefix=$(prefix)
bindir=$(exec_prefix)/bin
includedir=$(prefix)/include
libdir=$(exec_prefix)/lib
INSTALL=install
INSTALL_PROGRAM=$(INSTALL)
INSTALL_DATA=$(INSTALL) -m 644

recon: bin/recon
test: bin/test

all: recon test

check: test
	@bin/test --gtest_color=yes

bin/recon: recon.hpp test/recon.cc Makefile
	@mkdir -p $$(dirname $@)
	$(CXX) $(CXXFLAGS) $(RECON_CXXFLAGS) test/recon.cc -o $@ -lgflags -DGIT_REV=\"$(GIT_REV)\"

bin/test: recon.hpp test/test.cc Makefile
	@mkdir -p $$(dirname $@)
	$(CXX) $(CXXFLAGS) $(RECON_CXXFLAGS) test/test.cc test/gtest/gtest-all.cc test/gtest/gtest_main.cc -Itest -o $@

install: recon.hpp recon
	mkdir -p $(DESTDIR)$(includedir) $(DESTDIR)$(bindir)
	$(INSTALL_DATA) recon.hpp $(DESTDIR)$(includedir)/recon.hpp
	$(INSTALL_PROGRAM) bin/recon $(DESTDIR)$(bindir)/recon

uninstall:
	rm $(DESTDIR)$(includedir)/recon.hpp $(DESTDIR)$(bindir)/recon

clean:
	rm -rf bin
