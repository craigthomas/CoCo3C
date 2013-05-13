# Copyright (C) 2013 Craig Thomas
# This project uses an MIT style license - see LICENSE for details.

NAME = trs80e

COMMONSRC = cpu.c keyboard.c memory.c screen.c vdg_mc6847.c
COMMONOBJ = cpu.o keyboard.o memory.o screen.o vdg_mc6847.o

TESTSRC = test.c cpu_tests.c memory_tests.c unit_tests.c
TESTOBJ = test.o cpu_tests.o memory_tests.o unit_tests.o

MAINSRC = trs80e.c
MAINOBJ = trs80e.o

CFLAGS += -Wall -g $(shell sdl-config --cflags)
CTESTFLAGS = -fprofile-arcs -ftest-coverage

LDFLAGS += -lSDL_ttf $(shell sdl-config --libs)
LDTESTFLAGS = -fprofile-arcs

.PHONY: test sonar doc clean

trs80e: 
	$(CC) -c $(CFLAGS) $(COMMONSRC) $(MAINSRC)
	$(LINK.c) -o $(NAME) $(COMMONOBJ) $(MAINOBJ) $(LDFLAGS)

test: 
	$(CC) -c $(CFLAGS) $(CTESTFLAGS) $(COMMONSRC) $(TESTSRC)
	$(LINK.c) -o test $(COMMONOBJ) $(TESTOBJ) $(LDFLAGS) $(LDTESTFLAGS)
	./test

sonar:
	$(CC) -c $(CFLAGS) $(CTESTFLAGS) $(COMMONSRC) $(TESTSRC)
	$(LINK.c) -o test $(COMMONOBJ) $(TESTOBJ) $(LDFLAGS) $(LDTESTFLAGS)
	./test
	./gcovr -x -r . > ./gcovr-report.xml
	sonar-runner

doc:
	doxygen doxygen.conf

clean:
	@- $(RM) $(COMMONOBJ) $(MAINOBJ) $(TESTOBJ)
	@- $(RM) $(NAME) test
	@- $(RM) *.gcda *.gcno
