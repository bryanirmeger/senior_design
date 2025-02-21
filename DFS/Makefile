BASE_NAME=DFS
SRC_H= $(BASE_NAME).h $(DEFHEADERS)
SUBMIT_FILES=$(TEST_C) $(SRC_C) $(SRC_H) Makefile

DEFHEADERS=
COPYFILES=$(DEFHEADERS) Makefile
HEADERSDIR=headers

EXECUTABLE=test_$(BASE_NAME)
EXECUTABLE_GCOV=$(EXECUTABLE)_gcov

SRC_C=$(BASE_NAME).c $(OTHER_C)
OTHER_C=
TEST_C=test_$(SRC_C)
SHELL=/bin/bash
CC=gcc
CFLAGS=-g -std=c11 -Wall -Wshadow -Wvla -Werror -pedantic
CFLAGS_GCOV=$(CFLAGS) -ftest-coverage -fprofile-arcs
TEXT_FILES=

$(EXECUTABLE): $(SRC_C) $(TEST_C) $(SRC_H)
	$(CC) -o $(EXECUTABLE) $(SRC_C) $(TEST_C) $(CFLAGS)

.PHONY: test clean coverage copy_headers create_files gdb valgrind push
test: $(EXECUTABLE)
	./$(EXECUTABLE)
clean: 
	rm -f $(EXECUTABLE) $(EXECUTABLE_GCOV) $(wildcard *.gcda) $(wildcard *.c.gcov) $(wildcard *.gcno)
coverage: $(SRC_C) $(TEST_C)
	$(CC) -o $(EXECUTABLE_GCOV) $(SRC_C) $(TEST_C) $(CFLAGS_GCOV)
	./$(EXECUTABLE_GCOV)
	gcov -f $(SRC_C)
copy_headers: 
	cp ../$(HEADERSDIR)/$(COPYFILES) ../$(DIR)
create_files:
	touch $(SRC_C) $(SRC_H) $(TEST_C)
gdb: $(EXECUTABLE)
	gdb $(EXECUTABLE)
valgrind: $(EXECUTABLE)
	valgrind ./$(EXECUTABLE) -s --track-origins=yes --leak-check=full --verbose --show-leak-kinds=all
push: $(STRING)
	git add .
	git commit -m "$(STRING)"
	git push


