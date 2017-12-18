# Compiler definitions:
CXX	:= g++ -std=c++11 -pedantic -pthread -lm 
CC = gcc -pthread -lrt -lm 

# Main files definition:
MAIN = Main
SOURCES = $(MAIN).cpp Datalogger.cpp Comunicacao.cpp Joy.cpp 
HEADERS = Datalogger.h Comunicacao.h Joy.h

# Modules definition:
SUBDIRS = gdatalogger
CLEANDIRS = $(SUBDIRS:%=clean-%)

EXECUTABLE = Birrotor


# Recipes:
.PHONY: all
all: subdirs $(SOURCES) $(HEADERS) $(SUBDIRS) $(EXECUTABLE) 

.PHONY: subdirs $(SUBDIRS)
subdirs: $(SUBDIRS)
$(SUBDIRS):
	@$(MAKE) -C $@
@echo Objects: $(OBJECTS)


OBJECTS = $(MAIN).o Datalogger.o Comunicacao.o Joy.o
OBJECTS += $(wildcard gdatalogger/*.o) 

# Datalogger Module
Datalogger.o: Datalogger.cpp Datalogger.h
	$(CXX) -c Datalogger.cpp

# Comunicação Module
Comunicacao.o: Comunicacao.cpp Comunicacao.h
	$(CXX) -c Comunicacao.cpp

# Joystick Module
Joy.o: Joy.cpp Joy.h
	$(CXX) -c Joy.cpp

# Main files:
$(MAIN).o: $(MAIN).cpp $(HEADERS) subdirs

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ -lrt

# Print all Objects
.PHONY: print
print:
        @echo Sources: $(SOURCES)
        @echo Objects: $(OBJECTS)
        @echo Headers: $(HEADERS)
        @echo Executable: $(EXECUTABLE)

        @echo Subdirs: $(SUBDIRS)
        @echo Cleandirs: $(CLEANDIRS)

# Clean-up everything
.PHONY: subdirs $(CLEANDIRS)
clean: $(CLEANDIRS)
$(CLEANDIRS): 
	@$(MAKE) -C $(@:clean-%=%) clean
	-rm -f $(EXECUTABLE) *.o
