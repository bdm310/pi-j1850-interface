TARGET = spi
DEST = ./build
LIBS = 
CC = gcc
CFLAGS = -Wall
LDFLAGS = -Wl,-Map,$(DEST)/$(PRG).map

default: $(DEST)/$(TARGET)
all: default

OBJECTS = $(patsubst %.c,%.o,$(wildcard *.c))
HEADERS = $(wildcard *.h)

$(DEST)/%.o: %.c $(HEADERS)
	$(CC) -o $@ $(CFLAGS) -c $< `pkg-config --cflags dbus-1` `pkg-config --libs dbus-1`

$(DEST)/$(TARGET): $(DEST)/$(OBJECTS)
	$(CC) $(DEST)/$(OBJECTS) $(LDFLAGS) -o $@ `pkg-config --cflags dbus-1` `pkg-config --libs dbus-1`

clean:
	-rm -rf $(DEST)/*
