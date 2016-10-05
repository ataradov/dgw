UNAME := $(shell uname)

SRCS = \
  main.c \
  test.c \
  i2c.c \
  spi.c \
  gpio.c

HDRS = \
  main.h \
  test.h \
  dgw.h \
  i2c.h \
  spi.h \
  gpio.h

ifeq ($(UNAME), Linux)
  BIN = dgw
  SRCS += dgw_lin.c
  LIBS += -ludev
else
  BIN = dgw.exe
  SRCS += dgw_win.c
  LIBS += -lhid -lsetupapi
endif

CFLAGS += -W -Wall -Wextra -O2 -std=gnu99
CFLAGS += -Wno-deprecated-declarations

all: $(BIN)

$(BIN): $(SRCS) $(HDRS) $(HIDAPI)
	gcc $(CFLAGS) $(SRCS) $(LIBS) -o $(BIN)

clean:
	rm -rvf $(BIN)

