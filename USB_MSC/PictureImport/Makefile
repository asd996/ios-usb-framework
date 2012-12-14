TARGETS=usb_ls usb_cp
CC:=gcc
CFLAGS=-O2 -g
LDFLAGS=-lusb-1.0

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

usb_ls: dosfs.o ls.o
	$(CC) -o usb_ls dosfs.o ls.o $(LDFLAGS)

usb_cp: dosfs.o export.o
	$(CC) -o usb_ls dosfs.o export.o $(LDFLAGS)

all:	$(TARGETS)

clean:
	rm -f *.o $(TARGETS)
