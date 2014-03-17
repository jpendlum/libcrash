CC = gcc
LDFLAGS := -ludev
CFLAGS := -c -Wall -Werror
INCLUDES := -I.

.PHONY : all
all : libcrash

libcrash.o :
	$(CC) $(CFLAGS) $(INCLUDES) -fpic libcrash.c

libcrash : libcrash.o
	$(CC) -shared -o libcrash.so libcrash.o $(LDFLAGS)

install : libcrash
	cp libcrash.so /usr/local/lib
	cp libcrash.h /usr/local/include
	chmod 0755 /usr/local/lib/libcrash.so
	ldconfig

uninstall :
	rm /usr/local/lib/libcrash.so
	rm /usr/local/include/libcrash.h
	ldconfig

.PHONY : clean
clean :
	rm -f *.o libcrash.so