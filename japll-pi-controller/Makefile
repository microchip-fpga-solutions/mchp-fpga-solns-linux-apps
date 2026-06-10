CC ?= gcc
CFLAGS = -Wall
LDFLAGS = -lm
OBJ = japll-pi.o print.o

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

japll-pi: $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(OBJ) japll-pi
