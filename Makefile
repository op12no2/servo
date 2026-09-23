CFLAGS ?= -O2 -Wall -Wextra
servo: servo.c
	$(CC) $(CFLAGS) -o $@ $<
clean:
	rm -f servo
.PHONY: clean
