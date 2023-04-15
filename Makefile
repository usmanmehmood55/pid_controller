CC       = gcc
CFLAGS   = -Wall -g
LDFLAGS  =
OBJFILES = ring_buffer/ring_buffer.o main.o 
TARGET   = pid_controller.exe

all: $(TARGET)

$(TARGET): $(OBJFILES)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJFILES) $(LDFLAGS)

clean:
	rm -f $(OBJFILES) $(TARGET) *~