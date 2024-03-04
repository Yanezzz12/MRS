CC = g++ 

LIBS = -lm -lpthread

OBJS= Dijkstra.o 
PRG_OBJS= Dijkstra.cpp

Dijkstra: $(OBJS)
	  $(CC)  -o $@ $(OBJS) $(LIBS)

$(OBJS): $(PRG_OBJS)
	$(CC) $(CFLAGS) -c $*.cpp 
	
	
