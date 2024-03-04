CC = g++ 

LIBS = -lm -lpthread

OBJS= GoTo_State_Machine.o 
PRG_OBJS= GoTo_State_Machine.cpp

GoTo_State_Machine: $(OBJS)
	  $(CC)  -o $@ $(OBJS) $(LIBS) $(SOCKET_LIBS) $(OBJS_LASER)

$(OBJS): $(PRG_OBJS)
	$(CC) $(CFLAGS) -c $*.cpp 

	
	
