#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define MUM_NODES 350

typedef struct conection_
{
   int node;
   float cost;
}  conection;

typedef struct nodo_
{
   char flag;
   int num_node;
   float x;
   float y;
   int num_conections;
   conection conections[600];
   int parent;
   float acumulado;
}  nodo;

typedef struct step_
{
   int node;
   float x;
   float y;
   float acumulado;
}  step;



nodo nodes[MUM_NODES];
int num_nodes = 0;

// it reads the file that conteins the environment description
int read_nodes(char *file)
{
   FILE *fp;
   char data[ 200 ];
   int i=0;
   int flg = 0;
   float tmp;
   float dimensions_room_x,dimensions_room_y;
   int node_index,node_connection,cost;

   fp = fopen(file,"r"); 
    
   if( fp == NULL )
   {
      sprintf(data, "File %s does not exists\n", file);
      //printf("File %s does not exists\n", file);
      return(0);
   }

   while( fscanf(fp, "%s" ,data) != EOF)
   {
      if( strcmp(";(", data ) == 0 )
      {
         flg = 1;
         while(flg)
         {
            if(  0 < fscanf(fp, "%s", data));
            sscanf(data, "%f", &tmp);
            if(strcmp(")", data) == 0) flg = 0;
         }
      }
      else if((strcmp("node", data ) == 0) && ( flg == 0 ) )
      {
         if(  0 < fscanf(fp, "%s", data));
         nodes[i].num_node = atoi(data);
         if(  0 < fscanf(fp, "%s", data));
         nodes[i].x = atof(data);
         if(  0 < fscanf(fp, "%s", data));
         nodes[i++].y = atof(data);
      }
      else if((strcmp("connection", data ) == 0) && ( flg == 0 ) )
      {
         if(  0 < fscanf(fp, "%s", data));
         node_index = atoi(data);
         
         if(  0 < fscanf(fp, "%s", data));
         node_connection = atoi(data);

         nodes[node_index].conections[nodes[node_index].num_conections].node = node_connection;

         if(  0 < fscanf(fp, "%s", data));
         nodes[node_index].conections[nodes[node_index].num_conections].cost = atof(data);
         nodes[node_index].num_conections++;
      }
   }
   fclose(fp);
   return i;
}

void dijkstra_algorithm(int D ,int L)
{
   /*
      D = Nodo Inicial
      L = Nodo Final
      Y = Totalmente expandido
      N = Nuevo (Nodo sin padre ni acumulado)
      P = Nodo que no es ni Y ni N   (Parcialmente expandido)

      Video explicativo https://www.youtube.com/watch?v=LLx0QVMZVkk
   */

   int menor,flagOnce;
   int contador = 0;
   int j;
   int kk=0;

   nodes[D].acumulado = 0;

   while( nodes[L].flag != 'Y')
   {  
      for(j = 0 ; j < nodes[D].num_conections; j++)
         {

	    //printf("node %d\n",j);

            if( nodes[ nodes[D].conections[j].node].flag == 'N')
            {
               nodes[nodes[D].conections[j].node].acumulado = nodes[D].acumulado + nodes[D].conections[j].cost;
               nodes[nodes[D].conections[j].node].parent = D;
               nodes[nodes[D].conections[j].node].flag = 'P';
            }  
            else if( nodes[nodes[D].conections[j].node].flag == 'P' )
            {
               if( nodes[nodes[D].conections[j].node].acumulado > nodes[D].acumulado + nodes[D].conections[j].cost)
               {
                  nodes[nodes[D].conections[j].node].acumulado = nodes[D].acumulado + nodes[D].conections[j].cost;
                  nodes[nodes[D].conections[j].node].parent = D;
               }
            }
         }

      nodes[D].flag = 'Y';
         menor = 0;
         flagOnce = 1;
         for(int j = 0; j < num_nodes ; j++)
         {
	    //printf("node jj %d\n",j);
            if(nodes[j].flag == 'P')
            {
               if(flagOnce)
               {
                  menor = j;
                  flagOnce = 0;
               }
               else if( nodes[menor].acumulado > nodes[j].acumulado )
               {
                  menor = j;
               }
            }  
         }
         D = menor;
   
    kk++;
    //printf("kk %d\n",kk);
    if(kk > 1000) break;
   }


}



void printNode(int i) // use it for debug
{
      printf("\n\n");
      printf("# %d  x   %f y %f\n",nodes[i].num_node,nodes[i].x,nodes[i].y );
      printf("flag: %c parent: %d   acumulado: %f  \n",nodes[i].flag,nodes[i].parent,nodes[i].acumulado  );
      printf("num_conections %d \n",nodes[i].num_conections);
      for(int j=0 ; j < nodes[i].num_conections; j++  )
         printf(     "%d  %f \n",nodes[i].conections[j].node,nodes[i].conections[j].cost );
}





float dijkstra(float rx ,float ry ,float lx ,float ly, char *world_name, step *steps)
{
   char archivo[150];
   int i;
   int start = 0;
   int goal = 0;
   int padre;
   float acumulado=0;
   float distance_origin, distance_destination;


   strcpy(archivo,world_name);
   strcat(archivo,".top");

   for(i = 0; i < MUM_NODES; i++)
   {
      nodes[i].flag='N';
      nodes[i].num_conections = 0;
      nodes[i].parent = -1;
      nodes[i].acumulado = 0;
   }
 
   num_nodes=read_nodes(archivo); // Se lee el arcivo .top
   if(num_nodes == 0) return acumulado;

   for(i = 1; i < num_nodes; i++)
   {
   		if( sqrt(pow( nodes[i].x - rx ,2) + pow( nodes[i].y - ry ,2)) < sqrt( pow( nodes[start].x - rx ,2) + pow( nodes[start].y - ry ,2)) )	
   			start = i;
   		
   		if( sqrt(pow( nodes[i].x - lx ,2) + pow( nodes[i].y - ly ,2)) < sqrt(pow( nodes[goal].x - lx ,2) + pow( nodes[goal].y - ly ,2) ) )
   			goal = i;
   }

   //printf("start %d goal %d\n",start,goal);

   dijkstra_algorithm (goal ,start); // Se pasan al reves para no tener que voltear la lista resultante.
   distance_origin = sqrt(pow( nodes[start].x - rx ,2) + pow( nodes[start].y - ry ,2));  
   distance_destination = sqrt(pow( nodes[goal].x - lx ,2) + pow( nodes[goal].y - ly ,2));  

 
   padre = start;
   acumulado = nodes[padre].acumulado + distance_origin + distance_destination;
   //printNode(start);

   i = 0;

   while( padre != -1)
   {
   	 steps[i].node = nodes[padre].num_node;
   	 steps[i].x = nodes[padre].x;
   	 steps[i].y = nodes[padre].y;
   	 steps[i].acumulado = nodes[padre].acumulado;
	 //printf("%d node %d x %f y %f accumulated %f\n",i,steps[i].node,steps[i].x,steps[i].y,steps[i].acumulado); 
	 i++;
   	 padre = nodes[padre].parent;
   }


   return acumulado;


}

 
