#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define NUM_NODES 250


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

   nodes[D].acumulado = 0;

   while( nodes[L].flag != 'Y')
   {  
      for(j = 0 ; j < nodes[D].num_conections; j++)
         {
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
   }
}


int dijkstra(float rx ,float ry ,float lx ,float ly, char *path, char *world_name,step *steps )
{
   char archivo[150];
   int i;
   int start = 0;
   int goal = 0;
   int padre;


   for(i = 0; i < NUM_NODES; i++)
   {
      nodes[i].flag='N';
      nodes[i].num_conections = 0;
      nodes[i].parent = -1;
      nodes[i].acumulado = 0;
   }
 
   // it reads the topological file
   sprintf(archivo,"%s%s.top",path,world_name);
   num_nodes=read_nodes(archivo); // Se lee el arcivo .top


   for(i = 1; i < num_nodes; i++)
   {
   		if( sqrt(pow( nodes[i].x - rx ,2) + pow( nodes[i].y - ry ,2)) < sqrt( pow( nodes[start].x - rx ,2) + pow( nodes[start].y - ry ,2)) )	
   			start = i;
   		
   		if( sqrt(pow( nodes[i].x - lx ,2) + pow( nodes[i].y - ly ,2)) < sqrt(pow( nodes[goal].x - lx ,2) + pow( nodes[goal].y - ly ,2) ) )
   			goal = i;
   }

   dijkstra_algorithm (goal ,start); // Se pasan al reves para no tener que voltear la lista resultante.
   
   padre = start;
   i = 0;

   while( padre != -1)
   {
   	 steps[i].node = nodes[padre].num_node;
   	 steps[i].x = nodes[padre].x;
   	 steps[i].y = nodes[padre].y;
       	 i++;
   	 padre = nodes[padre].parent;
   }
	return i;
} 
