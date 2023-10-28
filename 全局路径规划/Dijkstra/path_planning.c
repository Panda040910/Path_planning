/**
  ******************************************************************************
  * @file    path_planning.c
  * @brief   This file provides code for the path_planning of Rc2024 ,
  *          the R1 pick the ball,based on  Dijkstra's algorithm function 
	* @author  PanDezhi  
	* @Date    2023.10.12 
  ******************************************************************************
  * @attention
  * struct Item {
  *      uint8_t number;                // Number of each grain  
  *      double distance;           // distance for caculation
	*      enum Status status;        // state taken or not_taken
	*      enum BallColor color;      // full or none 
	*      uint8_t init_distance;         // Distance from starting point
	*      uint8_t total_distance;        // shortest distance of path
	*      uint8_t total_order[12];       // shortest path 
  *  };
  * 
  * int main
  * {
  *  struct Item result = Path_all(num);  //the full grain u want to pick
  *  while{}
  * }
  ******************************************************************************
  *   O O O O O O
  *   L K J I H G
  * 
  *   O O O O O O
  *   F E D C B A
  ******************************************************************************  
**/
#include "path_planning.h"
#include "main.h"
#include <stdlib.h>          // for qsort
#include <stdio.h>
#include <stdbool.h>
#include <limits.h>

uint8_t source;                                       //The source point in the algorithm, i.e. the starting point
uint8_t V=12;                                         // Number of vertices in the graph
uint8_t init_distance[12]={0,0,0,0,0,0,0,0,0,0,0,0};  //The distance between each point and the slope of Zone 1 and Zone 2

//The graph in the algorithm, the relationship between each point and the point it can reach
//       A  B  C  D  E  F  G  H  I  J  K  L
uint8_t  graph[12][12] = {
        {0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},  //  Distances from  A to other 
        {1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0},  //  B  
        {0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},  //  C 
        {0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0},  //  D 
        {0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0},  //  E  
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1},  //  F 
        {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  //  G
        {0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0},  //  H
        {0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0},  //  I
        {0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0},  //  J
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1},  //  K
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0},  //  L 
				
    };

//The initial state of each grain		
 struct Item Grain_init[12] = {
        {1, 0, NOT_TAKEN, PURPLE, 0, 0},
        {2, 0, NOT_TAKEN, Full, 0, 0},
        {3, 0, NOT_TAKEN, PURPLE, 0, 0},
        {4, 0, NOT_TAKEN, Full, 0, 0},
				{5, 0, NOT_TAKEN, PURPLE, 0, 0},
				{6, 0, NOT_TAKEN, Full, 0, 0},
				{7, 0, NOT_TAKEN, Full, 0, 0},
				{8, 0, NOT_TAKEN, PURPLE, 0, 0},
				{9, 0, NOT_TAKEN, PURPLE, 0, 0},
				{10, 0, NOT_TAKEN, Full, 0, 0},
				{11, 0, NOT_TAKEN, Full, 0, 0},
        {12, 0, NOT_TAKEN, PURPLE, 0, 0}
    };
 
		
struct Item Grain[12];                                //caculate 
struct Item Empty_Grain[6];                           //Store empty valley information
struct Item Full_Grain[6];                            //Store Full valley information
struct Item Total_distance[6];                        //Six empty valleys corresponding to the path
int Empty_index = 0;
int Full_index = 0;		
		
/**********************************************************************
 *@ 	name		: Givecolor
 *@	function: Vision gives data
 *@	input		: none
 *@	output	: none
 ***********************************************************************/		
int Givecolor()
	{
		
	
	}
	
/**********************************************************************
 *@ 	name		: compare_single
 *@	function: Sort distance in an array in ascending order
 *@	input		: none
 *@	output	: none
 ***********************************************************************/		
int compare_single(const void *a, const void *b) {
    double dist_a = ((struct Item *)a)->distance;
    double dist_b = ((struct Item *)b)->distance;
    return (dist_a > dist_b) - (dist_a < dist_b);
}

/**********************************************************************
 *@ 	name		: compare_total
 *@	function: Sort total_distance in an array in ascending order
 *@	input		: none
 *@	output	: none
 ***********************************************************************/		
int compare_total(const void *a, const void *b) {
    double dist_a = ((struct Item *)a)->total_distance;
    double dist_b = ((struct Item *)b)->total_distance;
    return (dist_a > dist_b) - (dist_a < dist_b);
}	
 
/**********************************************************************
 *@ 	name		: minDistance
 *@	  function: Function to find the vertex with the minimum distance value
 *@param dist   Array storing the distances from the source vertex 
 *              to each vertex.
 *@param sptSet Boolean array indicating whether a vertex is 
 *              included in the shortest path tree.
 *@	   output	: The index of the vertex with the minimum distance value 
 *              among those not included.
 ***********************************************************************/	
int minDistance(int dist[], bool sptSet[]) {
    int min = INT_MAX, min_index;

    for (int v = 0; v < V; v++) {
        if (!sptSet[v] && dist[v] < min) {
            min = dist[v];
            min_index = v;
        }
    }

    return min_index;
}


/**********************************************************************
 *@ 	name		: printSolution 
 *@	function: Function to prints the shortest path distance from 
 *@           the source node to each vertex
 *@	input		: none
 *@	output	: none
 ***********************************************************************/	
void printSolution(int dist[],int V) {
    printf("Vertex   Shortest Distance\n");
    for (int i = 0; i < V; i++) {
        printf("%d        %d\n", i, dist[i]);
			  Grain[i].distance=dist[i];
    }
}


/**********************************************************************
 *@ 	name		: dijkstra
 *@	function: Dijkstra's algorithm function 
 *@	input		: src point
 *@	output	: none
 ***********************************************************************/	
void dijkstra(int src) {
	  src=src-1;
    int dist[V];     // Stores the shortest distance from the source to each vertex
    bool sptSet[V];  // If a vertex is included in the shortest path tree, sptSet[i] is true

    // Initialize all distances as INFINITE and all vertices as not in the shortest path tree
    for (int i = 0; i < V; i++) {
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }

    // Distance from the source vertex to itself is always 0
    dist[src] = 0;

    // Find the shortest path for all vertices
    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        // Update distances of adjacent vertices of the picked vertex
        for (int v = 0; v < V; v++) {
            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX && dist[u] + graph[u][v] < dist[v]) {
                dist[v] = dist[u] + graph[u][v];
            }
        }
    }

    // Print the shortest path
    printSolution(dist, V);
}


/**********************************************************************
 *@ 	name		: color_grouping
 *@	function: Classify grain
 *@	input		: none
 *@	output	: none
 ***********************************************************************/	
void color_grouping()
{
	for (int i = 0; i < 12; i++) 
	{
        if (Grain[i].color == PURPLE&&Grain[i].status == NOT_TAKEN) 
            Empty_Grain[Empty_index++] = Grain[i];
				if (Grain[i].color == Full&&Grain[i].status == NOT_TAKEN) 
            Full_Grain[Full_index++] = Grain[i];
   }
}


/**********************************************************************
 *@ 	name		: sort_distance
 *@	function: ascending array
 *@	input		: none
 *@	output	: none
 ***********************************************************************/	
void sort_distance()
{
    qsort(Empty_Grain, Empty_index, sizeof(struct Item), compare_single);
    qsort(Full_Grain, Full_index, sizeof(struct Item), compare_single);
}


/**********************************************************************
 *@ 	name		: Path_single
 *@	function: Compute the shortest path to the source point
 *@	input		: the number of full_grain u want to pick and the source point
 *@	output	: none
 ***********************************************************************/	
void Path_single(int num,int src)
{
	 // Define and initialize variables
	int choose=2*num;          // Choice variable for alternating between empty and full grains
	int source=0;
	source=src;
  for(int i=0;i<2*num;i++)
	{	
		//initialization
		Grain[source-1].total_distance += Grain[src-1].init_distance;
		dijkstra(src);
	  color_grouping();
		sort_distance();

		// alternately selected
    if(choose % 2 == 0)		
		{
			Grain[source-1].total_order[i]=Empty_Grain[0].number;      
			Grain[Grain[source-1].total_order[i]-1].status = TAKEN;    //refresh the status of Grain
			src=Empty_Grain[0].number;                                 //refresh the source   
		  Grain[source-1].total_distance += Empty_Grain[0].distance; //refresh the total_distance	
		}
		else
   {
		 Grain[source-1].total_order[i]=Full_Grain[0].number;
		 Grain[Grain[source-1].total_order[i]-1].status = TAKEN;
		 src =Full_Grain[0].number;
		 Grain[source-1].total_distance += Full_Grain[0].distance;
	 }
	 
	 Empty_index=0;
	 Full_index=0;
	 choose --;	
	}
}

/**********************************************************************
 *@ 	name		: Item Path_all
 *@	function: Find the shortest path to all points
 *@	input		: the number of full_grain u want
 *@	output	: Each piece of information about the starting 
 *@           point of the shortest path (stored in a structure)
 ***********************************************************************/	
struct Item Path_all(int num)
{
	 int Full_num[6];
	int index=0;
	
	//Make sure the first one takes the empty valley.
	 for(int i=0;i < 12; i++)
	{
		if(Grain_init[i].color==PURPLE)
		{
			Full_num[index++]=Grain_init[i].number;
		}
		
	}
   for(int i=0;i < 6; i++)
	{
		//Initialize Compute Structure
		for (int i = 0; i < 12; i++) 
		{
        Grain[i] = Grain_init[i];
    }
		
		//Caculation
	  Path_single(num,Full_num[i]);
		Total_distance[i] = Grain[Full_num[i]-1];
	}
	 
	 //Sort total distance of all Grain
   int size = sizeof(Total_distance) / sizeof(Total_distance[0]);
   qsort(Total_distance, size, sizeof(struct Item), compare_total);
	
	
	 //return the shortest distance grain
	return Total_distance[0];
}