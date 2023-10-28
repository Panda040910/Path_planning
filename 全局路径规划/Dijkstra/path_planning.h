#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H
#include <stdbool.h>
#include "main.h"

extern uint8_t V;
enum Status {
    NOT_TAKEN,
    TAKEN
};
enum BallColor {
    Full,
    PURPLE
};

struct Item {
   uint8_t number;                // Number of each grain  
   double distance;           // distance for caculation
	 enum Status status;        // state taken or not_taken
	 enum BallColor color;      // full or none 
	 uint8_t init_distance;         // Distance from starting point
	 uint8_t total_distance;        // shortest distance of path
	 uint8_t total_order[12];       // shortest path 
};

/******************************************************************
 *@ 	name		: Get init information
 *@	function	: Get visual data 
 *@	input		: none
 *@	output	: none
 *******************************************************************/
int Givecolor();



/******************************************************************
 *@ 	name		: caculation
 *@	function	: Find the shortest path.
 *@	input		: none
 *@	output	: none
 *******************************************************************/
int compare_single(const void *a, const void *b);
int compare_total(const void *a, const void *b);
int minDistance(int dist[], bool sptSet[]);
void printSolution(int dist[],int V); 
void dijkstra(int src);
void color_grouping();
void sort_distance();



/******************************************************************
 *@ 	name		:Planning 
 *@	function	:Output the result 
 *@	input		: none
 *@	output	: none
 *******************************************************************/
void Path_single(int num,int src);
struct Item Path_all(int num);


#endif