#include "Astar.h"

/*Author: Muhammad Shahab 
 *Github: https://github.com/MuhamdShahab/
 *Linkedin: https://www.linkedin.com/in/muhammad-shahab-ph923165051365/
 *Suggestions accepted!!
 *For further details checkout the Readme.txt 
*/


const int COL = (600/30);
const int ROW = (150/30);

int ** arena =  getmap(1,COL,ROW); //generate first param map like 1.

int source_x = 0;
int source_y = 0;
int goal_x = 2 ;
int goal_y = 3;
int left_lim = 0;
int right_lim = 19;

int** my_obstacles(int** arr)
{
  arr = place_obstacle(arr,2,2);
  arr = place_obstacle(arr,3,2);
  arr = place_obstacle(arr,3,1);
  arr = place_obstacle(arr,2,1);
  arr = place_obstacle(arr,1,2);
  return arr;
}

int main()
{
  arena = my_obstacles(arena);//place your obstacles
  astar_result apath = Astar(source_x,source_y,goal_x,goal_y,arena); //gives a 2D array of feasible path with number of coordinates
  cout<<"A-star found the following path: "<<endl; 
  printmap(apath.getpath(),apath.getsize(),2); //prints the coordinates with getsize number of columns and 2(x and y cordinates)
  cout<<"We have to traverse the following number of coordinates: "; cout<<apath.getsize()<<endl;
}
