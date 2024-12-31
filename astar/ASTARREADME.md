###		**CSC17  Final Programming Assignment**

You are to implement the A\* search algorithm on a hexagonal grid as
described in class.  I've provided you with enough code so you can focus
on the basic algorithm.  These instructions assume that you were present
during the classroom presentation, understand Dijkstra's Algorithm as
well as Algorithm A\*.

------------------------------------------------------------------

First, download all relevant files in
[astar.zip](https://cs.hofstra.edu/~cscccl/csc17/astar.zip), including
gifs and jpeg images and `unzip` into an astar folder. Compile
everything together with `javac *.java`

The **only file you should edit is the skeleton myastar.java**.  You
must override the `search` function inside this class, but first,
study carefully the documentation for the 
**[coord.java](https://cs.hofstra.edu/~cscccl/csc17/astar/doc/coord.html)**
,
**[astar_base.java](https://cs.hofstra.edu/~cscccl/csc17/astar/doc/astar_base.html)**,
and
**[HashedHeap.java](https://cs.hofstra.edu/~cscccl/csc17/astar/doc/HashedHeap.html)**
programs.  

The source code for these classes are inside the astar folder but
you should only have to study the documentation.
Try not to refer to the non-public items in the class, even if you can
(this makes things easier - less for you to worry about).

In the **astar_base.java** class, code has already been written (in
the constructor) to generate a random map containing land (grassy
texture), water and fire.  You shouldn't have to touch this unless you
add another type of terrain (see optional challenges).
   
   The following CRITICAL structures are declared in the `astar_base` class
   that you will inherit in your myastar subclass:

   ```
      protected int[][] Map;  // the map with value=terrain type
      protected int ROWS, COLS; // size of the Map matrix
      public static int[] DY = Hexagon.DY;  // y coordinates of neighbors
      public static int[][] DX = Hexagon.DX; // x coordinates of neighbors
   ```
   
   Although the map is a 2D array, each array cell is treated as a
   hexagon which means that it has six immediate neighbors.  The
   Hexagon class (in Hexagon.java) extends java.awt.Polygon.  The
   neighbors are designated in order west, northwest, northeast, east,
   southeast, and southwest.  The vector DY defines the row coordinate
   displacement: so given current row coordinate y, `y+DY[5]` will give
   the row coordinate of its southwest neighbor.  The column (DX)
   displacement is more complicated because it's different for even
   and odd rows.  Therefore DX is defined as a 2x6 2D array:
   the first row is the x-displacement if the current
   y coordinate is 0 or even, and the second row is the x-displacement
   if the current y coordinate is odd.  Thus, if the current row
   is y and the current column is x, then `x+DX[y%2][5]` will give the
   column coordinate of the southwest neighbor.  **As usual, you must
   always check if y,x stay within bounds**.

   ```
      protected int[] costof = {1,0,8000,8};  // cost vector
      public void setcosts(int l, int d, int f, int w) // call to change costs
   ```
   
   The costof vector indicates the cost of each type of terrain.  This
   means, for example, that the cost of land is 1 and the cost of
   water is 8. The cost of going to a map coordinate y,x is `costof[Map[y][x]]`.
   *A cost of -1 means that the terrain type is impassable.*

   ```
      public static int hexdist(int y1, int x1, int y2, int x2)
   ```

   This function conservatively estimates the shortest distance between
   two coordinates in the hexagonal grid. This function can 
   calculate the heuristic estimate that's part of algorithm A*.

   ```
      public coord makeneighbor(coord parent, int y, int x, int ty, int tx)
   ```
   This is a convenient function that you might find useful, but you need
   to figure out what it's doing and how to use it.


#### **What You Need to Write**

***You must write a 'class myastar extends astar_base'.  A template has
been created in myastar.java.  You must call the subclass "myastar"
because that's referred to in patherfinder.java. ALL your code must
be in this class.  You need to
```
   @Override
   public Optional<coord> search(int sy, int sx, int ty, int tx)
```
  This method should find an optimal path from the source coordinates
sy,sx to the destination coordinates ty,tx.  You need to construct a
coord object with y,x=ty,tx, and with the parent pointer properly set
so we can trace a path from ty,tx back to sy,sx.  In the skeleton myastar,
the search function searches for a solution randomly.  You must replace 
that code with a proper implementation of Algorithm A\*.  

In writing the code, you may call the `.get()` function on Optionals under
the following circumstances:
```
   while (myheap.size()>0) {
     var key_val_pair = myheap.pop().get();
     ...
   }
```
The Java language contains restrictions, such as not being able to change 
a local variable inside a lambda expression, that makes it too hard to
always stay pure with respect to monadic error handling.

--------------------------------


#### Running the Program


* Run with  java myastar (or java pathfinder):  
   The program will generate a new map, run your search function and 
   save map and starting positions in myastar.run

* Run with  java myastar myastar.run
   will now load map, starting positions and cost vector from specified file.
   You can rename myastar.run to something else if you wish to save it.

* Run with  java myastar saved1.run nocost
   will now load map and positions, but NOT the cost vector from a file. 
   This way you can see how different terrain costs will affect the same map.

* Run with  java myastar download "your name, 702111111" will download
   a map configuration from the professor's server, and upload the
   coord path you create back to the server. Place in one string
   argument both your name and Hofstra ID. The server will reply with
   a message indicating if your path is correct or has some problem
   that needs to be fixed.  If download fails (because the server is
   not currently running or is too busy), it will revert to generating
   a new map.  map config will be saved in myastar.run regardless of
   it was downloaded or generated. Rename the file so you can test it locally.
   There are 7 different map/cost configurations on the server.

   Please note that the server is only reachable within Hofstra because of
   Hofstra's firewalls.  The server may be down during some class times.
   If you do not give accurate information (name, ID number) to the server,
   you may be blocked from further connections.

--------------------------------------------

##### Graphical Adjustments

Much of graphics code in pathfinder.java is rather fragile and will break
easily if you change it.  You should concentrate on the algorithm.  There
are a few adjustments you can make, such as the size of the graphical
hexagons (the default is 24), which will also affect the size of the
window.  There are some static variables that can be change by overriding
the `void customize()` function - follow the instructions (in comments) in
the myastar.java skeleton.

Furthermore, the main of myastar.java can be given 2 integer arguments
indicating the number of rows and columns of the matrix to generate:
```
  java myastar 20 30  -- will create a 20x30 hexagonal grid.
```
If you wish to make further changes, such as the graphical gifs, you
will have to change pathfinder.java.  You may not change any other
posted file besides pathfinder.java, and you take full responsibility
for changing it

--------------------------------------------

##### CHALLENGE:  

  Build a road.  You can use the unused terrain type (doesn't have to be
desert).  The cost of traveling on the road needs to be significantly less
than all other types of terrain, so your figure should find the road and
use it frequently.  It may be hard to generate the road randomly so just
hand code it onto the existing map (just need a straight line).  Find 
appropriate images to represent the road and the human on the road.  You 
can change pathfinder.java only if you do this part, and only change the
images that are loaded.  You shouldn't have to change astar_base: just build
the road on top of the existing map in your subclass.

##### MEGACHALLENGE:

  Build docks next to water.  Travel on water should only be possible 
between docks.

##### GIGACHALLEGE:

  Currently there is only one "player" that's trying to find the
  target.  You can have two (or more) players.  Each player will try
  to reach the same target from a different starting point and with a
  different cost vector.  This option will require major and careful
  changes to pathfinder.java.  Try to make the modifications to the program
  inside subclasses as much as reasonable.

The optional challenges may not work with the pathfinder download feature,
so don't attempt them before you have finished checking your basic program.

