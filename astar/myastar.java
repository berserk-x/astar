// skeleton program for you to complete...
import java.util.*;

// don't change name of your class (referred to in pathfinder.java)
public class myastar extends astar_base
{
    public myastar(int r, int c)
    { super(r,c); }

    @Override
    public void customize() {
       ////// Things you can do here...
        //setcosts(2,0,1,10); // cost of land, desert, fire, water
        //pathfinder.gap = 15; // change size of graphical hexgagons
        //pathfinder.yoff = 20; // graphical top margin adjustment
        //pathfinder.delaytime = 300; //change animation speed
        //setRandFactor(0.13); // increase amount of water/fire
    }
    public static void main(String[] av) {
        pathfinder.main(av);
    }//main

    ///// The following function just searches randomly.
    ///// You must replace it with an implementation of Algorithm A*:
    @Override
    public Optional<coord> search(int sy, int sx, int ty, int tx)  {
        HashedHeap.set_initial_capacity(ROWS*COLS);
        var Frontier = new HashedHeap<Integer, coord>((a, b) -> a.compareTo(b));
        boolean[][] Interior = new boolean[ROWS][COLS];

        coord start = new coord(sy, sx);
        start.set_known_cost(costof[Map[sy][sx]]);
        start.add_estimated_cost(hexdist(sy, sx, ty, tx));
        Frontier.set(hash_key(sy, sx), start);

        while (Frontier.size() > 0) {
                coord current = Frontier.pop().get().val();

                    if (current.y == ty && current.x == tx) {
                        return Optional.of(current);
                    }

                    Frontier.pop(); 
                    Interior[current.y][current.x] = true;

                    for (int dir = 0; dir < 6; dir++) {
                        int ny = current.y + DY[dir];
                        int nx = current.x + DX[current.y % 2][dir];

                        if (ny < 0 || ny >= ROWS || nx < 0 || nx >= COLS || Interior[ny][nx]) {
                            continue;
                        }

                        int terrainCost = costof[Map[ny][nx]];
                        if (terrainCost == -1) continue;

                        int knownCost = current.known_cost() + terrainCost;
                        int estimatedCost = hexdist(ny, nx, ty, tx);
                        int totalCost = knownCost + estimatedCost;

                        coord neighbor = make_neighbor(current, current.y, current.x, ny, nx);
                        neighbor.set_known_cost(knownCost);
                        neighbor.add_estimated_cost(estimatedCost);

                        int neighborKey = hash_key(ny, nx);
                        Frontier.modify_or(neighborKey, old -> {
                            if (old.known_cost() > knownCost) {
                                old.set_parent(current);
                                old.set_known_cost(knownCost);
                                old.add_estimated_cost(estimatedCost - old.total_cost());
                            }
                            return old;
                        }, neighbor);
                    }
                }

                return Optional.empty();
            }
}//myastar

/*  More Help:
  
   Algorithm A* is a modification of Dijkstra's Algorithm with the following
   differences:
  
   1. Instead of finding shortest paths form the source to all destinations,
      A* is focused on finding the best path from the source one specific
      destination.  The source is given by coordinates sy,sx and the 
      destination by ty,tx.  The algorithm terminates when the destination
      node (coord object) has been removed from the frontier and inserted 
      into the interior.

   2. The cost of each node in the search tree is a sum of two elements:
        
             known_cost_from_src  +  estimated_cost_to_dst

      Where the known_cost_from_src is the same as the cost measure used
      in Dijkstra's algorithm.  The estimated_cost_to_dst is a heuristic
      estimate of the remaining cost from the current node to the
      destination.  Futhermore, the estimate must be conservative: it
      cannot exceed the actual cost.  For example, when finding the best
      route to drive to a certain destination, the estimate can be the
      straight-line distance to the destination, which is guaranteed to 
      not overestimate the actual distance.  Under this restriction, A*
      is guaranteed to also find the optimal path to target.  

      The `hexdist` function in the astar_base superclass provides a
      conservative estimate of the remaining cost to reach the target.

   You can also think of Dijkstra's algorithm as Algorithm A* with a 
   heuristic estimate of zero. 
   
   As for data structures, I suggest you use my HashedHeap for the 
   Frontier.  You can create an instance of it as follows:

     HashedHeap.set_initial_capacity(ROWS*COLS); //never have to resize/rehash
     var Frontier = new HashedHeap<Integer,coord>((x,y)->y.compareTo(x));

   This creates a minheap.  The keys of the HashedHeap are integers
   computed from the y,x coordinates of a coord object, specifically
   y*COLS+x: the one-dimensional representation of the 2D coordinate.
   If given initial capacity ROWS*COLS, this guarantees that there
   will be no hash-collisions, and no need to ever resize the
   structure.

   For the Interior, you can use a hashmap but I would suggest just using 
   a 2D array of booleans:

     boolean[][] Interior = new boolean[ROWS][COLS];

   Initially, each Interior[y][x]==false, until you set it otherwise.

   Refer to my notes on Dijkstra's Algorithm for pseudocode.
*/
