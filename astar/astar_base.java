import java.util.Optional;

/**
* Superclass of {@link myastar}, which is to be student-defined.  This class
* contains a 2D array {@link Map}. and a {@link costof} vector defining the
* cost of each type of terrain.  The main purpose of this class is to generate
* a random Map with different types of terrain (in its constructor).  It
* offers some convenient functions that will make writing the subclass
* easier.
*/
public class astar_base
{
    public static final int OPEN = 0;   // don't confuse with cost
    public static final int DESERT = 1;  
    public static final int FIRE = 2;
    public static final int WATER = 3;

    /**
     * constant for direction West, indexes DY,DX[y%2]
     */
    public static final int W = Hexagon.West;
    /**
     * constant for direction East, indexes DY,DX[y%2]
     */    
    public static final int E = Hexagon.East;
    public static final int NW = Hexagon.NorthWest;
    public static final int SW = Hexagon.SouthWest;
    public static final int NE = Hexagon.NorthEast;
    public static final int SE = Hexagon.SouthEast;
    /**
     * The y-coordinate displacement vector for each of the six possible
     * neighbors of a hexagon stored in a 2D array.
     */
    public static int[] DY = Hexagon.DY;
    /**
     * The x-coordinate displacement vector for each of the six possible
     * neighbors of a hexagon stored in a 2D array.  The x-displacement
     * is dependent on whether the y coordinate is even or odd, and is
     * thus a 2D array.  For example, to calculate the Northwest neighbor
     * of coordinates y,x, the y coordinate of the neighbor is y+DY[NW]
     * and the x coordinate of the neighbor is x+DX[y%2][NW].
     */    
    public static int[][] DX = Hexagon.DX;
    protected static double RandFactor = 0.10; // must be between 0 and .166
                                            // affects amount of water/fire
    /**
     * This function sets {@link RandFactor} and controls the random map
     * generation, It must be set to a value between
     * 0 and 0.166, exclusively.
     */
    public static void setRandFactor(double x) // affects map generation
    {
	if (x>0 && x<0.166) RandFactor = x;
    }

    static boolean genmap = true;
    protected int[] costof= {1,0,8000,8};  // cost vector of each terrain

    /**
     * Sets the {@link costof} vector that defines the cost of each type
     * of terrain, in order of <b>land (open), desert, fire (volcano) and water</b>.
     * Deserts are not currently generated and is reserved for future expansion.
     * A negative value indicates that the terrain type is impassible,
     * implying that there may be no path to the desired destination.
     */
    public void setcosts(int l, int d, int f, int w)
    {
	//	if (l<0 || d<0 || f<0 || w<0) return;
	int[] vec = {l,d,f,w};
	costof = vec;
    }

    /**
     * The 2D array that's rendered by hexagons, representing a map and
     * a graph where each node has up to six neighbors.  The values in
     * the matrix indicates the terrain type: land (open), desert,
     * fire or water.  This protected value is directly accessible in the
     * {@link myastar} subclass to be written by students.
     */
    protected int Map[][];  // the map itself, value=terrain type
    /**
     *  The number of rows in {@link Map}
     */
    protected int ROWS;
    /**
     *  The number of columns in {@link Map}
     */
    protected int COLS; 

    /**
     * This function currently does nothing, but is called by the constructor
     * before it does anything else. It can be overriden to customize a
     * variety of properties.
     */
    public void customize() {} // called at start of constructor (override it)

    /**
     * This convenient function transforms a y,x coordinate into a
     * one-dimensional coordinate.  It can be called from a subclass that
     * uses a hashmap or hashset that hashes integers.
     */
    public int hash_key(int y, int x) {
        return y*COLS + x;
    }
    
    astar_base(int r0, int c0)  // typically 32x44
    {
	customize();
	if (!genmap) return;	 // set map externally
	Map = new int[r0][c0];
	ROWS=r0;  COLS=c0;
        coord.set_columns(COLS);
	// generate random map  (initially all OPEN)
	int GENS = 10;  // number of generations
	double p, r;  // for random probability calculation
	int generation;  int i, j;
        // generate random map
	for(generation=0;generation<GENS;generation++)
	    {
	     for(i=0;i<ROWS;i++) 
		 for(j=0;j<COLS;j++)
	     {
	       p = 0.004; // base probability factor
	       // calculate probability of water based on surrounding cells
	       for(int k=0;k<6;k++)
		   {
		       int ni = i + DY[k], nj = j + DX[i%2][k];
		       if (ni>=0 && ni<ROWS && nj>=0 && nj<COLS && Map[ni][nj]==WATER) p+= RandFactor;
		   }
	       r = Math.random();
	       if (r<=p) Map[i][j] = WATER;
	     } // for each cell i, j
	    } // for each generation

	for(generation=0;generation<GENS-1;generation++)
	    {
	     for(i=0;i<ROWS;i++) 
		 for(j=0;j<COLS;j++)
	     {
	       p = 0.004; // base probability factor
	       // calculate probability of fire based on surrounding cells
	       for(int k=0;k<6;k++)
		   {
		       int ni = i + DY[k], nj = j + DX[i%2][k];
		       if (ni>=0 && ni<ROWS && nj>=0 && nj<COLS && Map[ni][nj]==FIRE) p+= RandFactor;
		   }
	       r = Math.random();
	       if (r<=p && Map[i][j]==0) Map[i][j] = FIRE;
	     } // for each cell i, j
	    } // for each generation

    } //constructor

    /** Determines the distance between hex coordinates y1,x1 and y2,x2.
     * Note: this function does not give exact distance but does not overestimate
     * it, so it is admissible for algorithm A*
     */
    public static int hexdist(int y1, int x1, int y2, int x2)
    {
        int dx = x1-x2, dy = y1-y2;
	int dd = Math.abs(dx - dy);
	dy = Math.abs(dy);
	int max = Math.abs(dx);
	if (dy>max) max = dy;
	if (dd>max) max = dd;
	return max;
    }

    // you need to override the search method in astar_base.java

    /**
     * Students can optionally call this function from their myastar subclass.
     * It will create a coord object for coordinates y,x
     * adjacent to coord parent (which will become its parent).
     * It sets known distance of the coord object to be the known distance
     * of the parent plus {@code costof[M[y][x]])}, and adds the heuristic
     * estimate by calling the {@link hexdist} function.  It does not,
     * however, check if any coordinates are out of bounds or passable.
    */
    public coord make_neighbor(coord parent, int y, int x, int ty, int tx)
    {
	int coordcost = costof[Map[y][x]];
	int estimate = hexdist(y,x,ty,tx);
	coord node = new coord(y,x);
        node.set_known_cost(parent.known_cost() + coordcost);
        node.add_estimated_cost(estimate);
        node.set_parent(parent);
	return node;
    }// makeneighbor


    /**
     * This function currently returns {@code Optional.empty()}, indicating
     * that no path is found.  Students must override this function in their
     * {@link myastar} subclass to implement Algorithm A*.  It must
     * return a {@link coord} object with coordinates ty,tx (target) and with
     * its parent pointer set to eventually lead back to coordinates
     * with sy,sx (source).
     */
    public Optional<coord> search(int sy, int sx, int ty, int tx)
    {
	return Optional.empty(); // means no path found
    }// this search can't find anything, please @Override

}//astar_base
