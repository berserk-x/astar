import java.util.Optional;

/**
* Object that represent a node in the tree created by a graph-search
* algorithm.  Each coord object has an optional "parent" that points
* to the previous cord, up to the root of the tree.  Following the
* parent back to the root, identified by having an empty parent, produces
* a path from root to the end coordinate.  Each coordinate is identified
* by a y,x coordinate, and ordered by a {@link total_cost} value.
*/
public class coord implements Comparable<coord>
{
    /**
     * The y coordinate, or row coordinate this object corresponds to.
     * The y coordinate is always given before the x coordinate.
     * The value is made public for convenience.
     */
    public int y;       // coordinates this object corresponds to
    /**
     * The x coordiante, or column coordinate that this coord corresponds to.
     */
    public int x;
    int knowncost=0; // known costfrom source, excluding estimate
    private int estcost=0;  // total cost, including knowncost and estimate**
    private static int COLUMNS;
    /**
     * The parent pointer, if it exists.  This is made public for convenience.
     */
    public Optional<coord> parent = Optional.empty();

    /**
     * The constructor initializes y and x, in that order
     */
    public coord(int a, int b) {y=a; x=b; }

    /**
     * Sets the static number of columns (range of x).  
     * This function must be called before {@link hashCode} can be called.
     *
     */
    public static void set_columns(int c) {
        if (c>0) COLUMNS = c;
    }

    /**
     * returns {@code y*COLUMNS+x}, i.e. the one-dimensional version
     * of the 2D coordinate y,x, for hashing the value as a single integer.
     * This hashCode function is consistend with {@link equals}.
     */
    @Override
    public int hashCode() {
        return y*COLUMNS + x;
    }

    /**
     * compares two coord objects by their y,x coordinates for equality.
     * false is returned if the object (oc) is null or is of the wrong
     * type.  Note that this functions takes an Object in order to
     * to override the .equals method of the Object superclass.
     * This function is compatible with {@link hashCode} but not with
     * {@link compareTo}.
     */
    @Override
    public boolean equals(Object oc) // conforms to old java specs
    {
	if (oc==null || !(oc instanceof coord)) return false;
	coord c = (coord)oc;
	return (x==c.x && y==c.y);
    }
    // Note that equals and compareTo ARE NOT COMPATIBLE:

    /**
     * Orders coord objects by their {@link total_cost}, which is the
     * known cost to root (origin) plus the estimated cost to target.
     */
    public int compareTo(coord c) // compares cost (not same as .equals)
    {
	return estcost - c.estcost;
    }

    /**
     *  sets the parent pointer to the provided object.
     */
    public void set_parent(coord pcoord) {
        parent = Optional.ofNullable(pcoord);
    }
    /**
     * sets the parent to the empty optional
     */
    public void set_terminus() {
        parent = Optional.empty();
    }
    /**
     * returns true iff parent does not exist
     */
    public boolean is_terminus() { return parent.isEmpty(); }

    /**
     * Returns the cumulative cost of this node as required
     * by Dijkstra's algorithm and Algorithm A*. This is the
     * known cost to root (origin) plus the estimated cost to target.
     */
    public int total_cost() { return estcost; }
    /**
     * Adds the estimated remaining cost to target to the known cost,
     * and sets the value to be returned by {@link total_cost}.
     */
    public void add_estimated_cost(int e) { estcost = knowncost+e; }
    /**
     * Returns the current known cost to origin, which is default-initialized
     * to zero.
     */
    public int known_cost() { return knowncost; }
    /**
     * Changes the {@link known_cost} value
     */
    public void set_known_cost(int k) { knowncost=k; }
    /**
     * Adds a value to the current {@link known_cost}
     */
    public void add_known_cost(int dk) { knowncost += dk; }

} // coord
