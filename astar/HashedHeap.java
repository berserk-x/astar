import java.util.Comparator;
import java.util.function.Function;
import java.util.stream.Stream;
import java.util.Optional;

/**
* <h2> Hashed Priority Heap Data Structure </h2>
*
* A hashed heap combines the benefits of a hashmap and a binary heap:
* keys are hashed and values are ordered (Comparable).  This compensates
* for the fact that searching for a value in a heap is O(n).
* The internal representation uses two arrays, keys and vals.  The keys
* array implements a closed-hash map with linear probing.  The vals array
* implements a binary heap.  Each key is mapped to the index in the vals
* array where the corresponding value is stored.  Each value is also stored
* along with the index in the keys array of the corresponding key.  Thus
* when values are swapped during a heap operation, the key-index information can
* be quickly updated (no need for hashing/rehashing).  The position
* of keys in the keys array do not change until the entire structure is
* resized to twice the capacity.
*
* As a result, the major operations on the structure will have a <b>practical
* cost</b> of either O(1) or O(log n).  By "practical cost" we mean the average,
* amoritized running time that discount worst-case scenarios for hashing.
*
* Another benefit of this arrangement is improved iterator/stream performance,
* because values are stored contiguously in the vals array along with the
* indices of the their keys.
*
* Some of the operations on this structure return key-value pairs of the
* following type:
* <pre>
* {@code
*  public record KVPair<K,V>(K key, V val) {
*    @Override
*    public String toString() { return key+":"+val; }
*  }
* }
* </pre>
*
* <i>Copyright (c) 2024 Chuck Liang under MIT License</i>
*
* @param <KT> The type of hashed keys
* @param <VT> The type of values with a natural ordering
*/
public class HashedHeap<KT,VT extends Comparable<? super VT>>
    implements Iterable<KVPair<KT,VT>>
{
    class KVI {
        KT key;
        int vi;
        KVI(KT k, int i) {key=k; vi=i;}
    }
    class VKI {
        VT val;
        int ki;
        VKI(VT v, int i) {val=v; ki=i;}
    }
    
    static int max_load_percentage = 75;
    static int init_cap = 1024;  // initial capacity

    /**
     *  Sets the static initial capacity to the nearest power of
     *  2 that's greater than the requested value.  For example,
     *  <b>{@code HashHeap.set_initial_capapcity(500);}</b> sets the initial
     *  capacity to 512.  The lowest initial capacity allowed is 2.  The
     *  default initial capacity is 1024.
     *  @return the initial capacity that's set.
     */
    public static int set_initial_capacity(int requested_cap) {
        init_cap = 2;
        while (init_cap < requested_cap) init_cap *= 2;
        return init_cap;
    }
    /**
     * sets the maximum load percentage allowed before the capacity of
     * the structure is doubled.  The default load percentage is 75.
     */
    public static void set_max_load_percentage(int percentage) {
        if (percentage>0 && percentage <=100)
            max_load_percentage = percentage;
    }
        
    KVI[] keys;
    VKI[] vals;
    int[] maxhashes;
    int mask;
    int size = 0;
    Comparator<VKI> cmp = (a,b) -> a.val.compareTo(b.val);

    @SuppressWarnings("unchecked")
    private HashedHeap(int cap) {
        keys = (KVI[]) new HashedHeap.KVI[cap];
        vals = (VKI[]) new HashedHeap.VKI[cap];
        maxhashes = new int[cap];
        mask = cap - 1;
    }
    /**
     * Constructor creates a <b>maxheap</b> with static initial capacity (default 1024).
     * Values are compared by their natural ordering.
     */
    public HashedHeap() { this(init_cap); }
    
    /**
     * Constructor that accepts a Comparator for the value type;
     * overrides comparison by natural ordering. For example, to
     * construct a <b>minheap</b> with the reverse natural ordering,
     * call <b>{@code new HashHeap<..,..>((x,y)->y.compareTo(x))}</b>
     */
    public HashedHeap(Comparator<VT> vcmp) {
        this(init_cap);
        if (vcmp!=null) cmp = (a,b) -> vcmp.compare(a.val,b.val);
    }

    /**
     * Returns the number of key-values pairs currently in the structure.
     */
    public int size() {return size;}

    /**
     * returns current capacity
    */
    public int current_capacity() { return keys.length; }
    /**
     * return current load percentage as an integer between 0 and 100.
     */
    public int load_percentage() {
        return size*100/keys.length;
    }

    boolean resize(int percentage) {
        int newcap = (keys.length*percentage)/100;
        if (size*100 > newcap*max_load_percentage) return false;
        var newhh = new HashedHeap<KT,VT>(newcap);
        newhh.cmp = this.cmp;
        this.stream()
            .forEach(kvp -> newhh.insert_new(kvp.key(),kvp.val()));
        this.keys = newhh.keys;
        this.vals = newhh.vals;
        this.maxhashes = newhh.maxhashes;
        this.mask = newhh.mask;
        return true;
    }

    int hash(KT key) {
        return Math.abs(key.hashCode()) & mask;
    }

    //returns index in keys where inserted/set
    private int mod_point;
    // returns previous value

    // mode 0: insert or change
    // mode 1: get
    // mode 2: remove
    Optional<VT> find_and(KT key,
                          Optional<Function<Optional<VT>,? extends VT>> modf,
                          byte mode) {
        mod_point = -1;
        Optional<VT> answer = Optional.empty();
        if (key==null) return answer;
        int h0 = hash(key);
        int h = h0;
        int hashes = 1;
        int reuse = -1;
        int valsi = -1; // location in vals (of new or exiting value)
        while (true) {
            if (keys[h]==null) {
                if (hashes < maxhashes[h0]) {
                    if (reuse<0) reuse = h;
                    h = (h+1)&mask;
                    hashes += 1;
                }
                else { // key does not exist
                    if (mode>0) return answer;  // get/remove
                    valsi = size; // about to insert new kv pair
                    break;
                }
            }
            else if (key.equals(keys[h].key)) { // change/get/remove
                valsi = keys[h].vi;
                if (mode==1) return Optional.of(vals[valsi].val);
                else if (mode==2) {  //remove
                    answer = Optional.of(vals[valsi].val);
                    int ki = vals[valsi].ki;
                    if (valsi<size-1) {
                        swap(valsi, --size);
                        int up = swapup(valsi);
                        if (up==valsi) swapdown(valsi);
                    }
                    vals[size] = null;  // inform GC
                    keys[ki] = null;    // and free slot
                    return answer;
                }//remove
                break;
            }
            else {
                h = (h+1)&mask;
                hashes += 1;                
            }
        }//while
        if (valsi==size) { // add new
            Optional<VT> newvalopt =
              modf.flatMap(f -> Optional.ofNullable(f.apply(Optional.empty())));
            if (newvalopt.isEmpty()) return answer;
            size++;
            if (reuse>=0) h = reuse;
            keys[h] = new KVI(key,valsi);            
            vals[valsi] = new VKI(newvalopt.get() ,h);
            swapup(valsi);
            if (size*100 >= keys.length * max_load_percentage)
                resize(200);
        }
        else { // key found, modify
            answer = Optional.of(vals[valsi].val);
            var an2 = answer;  // finalized for lambda-expression
            Optional<VT> newvalopt =
                modf.flatMap(f -> Optional.ofNullable(f.apply(an2)));
            if (newvalopt.isPresent()) {
                vals[valsi].val = newvalopt.get();
                int up = swapup(valsi);
                if (up==valsi) swapdown(valsi);
            }
        }
        if (hashes > maxhashes[h0]) maxhashes[h0] = hashes;
        mod_point = h;
        return answer;
    } // find_and

    void insert_new(KT key, VT val) { // internal only - sure it's new
        int h0 = hash(key);           // and enough space
        int h = h0;
        int hashes = 1;
        while (keys[h]!=null) {
            h = (h+1) & mask;
            hashes += 1;
        }
        maxhashes[h0] = hashes;
        size++;
        vals[size-1] = new VKI(val,h);
        keys[h] = new KVI(key,size-1);
    }//insert_new

    /**
     * This operation applies the given modifier function to the value, if
     * it exists, that's associated with the given key. If the key is not
     * found, the function is applied to an empty optional to produce a
     * value.  In either case the end result is that the value associated with the key
     * will be the value returned by the modifier function. The position
     * of the value in the heap is automatically adjusted.  This operation
     * has practical cost O(log n).
     *
     * For example, if the value type (VT) is Integer, then calling
     * <b>{@code .and_modify(key, entry -> entry.map(x -> x+1).orElse(0))}</b>
     * will either increment the value associated with the key by one, or
     * insert a new key associated with value zero.
     * @return the previous value associated with the key, if it exists.
     * Any null key or modifier will result in nothing being done and
     * an empty optional returned. Null is prevented from entering the structure.
     */
    public Optional<VT> and_modify(KT key,Function<Optional<VT>,? extends VT> modifier)
    {
        if (modifier==null) return Optional.empty();
        else return find_and(key,Optional.of(modifier),(byte)0);
    }

    /**
     * Either modifies the existing value with the supplied function
     * or insert the key with {@code default_val} as its value.
     * Returns previous value associated with the key, if it exists.
     * This function is designed to be an alternative to
     * {@code and_modify}.
     * Calling <b>{@code .modify_or(key,modifier,default_val)}</b> is 
     * equivalent to calling
     * <b>{@code .and_modify(key,entry -> entry.map(modifier).orElse(default_val))}</b>.
     */
    public Optional<VT> modify_or(KT key,Function<? super VT,VT> modifier, VT default_val)
    {
        if (modifier==null) return Optional.empty();
        return and_modify(key,entry -> entry.map(modifier).orElse(default_val));
    }

    /**
     * Either inserts or modifies the
     * value associated with the key. Returns the previous value associated
     * with the key, if it exists. This operation is equivalent to
     * calling <b>{@code .and_modify(key, x -> val)}</b>.
     */
    public Optional<VT> set(KT key, VT val) {
        if (val==null) return Optional.empty();
        else return find_and(key,Optional.of(x -> val),(byte)0);
    }
    /**
     * Alisas for {@link set}. Because keys must be unique, this function
     * must first search for the existence of the key before adding a key-value
     * pair to the structure.  The usual insertion procedure for binary heaps,
     * which has average cost O(1), cannot be applied, although it is used
     * internally.  This operation, like {@code set}, has O(log n) cost.
     */
    public Optional<VT> push(KT key, VT val) {  // alias
        return set(key,val); 
    }
    /**
     * Returns the value associated with the key, if it exists.  This
     * operation has practical cost O(1).
     */
    public Optional<VT> get(KT key) {
        return find_and(key,Optional.empty(),(byte)1);
    }
    /**
     * Removes the key-value pair indicated by the key, returning the value
     * if it exists.  This operation has practical cost O(log n).
     */
    public Optional<VT> remove(KT key) {
        return find_and(key,Optional.empty(),(byte)2);
    }

    /**
     * Removes the highest priority key-value pair from the structure.  This
     * operation has practical cost O(log n).
     */
    public Optional<KVPair<KT,VT>> pop() {
        if (size<1) return Optional.empty();
        int ki = vals[0].ki;
        Optional<KVPair<KT,VT>> answer =
            Optional.of(new KVPair<KT,VT>(keys[ki].key, vals[0].val));
        size--;
        if (size>0) {
            swap(0,size);
            swapdown(0);
        }
        keys[ki] = null;
        vals[size] = null;
        return answer;
    }//pop

    /**
     * Returns the highest priority key-value pair without removing it.
     * This operation has worst-case cost O(1).
     */
    public Optional<KVPair<KT,VT>> peek() {
        if (size<1) return Optional.empty();
        int ki = vals[0].ki;
        return
            Optional.of(new KVPair<KT,VT>(keys[ki].key, vals[0].val));
    }

    /**
     * Empties structure of all entries. This is an O(1) operation.
     */
    public void clear() {
        var newhh = new HashedHeap<KT,VT>(keys.length);
        newhh.cmp = this.cmp;
        this.keys = newhh.keys;
        this.vals = newhh.vals;
        this.maxhashes = newhh.maxhashes;
        this.mask = newhh.mask;
        this.size = newhh.size;
    }//clear

    /**
     * Returns a stream of key-value pairs in no particular order,
     * except that the first value streamed will have highest priority.
     */
    public Stream<KVPair<KT,VT>> stream() {
        return
            java.util.Arrays.stream(vals,0,size)
            .map(vki -> new KVPair<KT,VT>(keys[vki.ki].key, vki.val));
    }
    /**
     * Implements the Iterable interface by extracting the iterator from
     * {@link stream}.
     */
    public java.util.Iterator<KVPair<KT,VT>> iterator() {
        return stream().iterator();
    }

    /**
     * Returns a <b>consuming</b> stream of key-value pairs in order of
     * priority. Running this stream is equivalent to repeatedly calling
     * {@link pop}, and will empty the structure of all keys and values.
     * For example, calling 
     * <b>{@code
     *  .priority_stream().forEach(System.out::println)
     * }</b>
     * will print all key-value pairs in order of highest to lowest priority.
     */
    public Stream<KVPair<KT,VT>> priority_stream() {
        return
            Stream.generate(() -> pop())
            .flatMap(opt -> opt.stream())
            .limit(size);
    }
    
    

    ////////// heap routines
    int left(int i) { return 2*i+1;}
    int right(int i) { return 2*i+2;}
    int parent(int i) { return (i-1)/2; }

    private void swap(int i, int k) {
        VKI temp = vals[i];
        vals[i] = vals[k];
        vals[k] = temp;
        int kii = vals[i].ki;
        int kki = temp.ki;
        keys[kii].vi = i;
        keys[kki].vi = k;
    }

    int swapup(int i) {
        int pi = parent(i);
        while (i>0 && cmp.compare(vals[pi],vals[i])<0) {
            swap(i,pi);
            i = pi;
            pi = parent(i);
        }
        return i;
    }//swapup

    int swapdown(int i) {
        int si = 0;
        while (si >=0 ) {
            si = -1;
            int lf = left(i);
            int rt = right(i);
            if (lf<size && cmp.compare(vals[i],vals[lf])<0) si = lf;
            if (rt<size && cmp.compare(vals[i],vals[rt])<0
                        && cmp.compare(vals[lf],vals[rt])<0) si = rt;
            if (si >=0) {
                swap(i,si);
                i = si;
            }
        }
        return i;
    }//swapdown


    /*
    ////// testing
    public static void main(String[] args) {
        HashedHeap.set_initial_capacity(4);
        var hh = new HashedHeap<Double,Integer>((a,b)->b.compareTo(a));
        for(int i=0;i<100;i++)
            hh.set(Math.random(), (int)(Math.random()*1000));
        for(var kvp:hh) System.out.println(kvp);
        System.out.println("................. "+hh.current_capacity());
        hh.priority_stream().forEach(System.out::println);
        System.out.println("size "+hh.size());
    }
    */
}//HashedHeap

//javadoc -d doc HashedHeap.java  KVPair.java coord.java astar_base.java
