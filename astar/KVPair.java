/**
 * Generic key-value pairs, used by {@link HashedHeap}.
 */
public record KVPair<K,V>(K key, V val) {
    @Override
    public String toString() { return key+":"+val; }
}
    
