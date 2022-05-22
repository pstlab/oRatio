package it.cnr.istc.pst.utils;

import java.lang.reflect.Array;
import java.util.Iterator;

public class CartesianProductGenerator<T> implements Iterable<T[]> {

    private final T[][] elements;
    private final int[] productIndices;
    private final int size;

    @SuppressWarnings("unchecked")
    public CartesianProductGenerator(T[]... elements) {
        this.elements = elements;
        int c_size = 1;
        for (T[] c_elements : elements) {
            c_size *= c_elements.length;
        }
        this.productIndices = new int[elements.length];
        this.size = c_size;
    }

    public long getSize() {
        return size;
    }

    @Override
    @SuppressWarnings("unchecked")
    public Iterator<T[]> iterator() {
        return new Iterator<T[]>() {
            private int cursor = 0;

            @Override
            public boolean hasNext() {
                return cursor < size;
            }

            @Override
            public T[] next() {
                int j = 1;
                for (int i = elements.length - 1; i >= 0; i--) {
                    productIndices[i] = (cursor / j) % elements[i].length;
                    j *= elements[i].length;
                }
                T[] ret = (T[]) Array.newInstance(elements[0].getClass().getComponentType(), elements.length);
                for (int i = 0; i < ret.length; i++) {
                    ret[i] = elements[i][productIndices[i]];
                }
                cursor++;
                return ret;
            }
        };
    }
}
