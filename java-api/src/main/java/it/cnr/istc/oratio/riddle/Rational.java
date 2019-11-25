/*
 * Copyright (C) 2018 Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.cnr.istc.oratio.riddle;

/**
 *
 * @author Riccardo De Benedictis <riccardo.debenedictis@istc.cnr.it>
 */
public class Rational extends Number implements Comparable<Rational> {

    private static final long serialVersionUID = 1L;
    public static final Rational POSITIVE_INFINITY = new Rational(1, 0);
    public static final Rational NEGATIVE_INFINITY = new Rational(-1, 0);
    long num; // the numerator..
    long den; // the denominator..

    public Rational() {
        this.num = 0;
        this.den = 1;
    }

    public Rational(final Rational r) {
        this.num = r.num;
        this.den = r.den;
    }

    public Rational(final long n) {
        this.num = n;
        this.den = 1;
    }

    public Rational(final long n, final long d) {
        this.num = n;
        this.den = d;
        normalize();
    }

    public long getNumerator() {
        return num;
    }

    public long getDenominator() {
        return den;
    }

    public boolean isPositive() {
        return num > 0;
    }

    public boolean isNegative() {
        return num < 0;
    }

    public boolean isInfinite() {
        return den == 0;
    }

    public boolean isPositiveInfinite() {
        return isPositive() && isInfinite();
    }

    public boolean isNegativeInfinite() {
        return isNegative() && isInfinite();
    }

    public boolean neq(final Rational rhs) {
        return num != rhs.num || den != rhs.den;
    }

    public boolean lt(final Rational rhs) {
        return (den == 0 && rhs.den == 0) ? num < rhs.num : num * rhs.den < den * rhs.num;
    }

    public boolean leq(final Rational rhs) {
        return num * rhs.den <= den * rhs.num;
    }

    public boolean eq(final Rational rhs) {
        return num == rhs.num && den == rhs.den;
    }

    public boolean geq(final Rational rhs) {
        return num * rhs.den >= den * rhs.num;
    }

    public boolean gt(final Rational rhs) {
        return (den == 0 && rhs.den == 0) ? num > rhs.num : num * rhs.den > den * rhs.num;
    }

    public boolean neq(final long rhs) {
        return num != rhs || den != 1;
    }

    public boolean lt(final long rhs) {
        return num < den * rhs;
    }

    public boolean leq(final long rhs) {
        return num <= den * rhs;
    }

    public boolean eq(final long rhs) {
        return num == rhs && den == 1;
    }

    public boolean geq(final long rhs) {
        return num >= den * rhs;
    }

    public boolean gt(final long rhs) {
        return num > den * rhs;
    }

    @Override
    public int compareTo(Rational o) {
        if (eq(o)) {
            return 0;
        }
        if (lt(o)) {
            return -1;
        }
        if (gt(o)) {
            return 1;
        }
        throw new AssertionError();
    }

    public void add(final Rational rhs) {
        assert den != 0 || rhs.den != 0 || num == rhs.num; // inf + -inf or -inf + inf..

        // special cases..
        if (num == 0 || rhs.isInfinite()) {
            num = rhs.num;
            den = rhs.den;
            return;
        }
        if (rhs.num == 0 || isInfinite()) {
            return;
        }
        if (den == 1 && rhs.den == 1) {
            num += rhs.num;
            return;
        }

        long f = gcd(num, rhs.num);
        long g = gcd(den, rhs.den);

        num = (num / f) * (rhs.den / g) + (rhs.num / f) * (den / g);
        den = lcm(den, rhs.den);
        normalize();
        num *= f;
    }

    public void sub(final Rational rhs) {
        add(rhs.minus());
    }

    public void mult(final Rational rhs) {
        assert num != 0 || rhs.den != 0; // 0*inf..
        assert den != 0 || rhs.num != 0; // inf*0..

        // special cases..
        if (rhs.eq(1)) {
            return;
        }
        if (eq(1)) {
            num = rhs.num;
            den = rhs.den;
            return;
        }
        if (den == 1 && rhs.den == 1) {
            num *= rhs.num;
            return;
        }
        if (isInfinite() || rhs.isInfinite()) {
            num = ((num >= 0 && rhs.num >= 0) || (num <= 0 && rhs.num <= 0)) ? 1 : -1;
            den = 0;
            return;
        }

        Rational c = new Rational(num, rhs.den);
        Rational d = new Rational(rhs.num, den);

        num = c.num * d.num;
        den = c.den * d.den;
        normalize();
    }

    public void div(final Rational rhs) {
        Rational rec = new Rational();
        rec.num = rhs.den;
        rec.den = rhs.num;
        if (rhs.num >= 0) {
            rec.num = rhs.den;
            rec.den = rhs.num;
        } else {
            rec.num = -rhs.den;
            rec.den = -rhs.num;
        }
        Rational.this.mult(rec);
    }

    public void add(final long rhs) {
        // special cases..
        if (num == 0) {
            num = rhs;
            return;
        }
        if (rhs == 0 || isInfinite()) {
            return;
        }
        if (den == 1) {
            num += rhs;
            return;
        }

        num += rhs * den;
    }

    public void sub(final long rhs) {
        add(-rhs);
    }

    public void mult(final long rhs) {
        assert den != 0 || rhs != 0; // inf*0..

        // special cases..
        if (rhs == 1) {
            return;
        }
        if (eq(1)) {
            num = rhs;
            return;
        }
        if (isInfinite()) {
            num = ((num >= 0 && rhs >= 0) || (num <= 0 && rhs <= 0)) ? 1 : -1;
            return;
        }

        num *= rhs;
        if (den != 1) {
            normalize();
        }
    }

    public void div(final long rhs) {
        Rational rec = new Rational();
        rec.num = 1;
        rec.den = rhs;
        if (rhs >= 0) {
            rec.num = 1;
            rec.den = rhs;
        } else {
            rec.num = -1;
            rec.den = -rhs;
        }
        Rational.this.mult(rec);
    }

    public Rational plus(final Rational rhs) {
        assert den != 0 || rhs.den != 0 || num == rhs.num; // inf + -inf or -inf + inf..

        // special cases..
        if (num == 0 || rhs.isInfinite()) {
            return rhs;
        }
        if (rhs.num == 0 || isInfinite()) {
            return this;
        }
        if (den == 1 && rhs.den == 1) {
            return new Rational(num + rhs.num);
        }

        long f = gcd(num, rhs.num);
        long g = gcd(den, rhs.den);

        Rational res = new Rational((num / f) * (rhs.den / g) + (rhs.num / f) * (den / g), lcm(den, rhs.den));
        res.num *= f;
        return res;
    }

    public Rational minus(final Rational rhs) {
        return plus(rhs.minus());
    }

    public Rational times(final Rational rhs) {
        assert num != 0 || rhs.den != 0; // 0*inf..
        assert den != 0 || rhs.num != 0; // inf*0..

        // special cases..
        if (rhs.eq(1)) {
            return this;
        }
        if (eq(1)) {
            return rhs;
        }
        if (den == 1 && rhs.den == 1) {
            return new Rational(num * rhs.num);
        }
        if (isInfinite() || rhs.isInfinite()) {
            return ((num >= 0 && rhs.num >= 0) || (num <= 0 && rhs.num <= 0)) ? POSITIVE_INFINITY : NEGATIVE_INFINITY;
        }

        Rational c = new Rational(num, rhs.den);
        Rational d = new Rational(rhs.num, den);
        return new Rational(c.num * d.num, c.den * d.den);
    }

    public Rational divide(final Rational rhs) {
        Rational rec = new Rational();
        if (rhs.num >= 0) {
            rec.num = rhs.den;
            rec.den = rhs.num;
        } else {
            rec.num = -rhs.den;
            rec.den = -rhs.num;
        }
        return times(rec);
    }

    public Rational plus(final long rhs) {
        // special cases..
        if (num == 0) {
            return new Rational(rhs);
        }
        if (rhs == 0 || isInfinite()) {
            return this;
        }
        if (den == 1) {
            return new Rational(num + rhs);
        }

        Rational res = new Rational();
        res.num = num + rhs * den;
        res.den = den;
        return res;
    }

    public Rational minus(final long rhs) {
        return plus(-rhs);
    }

    public Rational times(final long rhs) {
        assert den != 0 || rhs != 0; // inf*0..

        // special cases..
        if (rhs == 1) {
            return this;
        }
        if (eq(1)) {
            return new Rational(rhs);
        }
        if (den == 1) {
            return new Rational(num * rhs);
        }
        if (isInfinite()) {
            return ((num >= 0 && rhs >= 0) || (num <= 0 && rhs <= 0)) ? POSITIVE_INFINITY : NEGATIVE_INFINITY;
        }

        return new Rational(num * rhs, den);
    }

    public Rational divide(final long rhs) {
        Rational rec = new Rational();
        rec.num = 1;
        rec.den = rhs;
        if (rhs >= 0) {
            rec.num = 1;
            rec.den = rhs;
        } else {
            rec.num = -1;
            rec.den = -rhs;
        }
        return times(rec);
    }

    public Rational minus() {
        Rational neg = new Rational();
        neg.num = -num;
        neg.den = den;
        return neg;
    }

    @Override
    public int hashCode() {
        int hash = 5;
        hash = 29 * hash + (int) (this.num ^ (this.num >>> 32));
        hash = 29 * hash + (int) (this.den ^ (this.den >>> 32));
        return hash;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null) {
            return false;
        }
        if (getClass() != obj.getClass()) {
            return false;
        }
        final Rational other = (Rational) obj;
        return this.num == other.num && this.den == other.den;
    }

    @Override
    public String toString() {
        if (den == 0) {
            return num > 0 ? "+inf" : "-inf";
        } else if (den == 1) {
            return Long.toString(num);
        } else {
            return num + "/" + den;
        }
    }

    private void normalize() {
        if (den != 1) {
            long c_gcd = gcd(num, den);
            if (den < 0) {
                c_gcd = -c_gcd;
            }
            num /= c_gcd;
            den /= c_gcd;
        }
        if (den < 0) {
            den = -den;
            num = -num;
        }
    }

    @Override
    public int intValue() {
        return (int) (num / den);
    }

    @Override
    public long longValue() {
        return num / den;
    }

    @Override
    public float floatValue() {
        return (float) num / den;
    }

    @Override
    public double doubleValue() {
        return (double) num / den;
    }

    /**
     * Computes the greatest common divisor.
     *
     * @param u
     * @param v
     * @return the greatest common divisor.
     */
    private static long gcd(long u, long v) {
        if (u < 0) {
            u = -u;
        }
        if (v < 0) {
            v = -v;
        }
        while (v != 0) {
            long r = u % v;
            u = v;
            v = r;
        }
        return u;
    }

    /**
     * Computes the least common multiplier.
     *
     * @param u
     * @param v
     * @return the least common multiplier.
     */
    private static long lcm(long u, long v) {
        if (u < 0) {
            u = -u;
        }
        if (v < 0) {
            v = -v;
        }
        return u * (v / gcd(u, v));
    }
}
