package edu.wpi.first.wpilibj.math;

import edu.wpi.first.wpilibj.math.numbers.N1;

public class VecBuilder<N extends Num> extends MatBuilder<N, N1> {
    public VecBuilder(Nat<N> rows) {
        super(rows, Nat.N1());
    }
}
