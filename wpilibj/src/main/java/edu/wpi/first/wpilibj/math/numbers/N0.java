package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N0 extends Num implements Nat<N0> {
    private N0() {
    }

    @Override
    public int getNum() {
        return 0;
    }

    public static final N0 N0 = new N0();
}