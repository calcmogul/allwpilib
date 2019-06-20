package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N16 extends Num implements Nat<N16> {
    private N16() {
    }

    @Override
    public int getNum() {
        return 16;
    }

    public static final N16 N16 = new N16();
}