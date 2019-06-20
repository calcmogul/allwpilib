package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N3 extends Num implements Nat<N3> {
    private N3() {
    }

    @Override
    public int getNum() {
        return 3;
    }

    public static final N3 N3 = new N3();
}