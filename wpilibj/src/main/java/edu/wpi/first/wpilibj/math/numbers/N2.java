package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N2 extends Num implements Nat<N2> {
    private N2() {
    }

    @Override
    public int getNum() {
        return 2;
    }

    public static final N2 N2 = new N2();
}