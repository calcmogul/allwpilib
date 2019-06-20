package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N20 extends Num implements Nat<N20> {
    private N20() {
    }

    @Override
    public int getNum() {
        return 20;
    }

    public static final N20 N20 = new N20();
}