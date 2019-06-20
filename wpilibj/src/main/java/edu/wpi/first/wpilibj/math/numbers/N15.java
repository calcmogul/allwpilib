package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N15 extends Num implements Nat<N15> {
    private N15() {
    }

    @Override
    public int getNum() {
        return 15;
    }

    public static final N15 N15 = new N15();
}