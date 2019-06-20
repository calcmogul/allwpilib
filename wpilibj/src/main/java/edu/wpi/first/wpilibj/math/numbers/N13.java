package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N13 extends Num implements Nat<N13> {
    private N13() {
    }

    @Override
    public int getNum() {
        return 13;
    }

    public static final N13 N13 = new N13();
}