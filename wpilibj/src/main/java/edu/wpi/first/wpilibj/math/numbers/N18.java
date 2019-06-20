package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N18 extends Num implements Nat<N18> {
    private N18() {
    }

    @Override
    public int getNum() {
        return 18;
    }

    public static final N18 N18 = new N18();
}