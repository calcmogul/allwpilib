package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N12 extends Num implements Nat<N12> {
    private N12() {
    }

    @Override
    public int getNum() {
        return 12;
    }

    public static final N12 N12 = new N12();
}