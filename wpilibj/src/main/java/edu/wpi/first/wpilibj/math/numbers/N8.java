package edu.wpi.first.wpilibj.math.numbers;

import edu.wpi.first.wpilibj.math.Nat;
import edu.wpi.first.wpilibj.math.Num;

public class N8 extends Num implements Nat<N8> {
    private N8() {
    }

    @Override
    public int getNum() {
        return 8;
    }

    public static final N8 N8 = new N8();
}