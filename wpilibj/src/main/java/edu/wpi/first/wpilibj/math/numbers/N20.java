/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
