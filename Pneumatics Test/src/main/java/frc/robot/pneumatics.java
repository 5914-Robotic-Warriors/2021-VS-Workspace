/////////////////////////////////////////////////////////////////////
//  File:  pneumatics.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Encapsulates pneumatics operation in a class that
//            implements wpilibj features regarding pneumatic
//            operations.  Assumes use of the CTR pneumatic
//            module.
//
//  Compiler: Java via Microsoft VS
//
//  Inception Date:  1/21/2022
//
//  Revisions:
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class pneumatics {
    Compressor comp;
    DoubleSolenoid d_solenoid; // double solenoid

    // default constructor
    pneumatics() {

        comp = new Compressor(PneumaticsModuleType.CTREPCM);
        d_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0,2);

    }

    void compEnabled() {
        comp.enableDigital();
    }
    void compDisabled() {
        comp.disable();
    }


// /////////////////////////////////////////////////////////////////
// // Function:
// /////////////////////////////////////////////////////////////////
// //
// // Purpose:
// //
// // Arguments:
// //
// // Returns:
// //
// // Remarks:
// //
// /////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////

// /////////////////////////////////////////////////////////////////
// // Function:
// /////////////////////////////////////////////////////////////////
// //
// // Purpose:
// //
// // Arguments:
// //
// // Returns:
// //
// // Remarks:
// //
// /////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////
void disableDoubleSolenoid() {
d_solenoid.set(DoubleSolenoid.Value.kOff);
}

// /////////////////////////////////////////////////////////////////
// // Function:
// /////////////////////////////////////////////////////////////////
// //
// // Purpose:
// //
// // Arguments:
// //
// // Returns:
// //
// // Remarks:
// //
// /////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////
void enableDoubleSolenoidForward(){
d_solenoid.set(DoubleSolenoid.Value.kForward);
}

// /////////////////////////////////////////////////////////////////
// // Function:
// /////////////////////////////////////////////////////////////////
// //
// // Purpose:
// //
// // Arguments:
// //
// // Returns:
// //
// // Remarks:
// //
// /////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////
void enableDoubleSolenoidReverse()
{
d_solenoid.set(DoubleSolenoid.Value.kReverse);
}

}

// /////////////////////////////////////////////////////////////////
// // Function:
// /////////////////////////////////////////////////////////////////
// //
// // Purpose:
// //
// // Arguments:
// //
// // Returns:
// //
// // Remarks:
// //
// /////////////////////////////////////////////////////////////////
// /////////////////////////////////////////////////////////////////