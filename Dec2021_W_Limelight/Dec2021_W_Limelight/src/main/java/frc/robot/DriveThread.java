/////////////////////////////////////////////////////////////////////
//  File:  DriveThread.java
/////////////////////////////////////////////////////////////////////
//
//Purpose:  Defines the thread class responsible for robot movement forward
//          and backward, left and right turns during autonomous
//          operation.  All autonomous movements would be included
//          in the t.run() function.
//
//Programmers:  
//
//Revisions:  Completely rewritten 12/07/2021
//
//Remarks: 
//
//         Implementing a drive thread for autonomous
//         operations.  Add the various movements within the run()
//         function.  Attempts have been made to simplify and make
//         functions consistent.
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
package frc.robot;

/////////////////////////////////////////////////////////////////////
//  Class:  DriveThread
/////////////////////////////////////////////////////////////////////
//
//Purpose: Defines the parameters used to drive the robot.
//
//
//Remarks:  FRC iterative robot template.
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
class DriveThread implements Runnable {
	String name;
	Thread t;
	Runtime r = Runtime.getRuntime();
	private Delay delay;



	// Fixed parameters for gyro operation. Specified here to facilitate
	// changes without confusion in the various functions using these
	// variables.
	final double ROT_SPEED = 0.5; // Starting rotation speed for turning
	// As we approach the target we reduce the speed by this factor
	final double ROT_ATTEN = 1.5;
	// proximity (in degrees) to target angle stage 1 rotation attenuation rate
	final double ANGL_PROX_1 = 25.0;
	// proximity (in degrees) to target angle stage 2 rotation attenuation rate
	final double ANGL_PROX_2 = 5.0;

	public static double position1;
	public double initPosition1;

	public double init_Angle;


	// Constructor
	DriveThread(String threadname) {


		// We want to establish an initial encoder reading. This will enable reseting
		// encoder position to zero when we start moving. We use absolute values to
		// make the subsequent subtraction more easily interpreted.
		Robot.robotDrive.frontLeftEncoder.setPosition(0.0);
		initPosition1 = Robot.robotDrive.frontLeftEncoder.getPosition(); // should be zero
		position1 = initPosition1;
		init_Angle = Robot.robotDrive.driveGyro.getAngle();

		// The initial position should be zero.
		System.out.println("Left Encoder Initial Position = " + initPosition1);
		System.out.println("Initial Heading = " + init_Angle);

		name = threadname;
		t = new Thread(this, name);
		System.out.println("New thread: " + t);
		delay = new Delay();
		t.start(); // Start the thread
	}

	public void run() {
		while (t.isAlive() == true) {

			// Set the flag active so that any joystick
			// manipulations are disabled while this
			// thread is active. Note that delays within
			// this thread will not affect the main()
			// program.
			Robot.drive_thread_active = true;

			// The various member functions would be called here.
			// For example:
			Robot.robotDrive.driveFwd(5.0); // move the robot forward 5.0 feet

			delay.delay_milliseconds(250.0);

			Robot.robotDrive.turnAbsolute(-90.0); // rotate 45 degrees CW

			delay.delay_milliseconds(250.0);

			Robot.robotDrive.turn2Heading(315.0); // turn to heading of 315 degrees

			Robot.robotDrive.turnAbsolute(180);

			delay.delay_milliseconds(100);

			Robot.robotDrive.turnAbsolute(-90);

			Robot.robotDrive.driveFwd(5.0);

			Robot.robotDrive.turnRight_Arcade(90);

			t.interrupt();
			
			// Wait for the thread to complete
			try {
				t.join();
			} catch (InterruptedException e) {
				System.out.println(name + "Interrupted.");
			}

			System.out.println(name + "Exiting Drive Thread");
			r.gc(); // force garbage collection (freeing of memory resources)

			// Reset flag
			Robot.drive_thread_active = false;
		}

		// Should get a false indication
		System.out.println("Thread Status = " + t.isAlive());
	}

}