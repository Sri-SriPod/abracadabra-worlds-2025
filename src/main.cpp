#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain motors
pros::MotorGroup left_motor_group({-9, -19, -3}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motor_group({20, 1, 2}, pros::MotorGearset::blue); // right motors on ports 19, 20, 18
pros::adi::DigitalOut mogo ('A');
pros::adi::DigitalOut outtake ('F');
pros::adi::DigitalOut doinker ('H');
pros::Motor intake1 (4);
pros::Motor intake2 (8);

bool mogo_value = false;
bool doinker_value = false;
// front, middle, back
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              12.437, // track width
                              lemlib::Omniwheel::NEW_275, 
                              480, 
                              2
);

// create an imu on port 5
pros::Imu imu(15);

// pros::Rotation vertical_sensor(15);
pros::Rotation horizontal_sensor(7);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, 0);
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, lemlib::Omniwheel::NEW_2, -2.5);

lemlib::OdomSensors sensors(NULL, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(6.7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              1, // derivative gain (kD)
                                              0.45, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew) (tune this)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              17.2, // derivative gain (kD)
                                              0.45, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);
void intake(){
    intake1.move(-115);
    intake2.move(115);
}

void initialize() {
	// print encoder readings to the brain
	pros::lcd::initialize(); // initialize brain screen
    auto imus = pros::Imu::get_all_devices();
    for (auto device : imus) {
        device.reset();
    }
    for (auto device : imus) {
        while (device.is_calibrating())
        pros::delay(100);
    }
    // chassis.calibrate(); // calibrate sensors
    chassis.calibrate(true); // calibrate sensors
    mogo.set_value(false);
    pros::Task screen_task([&]() {
        while (true) {
            
			// print measurements from the rotation sensor
			// pros::lcd::print(0, "Vertical: %i", vertical_sensor.get_position());
			pros::delay(10); // delay to save resources. DO NOT REMOVE

			/**
			When you push the robot forwards, the measured position of the vertical encoder(s) should increase. 
			If they decrease, the sensor(s) needs to be reversed. 
			When you push the robot to the right (relative to the robot), the position measured by horizontal encoders should increase. 
			If they decrease, the sensor(s) needs to be reversed.
			*/
            
			// print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // pros::lcd::print(3, "Horizontal: %i", horizontal_sensor.get_position());
            // delay to save resources
            pros::delay(20);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}


/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void rerun(){
    pros::delay(500);
}

void autonomous() {

    }


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    while (true) {
        // move the robot
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        chassis.arcade(rightY, leftX);
        

        // get left y and right x positions

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            if (mogo_value){
                mogo.set_value(false);
                mogo_value = false;
            }
            else{
                mogo.set_value(true);
                mogo_value = true;
            }

            pros::delay(170); // how long you can press it for

        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            if (doinker_value){
                doinker.set_value(false);
                doinker_value = false;
            }
            else{
                doinker.set_value(true);
                doinker_value = true;
            }

            pros::delay(170); // how long you can press it for
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake();
        }
        // else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        //     intake1.move(-127);
        //     intake2.move(25);
        // }
        else{
            intake1.brake();
            intake2.brake();
        }
        
        
        pros::delay(25); 
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            rerun();

            pros::delay(250); // how long you can press it for
        }
        
        
    }

}