#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/colors.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/vision.hpp"
#include "pros/llemu.hpp"
#include "pros/adi.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain motors
// pros::MotorGroup left_motor_group({-18, 19, -20}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
// pros::MotorGroup right_motor_group({12, -11, 13}, pros::MotorGearset::blue); // right motors on ports 19, 20, 18

pros::MotorGroup left_motor_group({-18, -20, 19}, pros::MotorGearset::blue); // left motors on ports 1, 2, 3
pros::MotorGroup right_motor_group({12, 13, -11}, pros::MotorGearset::blue); // right motors on ports 19, 20, 18
pros::adi::DigitalOut mogo ('A');
// pros::adi::DigitalOut doinker ('B');
// pros::adi::DigitalOut mogorush('C');
// pros::adi::DigitalOut ringrush('D');

pros::Motor intake (16);
pros::Motor lb(15);
pros::Rotation lbrot(17);
// pros::Distance lbdist(19);
pros::Optical lbcolor(14);

bool mogo_value = false;
bool doinker_value = false;
// front, middle, back
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              9.829, // track width
                              lemlib::Omniwheel::NEW_275, 
                              480,
                              2
);

pros::Imu imu(8);
pros::Rotation vertical_encoder(9);
pros::Rotation horizontal_encoder(10);

// offset of the tracking wheel is equal to the length of the line perpendicular to it that ends at the tracking center

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
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

const int numStates = 5;
int states[numStates] = {5, 32, 70, 120, 250};
int currState = 0;
int target = states[0];
bool manualcontrol = true;

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

void snap() {
    int currPos = lbrot.get_position()/100;
    int closest = states[0];
    for (int i=0; i<numStates; i++){
        if ((abs(states[i]) - currPos) < (abs(closest - currPos))){
            closest = states[i];
        }
    }
    target = closest;

}

void ladybrown() {
    double kp = 2;
    double error = target - (lbrot.get_position()/100.0);
    double velocity = kp * error;
    lb.move(velocity);
}


void initialize() {
	// print encoder readings to the brain
	pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate();
    // auto imus = pros::Imu::get_all_devices();
    // for (auto device : imus) {
    //     device.reset();
    // }
    // for (auto device : imus) {
    //     while (device.is_calibrating())
    //     pros::delay(100);
    // }
    // chassis.calibrate(true); // calibrate sensors
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "Lb position: %d", (lbrot.get_position()/100));
            // pros::lcd::print(0, "Lb distance %f!", lbdist.get_distance());
            // pros::lcd::print(0, "Lb confidence %f!", lbdist.get_confidence());
            pros::delay(10);
        }
    });
    pros::Task lbtask([&]() {
        while (true) {
            pros::lcd::print(1, "target: %d", target);
            // pros::lcd::print(2, "distance: %d", lbdist.get_distance());
            // pros::lcd::print(3, "distance: %d", lbdist.get_confidence());
            pros::lcd::print(3, "manualcontrol: %d", manualcontrol);
            pros::lcd::print(4, "hue: %lf", lbcolor.get_hue());
            if (!manualcontrol){
                ladybrown();
            }
            if ((lbcolor.get_hue() < 230) && lbcolor.get_hue() > 200){
                pros::lcd::print(5, "hue: %lf", lbcolor.get_hue());
            }
            // if (lbdist.get_distance() < 10 && (lbdist.get_distance() > 5)){
            //     intake.brake();
            //     pros::lcd::print(4, "bababooey: %d", lbdist.get_distance());
            // }
            // pros::lcd::print(0, "Lb distance %f!", lbdist.get_distance());
            // pros::lcd::print(0, "Lb confidence %f!", lbdist.get_confidence());
            pros::delay(10);
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

void autonomous() {}

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

        // if want to prioritize turning over throttle, value closer to 1.
        // chassis.arcade(rightY, leftX, false, 0.75);
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
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
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move(127);
        }
        else{
            intake.brake();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            manualcontrol = true;
            lb.move(127);
        }
        else{
            snap();
            manualcontrol = false;
        }
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            manualcontrol = true;
            lb.move(-127);
        }

        pros::delay(20);
    }

}