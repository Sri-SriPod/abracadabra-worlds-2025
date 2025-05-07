#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
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
pros::adi::DigitalOut doinker ('H');
pros::adi::DigitalOut intakelift('E');
pros::Motor intake (16);
pros::Motor lb(15);
pros::Rotation lbrot(17);
// pros::Distance lbdist(19);
pros::Optical lbcolor(14);

bool mogo_value = false;
bool doinker_value = false;
bool intake_value = false;
bool colorstop = false;
bool colordetect = false;
float initial = intake.get_position();

bool auton_alliance_stake = false;
// front, middle, back
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              9.829, // track width
                              lemlib::Omniwheel::NEW_275, 
                              480,
                              2
);

pros::Imu imu(4);
pros::Rotation vertical_encoder(9);
pros::Rotation horizontal_encoder(10);

// offset of the tracking wheel is equal to the length of the line perpendicular to it that ends at the tracking center

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 0); // -0.9
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -1.5); // -2

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(3.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0.5, // derivative gain (kD)
                                              0.45, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(1.9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
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

const int numStates = 5;
int states[numStates] = {5, 32, 70, 200, 250};
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
    lbcolor.set_led_pwm(100);
    pros::Task screen_task([&]() {
        while (true) {
            bool blue = ((lbcolor.get_hue() < 300) && (lbcolor.get_hue() > 120)) && ((lbcolor.get_proximity() > 245) && (target != states[1]));
            bool red = ((lbcolor.get_hue() < 10) || (lbcolor.get_hue() > 300)) && ((lbcolor.get_proximity() > 245) && (target != states[1]));
            if (red){
                initial = intake.get_position();
                pros::lcd::print(2, "yooooo: %f", initial);
                colordetect = true;
                }
            if (colordetect){
                if (intake.get_position() - initial > 37){
   
                    intake.move(-127);
                    colorstop = true;
                    pros::delay(700);
                    // pros::delay(100);
                    // intake.move(0);
                    // pros::delay(2000);
                    colorstop= false;
                    colordetect = false;
                    pros::lcd::clear_line(2);
                }
            }


            pros::lcd::print(0, "Lb position: %d", (lbrot.get_position()/100));
            pros::lcd::print(5, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(6, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(7, "Theta: %f", chassis.getPose().theta); // heading

            // pros::lcd::print(5, "verical Sensor: %i", vertical_encoder.get_position());
            // pros::lcd::print(6, "horizontal Sensor: %i", horizontal_encoder.get_position());



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
            // pros::lcd::print(2, "manualcontrol: %d", manualcontrol);
            pros::lcd::print(3, "hue: %lf", lbcolor.get_hue());
            pros::lcd::print(4, "intake pos: %f", intake.get_position());
            if (!manualcontrol){
                ladybrown();
            }
            if (auton_alliance_stake){
                if ((lbrot.get_position()/100) > 160){
                    lb.move(0);
                }
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

 void blue_3_top() {
    chassis.setPose(-55.797, -14.794, 340);
    // // score lb, bring back
    auton_alliance_stake = true;
    lb.move(90);
    // auton_alliance_stake = false;
    // manualcontrol = false;
    // target = 0;
    // first mogo
    pros::delay(1000);
    auton_alliance_stake = false;
    manualcontrol = false;
    target = 3;
    pros::delay(1000);
    // chassis.turnToHeading(306, 10000);
    // pros::delay(5000);
    chassis.moveToPose(-24.5, -24, 306, 7000, {.forwards=false});
    // // clamp mogo
    // mogo_value = true;
    // mogo.set_value(true);
    // chassis.turnToHeading(40, 500);
    // chassis.moveToPose(47.548, -0.239, 40, 1000, {.forwards=true}, false);
    // intake.move(127);
    // pros::delay(1000);
    // intake.move(0);
    // // lift ladybrown to state
    // manualcontrol = false;
    // target = states[1];
    // chassis.turnToHeading(205, 500);
    // intake.move(127);
    // chassis.moveToPose(23.671, -47.325, 205, 1000, {.forwards=true});
    // chassis.turnToHeading(300, 500);
    // chassis.moveToPose(47.325, -0.239, 300, 1000);
    // chassis.turnToHeading(40, 500);

}


void jiggle(){
    for (int i = 0; i < 5; i++){
        left_motor_group.move(50);
        right_motor_group.move(50);
        pros::delay(100);
        left_motor_group.move(-50);
        right_motor_group.move(-50);
        pros::delay(100);
        left_motor_group.move(0);
        right_motor_group.move(0);
    }
}

void blue_pos(){
    chassis.setPose(62.569, -23, 90);
    chassis.moveToPoint(23, -23, 1000, {.forwards=false});
    // clamp mogo, lift intake
    pros::delay(1000);
    mogo.set_value(true);
    mogo_value = true;
    // intakelift.set_value(true);
    // intake_value = true;
    pros::delay(1000);
    intake.move(127);
    // get top ring
    // chassis.turnToPoint(47.281, 0, 1000, {}, true);
    // chassis.moveToPoint(47.281, 0, 1000, {.maxSpeed = 60}, true);
    intake.move(127);
    // get 2nd ring
    chassis.turnToPoint(25.326, -44.918, 1000, {}, true);
    // intakelift.set_value(false);
    // intake_value = false;
    intake.move(127);
    pros::delay(1000);
    chassis.moveToPoint(25.326, -44.918, 1000, {}, true);
    pros::delay(1500);
    intake.move(127);
    // corner
    chassis.turnToPoint(53.238, -63.169, 1000, {}, true);
    chassis.moveToPoint(53.238, -63.169, 1000, {}, true);
    pros::delay(500);
    chassis.turnToPoint(66.39, -66.658, 1000, {}, true);
    pros::delay(1000);
    intake.move(127);
    left_motor_group.move(60);
    right_motor_group.move(60);
    pros::delay(2000);
    left_motor_group.move(-80);
    right_motor_group.move(-80);
    pros::delay(70);
    left_motor_group.move(0);
    right_motor_group.move(0);
    // touch bar
    pros::delay(500);
    chassis.turnToPoint(22.642, -22.642, 1000, {}, true);
    manualcontrol = false;
    target = 180;
    chassis.moveToPoint(22.642, -22.642, 1000, {.maxSpeed = 100}, true);
    pros::delay(1000);
    intake.move(0);
}

void blue_neg(){
    chassis.setPose(62.569, 23, 90);
    chassis.moveToPoint(23, 23, 1000, {.forwards=false});
    // clamp mogo, lift intake
    mogo.set_value(true);
    mogo_value = true;
    intakelift.set_value(true);
    intake_value = true;
    intake.move(127);
    pros::delay(500);
    // get top ring
    chassis.turnToPoint(47.281, 0, 1000, {}, true);
    chassis.moveToPoint(47.281, 0, 1000, {}, true);
    intake.move(127);
    // lower intake
    chassis.turnToPoint(23.376, 47.229, 1000, {}, true);
    intakelift.set_value(false);
    intake_value = false;
    pros::delay(1000);
    // get 2nd ring
    chassis.moveToPoint(23.376, 47.229, 1000, {}, true);
    intake.move(127);
    pros::delay(1000);
    // corner doinker
    chassis.turnToPoint(60.25, 60.09, 1000, {}, true);
    chassis.moveToPoint(60.25, 60.09, 1000, {}, true);
    pros::delay(500);
    chassis.turnToHeading(47, 1000, {}, false);
    pros::delay(50);

    // left_motor_group.move(-127);
    // right_motor_group.move(-127);
    // pros::delay(45);
    // left_motor_group.move(0);
    // right_motor_group.move(0);
    
    doinker.set_value(true);
    doinker_value = true;
    pros::delay(500);
    chassis.turnToPoint(23.376, -47.229, 1000, {}, true);
    doinker.set_value(false);
    doinker_value = false;
    pros::delay(500);
    chassis.turnToHeading(44, 1000, {}, false);
    pros::delay(500);
    intake.move(127);
    // get last ring
    left_motor_group.move(80);
    right_motor_group.move(80);
    pros::delay(500);
    left_motor_group.move(-80);
    right_motor_group.move(-80);
    pros::delay(150);
    left_motor_group.move(0);
    right_motor_group.move(0);
    intake.move(127);
    // touch bar
    chassis.turnToPoint(18.01, 18.509, 1000, {}, true);
    manualcontrol = false;
    target = 70;
    chassis.moveToPoint(18.01, 18.509, 1000, {}, true);
    pros::delay(1000);
    intake.move(0);
}

void red_pos(){
    // chassis.setPose(-58.693, -13.379, 340);
    // auton_alliance_stake = true;
    // lb.move(90);
    // pros::delay(1000);
    // auton_alliance_stake = false;
    // manualcontrol = false;
    // target = 3;
    // pros::delay(1000);
    // chassis.turnToPoint(-23, -23, 1000, {.forwards=false});
    chassis.setPose(-62.569, -23, 270);
    chassis.moveToPoint(-23, -23, 1000, {.forwards=false});
    // clamp mogo, lift intake
    pros::delay(1000);
    mogo.set_value(true);
    mogo_value = true;

    // intakelift.set_value(true);
    // intake_value = true;

    pros::delay(1000);
    intake.move(127);
    // get top ring

    // chassis.turnToPoint(-47.281, 0, 1000, {}, true);
    // chassis.moveToPoint(-47.281, 0, 1000, {.maxSpeed = 60}, true);
    // intake.move(127);

    // get 2nd ring
    chassis.turnToPoint(-25.326, -44.918, 1000, {}, true);
    // intakelift.set_value(false);
    // intake_value = false;
    intake.move(127);
    pros::delay(1000);
    chassis.moveToPoint(-25.326, -44.918, 1000, {}, true);
    pros::delay(2000);
    intake.move(127);
    // corner
    chassis.turnToPoint(-53.659, -65.206, 1000, {}, true);
    chassis.moveToPoint(-53.659, -65.206, 1000, {}, true);
    pros::delay(100);
    chassis.turnToPoint(-66.39, -66.658, 1000, {}, true);
    pros::delay(1000);
    intake.move(127);
    left_motor_group.move(60);
    right_motor_group.move(60);
    pros::delay(2000);
    left_motor_group.move(-80);
    right_motor_group.move(-80);
    pros::delay(100);
    left_motor_group.move(0);
    right_motor_group.move(0);
    // touch bar
    chassis.turnToPoint(-22.642, -22.642, 1000, {}, true);
    manualcontrol = false;
    target = 180;
    chassis.moveToPoint(-22.642, -22.642, 1000, {.maxSpeed = 115}, true);
    pros::delay(1200);
    intake.move(0);
}

void red_neg(){
    chassis.setPose(-62.569, 23, 270);
    chassis.moveToPoint(-23, 23, 1000, {.forwards=false});
    // clamp mogo, lift intake
    mogo.set_value(true);
    mogo_value = true;
    intakelift.set_value(true);
    intake_value = true;
    intake.move(127);
    pros::delay(500);
    // get top ring
    chassis.turnToPoint(-47.281, 0, 1000, {}, true);
    chassis.moveToPoint(-47.281, 0, 1000, {}, true);
    intake.move(127);
    pros::delay(300);
    // get 2nd ring
    chassis.turnToPoint(-23.376, 47.229, 1000, {}, true);
    intakelift.set_value(false);
    intake_value = false;
    intake.move(127);
    pros::delay(1000);
    chassis.moveToPoint(-23.376, 47.229, 1000, {}, true);
    intake.move(127);
    pros::delay(1000);
    // corner + doinker
    chassis.turnToPoint(-64.5, 54.584, 1000, {}, true);
    chassis.moveToPoint(-64.5, 54.584, 1000, {}, true);
    pros::delay(500);
    intake.move(0);
    chassis.turnToHeading(323, 1000, {}, false);
    left_motor_group.move(-127);
    right_motor_group.move(-127);
    pros::delay(30);
    left_motor_group.move(0);
    right_motor_group.move(0);
    // clear rings
    doinker.set_value(true);
    doinker_value = true;
    pros::delay(500);
    chassis.turnToPoint(-23.376, 47.229, 1000, {}, true);
    doinker.set_value(false);
    doinker_value = false;
    // intake last one
    chassis.turnToHeading(330, 1000);
    pros::delay(500);
    intake.move(127);
    left_motor_group.move(80);
    right_motor_group.move(80);
    pros::delay(500);
    left_motor_group.move(-80);
    right_motor_group.move(-80);
    pros::delay(150);
    left_motor_group.move(0);
    right_motor_group.move(0);
    intake.move(127);
    // touch bar
    chassis.turnToPoint(-18.01, 18.509, 1000, {}, true);
    manualcontrol = false;
    target = 70;
    chassis.moveToPoint(-18.01, 18.509, 1000, {}, true);
    pros::delay(1000);
    intake.move(0);
}

void blue_neg1() {
    chassis.setPose(62.569, 23, 90);
    chassis.moveToPoint(23, 23, 1000, {.forwards=false});
    // clamp mogo, lift intake
    mogo.set_value(true);
    mogo_value = true;
    intakelift.set_value(true);
    intake_value = true;
    intake.move(127);
    pros::delay(500);
    // get top ring
    chassis.turnToPoint(47.281, 0, 1000, {}, true);
    chassis.moveToPoint(47.281, 0, 1000, {}, true);
    intake.move(127);
    chassis.turnToPoint(4.525, 43.671, 1000, {}, true);
    // lower intake
    intakelift.set_value(false);
    intake_value = false;
    pros::delay(500);
    chassis.moveToPoint(9.02, 38.676, 1000, {}, true);
    pros::delay(700);
    chassis.turnToPoint(8.021, 51.162, 1000, {}, true);
    // from here, same code as before... (corner + doinker)
    chassis.turnToPoint(60.25, 60.09, 1000, {}, true);
    chassis.moveToPoint(60.25, 60.09, 1000, {}, true);
    pros::delay(500);
    chassis.turnToHeading(47, 1000, {}, false);
    pros::delay(50);
    // left_motor_group.move(-127);
    // right_motor_group.move(-127);
    // pros::delay(45);
    // left_motor_group.move(0);
    // right_motor_group.move(0);
    doinker.set_value(true);
    doinker_value = true;
    pros::delay(500);
    chassis.turnToPoint(23.376, -47.229, 1000, {}, true);
    doinker.set_value(false);
    doinker_value = false;
    pros::delay(500);
    chassis.turnToHeading(44, 1000, {}, false);
    pros::delay(500);
    intake.move(127);
    // get last ring
    left_motor_group.move(80);
    right_motor_group.move(80);
    pros::delay(500);
    left_motor_group.move(-80);
    right_motor_group.move(-80);
    pros::delay(150);
    left_motor_group.move(0);
    right_motor_group.move(0);
    intake.move(127);
    // touch bar
    chassis.turnToPoint(18.01, 18.509, 1000, {}, true);
    manualcontrol = false;
    target = 70;
    chassis.moveToPoint(18.01, 18.509, 1000, {}, true);
    pros::delay(1000);
    intake.move(0);
    pros::delay(1000);
}

void red_neg1(){
    chassis.setPose(-62.569, 23, 270);
    chassis.moveToPoint(-23, 23, 1000, {.forwards=false});
    // clamp mogo, lift intake
    pros::delay(1000);
    mogo.set_value(true);
    mogo_value = true;
    intakelift.set_value(true);
    intake_value = true;
    intake.move(127);
    pros::delay(2500);
    // get top ring
    chassis.turnToPoint(-47.281, 0, 1000, {}, true);
    chassis.moveToPoint(-47.281, 0, 1000, {.maxSpeed = 100}, true);
    intake.move(127);
    chassis.turnToPoint(-5.271, 42.041, 1000, {}, true);
    // lower intake
    intakelift.set_value(false);
    intake_value = false;
    intake.move(127);
    pros::delay(500);
    chassis.moveToPoint(-5.271, 42.041, 1000, {}, true);
    pros::delay(900);
    chassis.turnToPoint(-10.907, 59.218, 1000, {}, true);
    chassis.moveToPoint(-10.907, 59.218, 1000, {}, true);
    pros::delay(1000);
    intake.move(127);
    chassis.turnToPoint(-23.79, 47.409, 1000, {}, true);
    chassis.moveToPoint(-23.79, 47.409, 1000, {}, true);
    pros::delay(1000);
    // from here, same code as before... (corner + doinker)
    chassis.turnToPoint(-53.238, 63.169, 1000, {}, true);
    chassis.moveToPoint(-53.238, 63.169, 1000, {}, true);
    pros::delay(100);
    chassis.turnToPoint(-66.39, 66.658, 1000, {}, true);
    pros::delay(100);
    intake.move(127);
    left_motor_group.move(60);
    right_motor_group.move(60);
    pros::delay(2000);
    left_motor_group.move(-80);
    right_motor_group.move(-80);
    pros::delay(1000);
    left_motor_group.move(0);
    right_motor_group.move(0);
    // touch bar
    chassis.turnToPoint(-18.01, 18.509, 1000, {}, true);
    manualcontrol = false;
    target = 70;
    chassis.moveToPoint(-18.01, 18.509, 1000, {}, true);
    pros::delay(1000);
    intake.move(0);

}


void autonomous() {
    // 12.625 by 15
    // chassis.setPose(0, 0, 0);
    // chassis.moveToPoint(0, 48, 1000);
    // chassis.turnToHeading(180, 1000);
    // blue_pos(); // 4 rings in mogo
    // blue_neg(); // 4 rings in mogo
    // red_neg(); // 4 rings in mogo
    red_pos(); // 4 rings in mogo
    // blue_neg1(); // 6 rings in mogo
    // red_neg1(); // 6 rings in mogo
    
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

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            if (doinker_value){
                doinker.set_value(false);
                doinker_value = false;
            }
            else{
                doinker.set_value(true);
                doinker_value = true;
            }

            pros::delay(200); // how long you can press it for
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            if (intake_value){
                intakelift.set_value(false);
                intake_value = false;
            }
            else{
                intakelift.set_value(true);
                intake_value = true;
            }

            pros::delay(200); // how long you can press it for
        }

        if (!colorstop){
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
                intake.move(127);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
                intake.move(-127);   // Forward intake
            }
            else{
                intake.brake();
            }
            pros::delay(10);
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