#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
      
    //kalaman filter class params
    //init covariance matrix (same values as in udacity class)
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;
    //init state transition matrix (same values as in udacity class)
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ <<  1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;

    //init with data coming from measurements as passed as parameter
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
        float range     = measurement_pack.raw_measurements_(0); //rho
        float bearing_angle    = measurement_pack.raw_measurements_(1); //phi
        float radial_velocity = measurement_pack.raw_measurements_(2); //rho dot

        //coords (convert from polar)
        ekf_.x_(0) = range * cos(bearing_angle); //x
        if ( ekf_.x_(0) < 0.0001 ) {
            ekf_.x_(0) = 0.0001;
        }
        ekf_.x_(1) = range * sin(bearing_angle); //y
        if ( ekf_.x_(1) < 0.0001 ) {
            ekf_.x_(1) = 0.0001;
        }
        //velocity
        ekf_.x_(2) = radial_velocity * cos(bearing_angle); //vx
        ekf_.x_(3) = radial_velocity * sin(bearing_angle); //vy
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
        //for lidar set only x,y . no velocity data /ie. assuming constant
        //coords
        ekf_.x_(0) = measurement_pack.raw_measurements_(0); //x
        ekf_.x_(1) = measurement_pack.raw_measurements_(1); //y
        //velocity
        ekf_.x_(2) = 0.0; //vx
        ekf_.x_(3) = 0.0; //vy
    }

    previous_timestamp_ = measurement_pack.timestamp_ ;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

    //time elapsed
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_; //save current time for next iteration
    //set F matrix (1. Modify the F matrix so that the time is integrated)
    ekf_.F_(0,2) = dt;
    ekf_.F_(1,3) = dt;
    //set Q matrix (2. Set the process covariance matrix Q)
    ekf_.Q_ = MatrixXd(4, 4);
    // set the acceleration noise components
    float noise_ax = 9.0;
    float noise_ay = 9.0;
    //calculate Q
    float dt4_ax = pow(dt, 4)/4 * noise_ax;
    float dt3_ax = pow(dt, 3)/2 * noise_ax;
    float dt2_ax = pow(dt, 2) * noise_ax;
    float dt4_ay = pow(dt, 4)/4 * noise_ay;
    float dt3_ay = pow(dt, 3)/2 * noise_ay;
    float dt2_ay = pow(dt, 2) * noise_ay;

    ekf_.Q_ <<   dt4_ax, 0, dt3_ax, 0,
                0, dt4_ay, 0, dt3_ay,
                dt3_ax, 0, dt2_ax, 0,
                0, dt3_ay, 0, dt2_ay;
    
    ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Hj_= tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
