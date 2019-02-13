/*
Used for teaching controller design
Pavan Saranguhewa
Course Instructor : Lantao Liu
ISE, Indiana University
*/

//We are using Eigen matrix library for c++
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <iostream>

using namespace std;
using namespace Eigen;

class LQR_dubin{
public:
  //Creation of the class object
  LQR_dubin(MatrixXd Q, MatrixXd R, float dt, int N) : Q(Q), R(R), dt(dt), N(N) {

  }

  //initializing function for our LQR model. 
  //Each time the goal is updated we need to call this.
  void LQR_init(VectorXd Goal) {

    A = MatrixXd(3,3);  //A Matrix
    A << 1, 0, -Goal(3,0)*sin(Goal(2,0))*dt,
         0, 1, Goal(3,0)*cos(Goal(2,0))*dt,
         0, 0, 1;

    B = MatrixXd(3,2);  //B Matrix
    B << cos(Goal(2,0))*dt, 0,
         sin(Goal(2,0))*dt, 0,
         0, dt;

    for(int i = 0; i<=N; i++) {
      Pt[i] = MatrixXd::Zero(3,3);  //Pt[i] element of LQR calculation
    }

    Pt[N] = Q;  //Assigning Qf as Pt[N]
    for(int i = N;i>0;i--) {
      //Calculating Pt[i-1] from Pt[i]
      Pt[i-1] = Q + A.transpose()*Pt[i]*A - A.transpose()*Pt[i]*B*(R + B.transpose()*Pt[i]*B).inverse()*B.transpose()*Pt[i]*A;

    }
  }

  //Function to calculate the control output based on current state
  Vector2d Update_Ut(int index, MatrixXd Goal, double Odom_x, double Odom_y, double Odom_yaw) {
  
    MatrixXd Kt = MatrixXd::Zero(2,3);  //Kt value for the current time step

    Kt = -(R + B.transpose()*Pt[index+1]*B).inverse()*B.transpose()*Pt[index+1]*A;  //Calculating Kt
    MatrixXd Xt = MatrixXd::Zero(3,1);  //Xt matrix to hold the current state

    Xt << Odom_x - Goal(0,0), //Populating Xt with current actual position and orientation
          Odom_y - Goal(1,0),
          Odom_yaw - Goal(2,0);

    //INFO messages
    cout << "X distance to go : " << Xt(0,0) << endl;
    cout << "Y distance to go : " << Xt(1,0) << endl;
    cout << "Theta to go : " << Xt(2,0) << endl;
    MatrixXd Ut;  // Control outputs matrix Ut
    Ut = Kt*Xt;   // Calculating Ut

    Vector2d u_out;
    float u0 = Ut(0,0) + Goal(3,0); //Ut(0,0) is speed. Adding speed at the goal to the calculated speed
    float u1 = Ut(1,0); //Ut(1,0) is angular velocity
    
    //Limiting the speed of the vehicle to acceptable values.
    if (u0 > 1.5)
      u0 = 1.5;
    else if (u0 < -1.5)
      u0 = -1.5;
    u_out << u0, u1;  
    return u_out; //returning the vector containing control output speed and angular velocity.
  }


private:
  //Variables used in the class
  MatrixXd A,B,Q,R;
  float dt; //Timestep
  int N;  //Horizon
  MatrixXd *Pt = new MatrixXd[N+1]; //Defining a matrix array for holding Pt values

};
