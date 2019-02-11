/*
Used for teaching controller design
Lantao Liu
ISE, Indiana University
*/

#include <eigen3/Eigen/Dense>
#include <sstream>
#include <iostream>

using namespace std;
using namespace Eigen;

class LQR_dubin{
public:
  LQR_dubin(MatrixXd Q, MatrixXd R, float dt, int N) : Q(Q), R(R), dt(dt), N(N) {

  }

  void LQR_init(VectorXd Goal) {

    A = MatrixXd(3,3);
    A << 1, 0, -Goal(3,0)*sin(Goal(2,0))*dt,
         0, 1, Goal(3,0)*cos(Goal(2,0))*dt,
         0, 0, 1;

    B = MatrixXd(3,2);
    B << cos(Goal(2,0))*dt, 0,
         sin(Goal(2,0))*dt, 0,
         0, dt;

    for(int i = 0; i<=N; i++) {
      Pt[i] = MatrixXd::Zero(3,3);
    }

    Pt[N] = Q;
    for(int i = N;i>0;i--) {
      Pt[i-1] = Q + A.transpose()*Pt[i]*A - A.transpose()*Pt[i]*B*(R + B.transpose()*Pt[i]*B).inverse()*B.transpose()*Pt[i]*A;

    }
  }

  Vector2d Update_Ut(int index, MatrixXd Goal, double Odom_x, double Odom_y, double Odom_yaw) {

    MatrixXd Kt = MatrixXd::Zero(2,3);

    Kt = -(R + B.transpose()*Pt[index+1]*B).inverse()*B.transpose()*Pt[index+1]*A;
    MatrixXd Xt = MatrixXd::Zero(3,1);

    Xt << Odom_x - Goal(0,0),
          Odom_y - Goal(1,0),
          Odom_yaw - Goal(2,0);

    cout << "X distance to go : " << Xt(0,0) << endl;
    cout << "Y distance to go : " << Xt(1,0) << endl;
    cout << "Theta to go : " << Xt(2,0) << endl;
    MatrixXd Ut;
    Ut = Kt*Xt;

    Vector2d u_out;
    float u0 = Ut(0,0) + Goal(3,0); //Adding velocity at the goal
    float u1 = Ut(1,0);
    if (u0 > 1.5)
      u0 = 1.5;
    else if (u0 < -1.5)
      u0 = -1.5;
    u_out << u0, u1;
    return u_out;
  }


private:
  MatrixXd A,B,Q,R;
  float dt;
  int N;
  MatrixXd *Pt = new MatrixXd[N+1];

};
