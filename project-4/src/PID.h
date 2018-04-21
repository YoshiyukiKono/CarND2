#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double cte;
  //double steer_value;

  double best_error;
  double error;
 
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  void InitTwiddle(double Kp, double Ki, double Kd, double tol);
  bool IsReset();
  void Restart();
  void Clear();

private:
  double dp[3];
  double p[3];
  double best_p[3];
  bool isTwiddle;
  
  virtual void Calc() = 0;
  //void CalcSteerValue();
  //void AdjustSteerValue();
  
  void Train(double tol);
  void Twiddle(double tol);
  //void TwiddleInLoop();
  //void TwiddleInOver();
  void ReflectTwiddleParams();
  
  void UpdateBest();
  void SetBest();
  void MoveNextParam();
  
  int num_step;
  int NUM_TRAIN_STEPS;
  int num_round;
  int NUM_TRAIN_ROUNDS;

  bool isInitialized;
  bool isOver;
  bool isReset;
  bool isFirstRun;
  int idx_p;
  
  double tol;

};

class SteeringControler : public PID {
public:
  double steer_value;
private:
  void Calc();
  void AdjustSteerValue();
};

class ThrottleControler : public PID {
public:
  double throttle_value;
private:
  void Calc();
  void AdjustThrottleValue();
};
#endif /* PID_H */
