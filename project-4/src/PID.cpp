#include "PID.h"
#include <iostream>
#include <limits>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	isTwiddle = false;
	num_round = 0;
	num_step = 0;
	idx_p = 0;
	isInitialized = false;
	isOver = false;
	isReset = false;
	isFirstRun = true;
}

PID::~PID() {}

/*
The simulator will provide you the cross track error (CTE) and the velocity (mph) 
in order to compute the appropriate steering angle.
One more thing. The speed limit has been increased from 30 mph to 100 mph. 
Get ready to channel your inner Vin Diesel and try to drive SAFELY as fast as possible! 
NOTE: you don't have to meet a minimum speed to pass.
	*/
void PID::Init(double Kp, double Ki, double Kd) {

	p_error = 0;
	i_error = 0;
	d_error = 0;

	//Coefficients
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	isInitialized = true;
}
void PID::InitTwiddle(double Kp, double Ki, double Kd, double tol) {
	isTwiddle = true;
	best_error = std::numeric_limits<double>::max();

	dp[0] = 0.1*Kp;//0.5;//1;
	dp[1] = 0.1*Ki;//0.001;//1;
	dp[2] = 0.1*Kd;//0.5;//1;

	p[0] = Kp;
	p[1] = Ki;
	p[2] = Kd;
	best_p[0] = Kp;
	best_p[1] = Ki;
	best_p[2] = Kd;

	NUM_TRAIN_STEPS = 1000;//800;//600;//400;//800;//400;//1000; When using 400, best error was 0.002. When 600, 0.003
	num_round = 0;
	NUM_TRAIN_ROUNDS = 20;
	error = 0;
	this->tol = tol;
	Init(Kp, Ki, Kd);
}
void PID::UpdateError(double cte) {
//std::cout << "[UpdateError] num_step:" << num_step << " isTwiddle:" << isTwiddle << std::endl;
	this->cte = cte;
	d_error = cte - p_error;
	p_error  = cte;
	i_error += cte;
	if (isTwiddle) {
		Train(tol);
	} 
	Calc();
	num_step += 1;
}

double PID::TotalError() {
	// Calculate the total PID error.
	return -Kp * p_error -Kd * d_error -Ki * i_error;
}

void PID::Train(double tol) {

	if (num_step < NUM_TRAIN_STEPS) {
		if (num_step >= NUM_TRAIN_STEPS/2) {
			if (num_step == NUM_TRAIN_STEPS/2) {
std::cout << "[RND(" << num_round << ") IDX(" << idx_p << ")] Start new cte:" << cte << " ERR:" << error << endl;
			}
			error += cte*cte;
//std::cout << "[RND(" << num_round << ") IDX(" << idx_p << ")] cte:" << cte << " ERR:" << error << endl;
		}
	} else if (num_step == NUM_TRAIN_STEPS) {
		error = error/int(num_step/2);
		if (num_round == 0) {
std::cout << "[RND(" << num_round << ")] cte:" << cte << " ERR:" << error << " <Skipp the first round as its error tends to be better than the comming rounds.>" << endl;
			num_step = 0;
			Restart();
			return;
		}
		if (isFirstRun) {
			best_error = error;
			isFirstRun = false;
		}
		
std::cout << "[Train] num_step:" << num_step << "/" << NUM_TRAIN_STEPS << " error:" << error << std::endl;
		num_step = 0;
		Twiddle(tol);
	} 
	return;
}
void PID::Twiddle(double tol) {

std::cout << "[RND(" << num_round << ") IDX(" << idx_p << ")] BEST:" << best_error << " ERR:" << error << endl;
std::cout << "[TOTAL] dp:" << dp[0] + dp[1] + dp[2] << " dp[0]:" << dp[0] << " dp[1]:" << dp[1] << " dp[2]:" << dp[2] << std::endl;

	if (isOver) {
		if (error < best_error) {
std::cout << "[Twiddle] After decreasing, less than best! move to ANOTHER PARAM after increasing dp of this parameter." << std::endl;
			best_error = error;
			UpdateBest();
			dp[idx_p] *= 1.1;
		} else {
std::cout << "[Twiddle] Even after decreasing, larger than best, move to ANOTHER PARAM after decreasing dp of this parameter." << std::endl;
//std::cout << "[Twiddle] p[idx_p]:" << p[idx_p] << " PLUS dp[idx_p]:"  << dp[idx_p] << " EQUAL:" << (p[idx_p] + dp[idx_p]) << std::endl;
			p[idx_p] += dp[idx_p];
			ReflectTwiddleParams();
			dp[idx_p] *= 0.9;
		}
		MoveNextParam();
		isOver = false;
		Restart();
		return;
	}

	if(dp[0]+dp[1]+dp[2] > tol) {
//std::cout << "[round(" << num_round << ") idx(" << idx_p << ")] best err:" << best_error << " err:" << error << " dp[0]:" << dp[0] << " dp[1]:" << dp[1] << " dp[2]:" << dp[2] << std::endl;
			//TwiddleInLoop();
			// Use the latest error
			//double error = cte; // Need to update error for every loop for idx_p (TODO)
			if (num_round == 0) {
				p[idx_p] += dp[idx_p];
std::cout << "[Twiddle] This is just after the first round. So, just try to increase param and get another error that will be compared to the first round error." << std::endl;
				Restart();
				ReflectTwiddleParams();
				//std::cout << "PID Twiddle (num_round == 0) num_round:" << num_round << " idx_p:" << idx_p << std::endl;
				return;
			}
			if (error < best_error) { // improvement
				best_error = error;
				dp[idx_p] *=1.1; 
std::cout << "[Twiddle] Less than best! move to ANOTHER PARAM after increasing dp of this parameter." << std::endl;
				MoveNextParam();//idx_p += 1;
				UpdateBest();
				Restart();
				return;
			} else {
std::cout << "[Twiddle] Larger than best. try THIS PARAM again with decreasing the paramter." << std::endl;
//std::cout << "[Twiddle] p[idx_p]:" << p[idx_p] << " MINUS 2*dp[idx_p]:"  << 2*dp[idx_p] << " EQUAL:" << (p[idx_p] - 2*dp[idx_p]) << std::endl;
				isOver = true;
				p[idx_p] -= 2*dp[idx_p]; // other direction
				ReflectTwiddleParams();
				Restart();
				return;
			}
	} else {
std::cout << "[Twiddle] Total dp value is larger than tolerance.So, training is completed. - dp[0]+dp[1]+dp[2]:" << (dp[0]+dp[1]+dp[2]) << " tol:" << tol << std::endl;
		isTwiddle = false;
		isReset = true;
		SetBest();
	}
}

void PID::Restart() {
//std::cout << "[Restart]" << std::endl;
	isReset = true;
	num_round += 1;
	Clear();
}
void PID::Clear() {
//std::cout << "[Restart]" << std::endl;
	num_step = 0;
	error = 0;
	p_error = 0;
	i_error = 0;
	d_error = 0;
}
bool PID::IsReset() {
	if (!isTwiddle) {
		return false;
	} else if (isInitialized) {
		isInitialized = false;
		return true;
	} else if (isReset) {
		isReset = false;
		num_step = 0;
		return true;
	} else {
		return false;
	}
}
void PID::UpdateBest() {
	best_p[0] = Kp;
	best_p[1] = Ki;
	best_p[2] = Kd;
std::cout << "[UpdateBest] Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd << std::endl;
}
void PID::SetBest() {

	Kp = best_p[0];
	Ki = best_p[1];
	Kd = best_p[2];

std::cout << "[SetBest] Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd << " best error:" << best_error << std::endl;
}
void PID::MoveNextParam() {
	idx_p += 1;
	if (idx_p >= 3) {
		idx_p = 0;
	}
}

void PID::ReflectTwiddleParams() {
	if(idx_p == 0) {
		this->Kp = p[0];
	} else if(idx_p == 1) {
		this->Ki = p[1];
	} else if(idx_p == 2) {
		this->Kd = p[2];
	}
	std::cout << "[Reflect] Kp:" << Kp << " Ki:" << Ki << " Kd:" << Kd << "(Best - Kp:" << best_p[0] << " Ki:" << best_p[1] << " Kd:" << best_p[2] << ")" <<  std::endl;
}

void SteeringControler::Calc() {
	steer_value = TotalError();
	AdjustSteerValue();
}
void SteeringControler::AdjustSteerValue() {
	const double STEER_VAL_MAX = 1;
	const double STEER_VAL_MIN = -1*STEER_VAL_MAX;
	if (steer_value > STEER_VAL_MAX) {
		steer_value = STEER_VAL_MAX;
	} else if (steer_value < STEER_VAL_MIN) {
		steer_value = STEER_VAL_MIN;
	} 
}

void ThrottleControler::Calc() {
	throttle_value = 1 - fabs(TotalError());
	AdjustThrottleValue();
}
void ThrottleControler::AdjustThrottleValue() {
	const double THROTTLE_VAL_MAX = 1;
	const double THROTTLE_VAL_MIN = -1*THROTTLE_VAL_MAX;
	if (throttle_value > THROTTLE_VAL_MAX) {
		throttle_value = THROTTLE_VAL_MAX;
	} else if (throttle_value < THROTTLE_VAL_MIN) {
std::cout << "[AdjustThrottleValue] throttle_value:" << throttle_value << std::endl;
		throttle_value = THROTTLE_VAL_MIN;
	} 
}