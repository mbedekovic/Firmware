/*
 * AttitudeFeedForward.h
 *
 *  Created on: Jun 26, 2017
 *      Author: matija
 */

#ifndef ATTITUDEFEEDFORWARD_H_
#define ATTITUDEFEEDFORWARD_H_

class AttitudeFeedForward {
public:
    float u_ff_;
    float u_ff_old_;
    float ud_old_;
    float Kff_;
    float b1_, a1_;
    float Tm_, Ts_;

    //methods for setting gain and feedforward model params
    void setFeedForwardGain(float kff);
    void setFeedForwardModel(float timeConstant, float sampleTime, float modelFasterFactor);

    //feedforward output compute
    float feedForwardCompute(float ud);

    AttitudeFeedForward();
    ~AttitudeFeedForward();
};

#endif /* ATTITUDEFEEDFORWARD_H_ */
