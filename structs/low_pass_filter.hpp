#ifndef __LOW_PASS_FILTER_HPP__
#define __LOW_PASS_FILTER_HPP__

class LowPassFilter {
  private:
    double alpha;
    double last;
    bool init;

  public:
    void set_alpha(double alpha) { this->alpha = alpha; }
    void reset() { this->init = false; }
    double last_value() const { return this->last; }
    double filter(const double &input) {
        if (!this->init) {
            this->last = input;
            this->init = true;
        } else
            this->last = this->alpha * input + (1 - this->alpha) * this->last;
        return this->last;
    }
};

#endif