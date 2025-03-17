#ifndef __LOW_PASS_FILTER_HPP__
#define __LOW_PASS_FILTER_HPP__

class LowPassFilter {
  private:
    double alpha_;
    double last_;
    bool initialzed_;

  public:
    void set_alpha(double alpha) { this->alpha_ = alpha; }
    void reset() { this->initialzed_ = false; }
    double last_value() const { return this->last_; }
    double filter(const double &input) {
        if (!this->initialzed_) {
            this->last_       = input;
            this->initialzed_ = true;
        } else
            this->last_ = this->alpha_ * input + (1 - this->alpha_) * this->last_;
        return this->last_;
    }
};

#endif // __LOW_PASS_FILTER_HPP__