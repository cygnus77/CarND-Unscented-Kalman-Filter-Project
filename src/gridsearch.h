#ifndef GRID_SEARCH
#define GRID_SEARCH
#include <vector>
#include <functional>
#include <algorithm>

// Base class for grid search variables
class var {
public:
  virtual ~var() {};
  // reset variable to initial value
  virtual void init() = 0;
  // has variable reached max value
  virtual bool isDone() const = 0;
  // increment to next value
  // return true on wrap-around
  virtual bool increment() = 0;
};

// Variable range class: pointer to variable and range of values it can take
class var_range : public var {
public:
  double* pvar;
  double start;
  double end;
  double step;

  // constructor - initialize vars
  var_range(double* pvar, double start, double end, double step) : pvar(pvar), start(start), end(end), step(step) { }
  virtual ~var_range() {}

  void init() {
    *pvar = start;
  }

  bool isDone() const {
    return *pvar >= end;
  }

  bool increment() {
    if (*pvar < end) {
      *pvar += step;
      return false;
    }
    else {
      *pvar = start;
      return true;
    }
  }
};

// Variable with fixed number of (predefined) values
class var_enum : public var {
public:
  double *pvar;
  std::vector<double> values;
  int idx = 0;

  // constructor
  var_enum(double* pvar, const std::vector<double>& vals) : pvar(pvar), values(vals) {}
  virtual ~var_enum() {}

  void init() {
    idx = 0;
    *pvar = values[idx];
  }

  bool isDone() const {
    return idx >= values.size() - 1;
  }

  bool increment() {
    bool wrap = false;
    idx++;
    if (idx >= values.size()) {
      idx = 0;
      wrap = true;
    }
    *pvar = values[idx];
    return wrap;
  }
};

class gridsearch {
public:
  // Excute full grid search
  static void doGridSearch(std::vector<var*>& variables, std::function<void(void)> fn)
  {
    // Initialize all variables
    std::for_each(variables.begin(), variables.end(), [](var* x) { x->init(); });
    // Keep going till search is completed
    while (!isGridSearchDone(variables)) {
      // execute function
      fn();
      // increment to next search
      incrementGridSearch(variables);
    }
  }

private:
  // Check to see if all variables have reached their max
  static bool isGridSearchDone(std::vector<var*>& variables) {
    for (int i = 0; i < variables.size(); i++) {
      if (!variables[i]->isDone())
        return false;
    }
    return true;
  }

  // Perform one increment on the grid search
  static void incrementGridSearch(std::vector<var*>& variables) {
    for (int i = 0; i < variables.size(); i++) {
      if (!variables[i]->increment())
        return;
    }
  }

};
#endif