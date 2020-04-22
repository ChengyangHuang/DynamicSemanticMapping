#include <algorithm>
#include <assert.h>
#include <iostream>

#include "bkioctree_node.h"

namespace semantic_bki {

    /// Default static values
    int Semantics::num_class = 2;
    float Semantics::sf2 = 1.0f;
    float Semantics::ell = 1.0f;
    float Semantics::prior = 0.5f;
    float Semantics::var_thresh = 1000.0f;
    float Semantics::free_thresh = 0.3f;
    float Semantics::occupied_thresh = 0.7f;

    void Semantics::get_probs(std::vector<float>& probs) const {
      assert (probs.size() == num_class);
      float sum = 0;
      for (auto m : ms)
        sum += m;
      for (int i = 0; i < num_class; ++i)
        probs[i] = ms[i] / sum;
    }

    void Semantics::get_vars(std::vector<float>& vars) const {
      assert (vars.size() == num_class);
      float sum = 0;
      for (auto m : ms)
        sum += m;
      for (int i = 0; i < num_class; ++i)
        vars[i] = ((ms[i] / sum) - (ms[i] / sum) * (ms[i] / sum)) / (sum + 1);
    }

    void Semantics::update(std::vector<float>& ybars) {
      assert(ybars.size() == num_class);
      classified = true;
      for (int i = 0; i < num_class; ++i)
        ms[i] += ybars[i];

      std::vector<float> probs(num_class);
      get_probs(probs);

      semantics = std::distance(probs.begin(), std::max_element(probs.begin(), probs.end()));

      if (semantics == 0)
        state = State::FREE;
      else
        state = State::OCCUPIED;
    }

    void Semantics::set_alphas(std::vector<int> dyn_classes, std::vector<float>& alphas) {
      assert(alphas.size() == num_class);
      for (int i = 0; i < dyn_classes.size(); i++) {
        ms[dyn_classes[i]] = alphas[dyn_classes[i]];
      }
    }

    void Semantics::decay_alphas(std::vector<int> dyn_classes){
      float thres = 10.0;
      for (int i = 0; i < dyn_classes.size(); i++) {
        (ms[dyn_classes[i]] > thres) ? (ms[dyn_classes[i]] - thres) : (ms[dyn_classes[i]] = 0.0);
        // ms[dyn_classes[i]] = 0.0;
        // std::cout << "decay alpha done" << std::endl;
      }
    }
}
